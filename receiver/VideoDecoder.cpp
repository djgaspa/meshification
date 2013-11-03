#include <iostream>
#include <stdexcept>
#include <vector>
#define VPX_CODEC_DISABLE_COMPAT 1
#include <vpx/vpx_decoder.h>
#include <vpx/vp8dx.h>
#include "VideoDecoder.hpp"

inline static void check(const vpx_codec_err_t& err)
{
    if (err)
	throw std::runtime_error(vpx_codec_err_to_string(err));
}

struct VideoDecoder::Impl
{
    vpx_codec_ctx_t ctx;
    vpx_codec_iter_t iter;
    Impl() : iter(0) {}
};

VideoDecoder::VideoDecoder() :
    p_(new Impl),
    n_frames(0)
{
    check(vpx_codec_dec_init(&p_->ctx, vpx_codec_vp8_dx(), 0, 0));
}

VideoDecoder::~VideoDecoder()
{
    vpx_codec_err_t err = vpx_codec_destroy(&p_->ctx);
    if (err)
	std::cerr << "Error destroying VP8 decoder context: " << vpx_codec_err_to_string(err) << std::endl;
    delete p_;
}

void VideoDecoder::operator()(std::istream &in, unsigned char *y_img, unsigned char *u_img, unsigned char *v_img)
{
    if (!in)
        throw std::logic_error("Error reading from video stream");
    unsigned size;
    in.read((char*)&size, sizeof(size));
    std::vector<unsigned char> tmp(size);
    in.read((char*)&tmp[0], tmp.size());
    check(vpx_codec_decode(&p_->ctx, &tmp[0], tmp.size(), 0, 0));
    p_->iter = 0;
    auto* frame = vpx_codec_get_frame(&p_->ctx, &p_->iter);
    if (frame == 0)
        throw std::logic_error("Error decoding VPX stream. Frame not available.");
    const int width = frame->d_w, height = frame->d_h;
    for (int i = 0; i < width; ++i)
        for (int j = 0; j < height; ++j)
           y_img[i + j * width] = frame->planes[0][i + j * frame->stride[0]];
    const int c_width = width / 2, c_height = height / 2;
    for (int i = 0; i < c_width; ++i)
        for (int j = 0; j < c_height; ++j) {
            v_img[i + j * c_width] = frame->planes[1][i + j * frame->stride[1]];
            u_img[i + j * c_width] = frame->planes[2][i + j * frame->stride[2]];
        }
}
