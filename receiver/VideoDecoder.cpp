#include <iostream>
#include <stdexcept>
#include <vector>
#define VPX_CODEC_DISABLE_COMPAT 1
#include <vpx/vpx_decoder.h>
#include <vpx/vp8dx.h>
#include "VideoDecoder.hpp"

void yuv2rgb(const vpx_image_t* source, unsigned char* destination, int flags)
{
    int sx=source->d_w;
    int sy=source->d_h;

    int i,j;
    int y;
    int u=128;
    int v=128;
    int w;
    int r,g,b;
    //int size = source->w;
    int size = source->stride[0];
    int a=0;int c=0;
    int c1,c2,c3;


    for (j = 0; j < sy; ++j) {
	for (i = 0; i < sx; ++i) {
	    if (a == 0) {	
		w=(i/2+j/2*size/2);
		u=source->planes[2][w];
		v=source->planes[1][w];
		c1=(1140*(v-128))>>10;
		c2=(581*(v-128) + 395*(u-128))>>10;
		c3=(2032*(u-128))>>10;
		a++;
	    } else
		a = 0;

	    y=source->planes[0][i+j*size];
	    r=y +c1; g=y -c2; b=y +c3;

	    if(r>255) r=255;
	    if(g>255) g=255;
	    if(b>255) b=255;

	    if(r<0) r=0;
	    if(g<0) g=0;
	    if(b<0) b=0;

	    if (flags == 0) {
		destination[c++]=r;
		destination[c++]=g;
		destination[c++]=b;
		//destination[c++]=255;
	    } else {
		if (y < 16)
		    destination[4*(i+j*sx)+3]=0;
		else
		    destination[4*(i+j*sx)+3]=255;
		destination[4*(i+j*sx)+2]=r;
		destination[4*(i+j*sx)+1]=g;
		destination[4*(i+j*sx)+0]=b;
	    }
	}
    }
}

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

void VideoDecoder::operator()(std::istream& in, unsigned char* buffer)
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
    yuv2rgb(frame, buffer, 0);
}
