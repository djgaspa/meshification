/*
    Copyright (C) 2011-2012 Paolo Simone Gasparello <p.gasparello@sssup.it>

    This file is part of meshificator.

    meshificator is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    meshificator is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with meshificator.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <stdexcept>
#define VPX_CODEC_DISABLE_COMPAT 1
#include <vpx/vpx_encoder.h>
#include <vpx/vp8cx.h>
#include "VideoEncoder.hpp"

static void rgb2yuv(vpx_image_t *raw, int sx, int sy, const unsigned char *source)
{
    unsigned int i, j;
    unsigned int offset1;
    unsigned int offset2;
    for (j = 0; j < sy; ++j)
        for (i = 0; i < sx; ++i) {
            raw->planes[0][i+j*sx]=((int)(114*source[3*(i+j*sx)])+(int)(587*source[3*(i+j*sx)+1])+(int)(299*source[3*(i+j*sx)+2]))>>10;;
        }

    int u, v = 0;
    int a = 0;
    for (j = 0;j < sy; j = j + 2) //This loop samples each 2 pixels (both in X and Y)
        for (i = 0; i < sx; i = i + 2) {
            offset1=3*(i+j*sx);
            u=(((500*source[offset1]) - (419*source[offset1+1]) - (81*source[offset1+2])+128000)>>10);
            v=((-(169*source[offset1]) - (331*source[offset1+1]) + (500*source[offset1+2])+128000)>>10);
            raw->planes[1][a]=u;
            raw->planes[2][a++]=v;
        }
}

class Frame
{
    vpx_image_t frame;
public:
    Frame(const int w, const int h) {
        vpx_img_alloc(&frame, VPX_IMG_FMT_I420, w, h, 0);
    }
    vpx_image_t* get() {
        return &frame;
    }
    ~Frame() {
        vpx_img_free(&frame);
    }
    Frame& operator=(const char* buffer) {
        ::rgb2yuv(&frame, 640, 480, (const unsigned char*)buffer);
    }
};

inline static void check(const vpx_codec_err_t& err)
{
    if (err)
        throw std::runtime_error(vpx_codec_err_to_string(err));
}

struct VideoEncoder::Impl
{
    vpx_codec_ctx_t ctx;
    Frame frame;
    Impl(const int w, const int h) :
        frame(w, h)
    {}
};

VideoEncoder::VideoEncoder(const int w, const int h) :
    p_(new Impl(w, h)),
    n_frames(0)
{
    vpx_codec_enc_cfg_t cfg;
    check(vpx_codec_enc_config_default(vpx_codec_vp8_cx(), &cfg, 0));
    cfg.g_error_resilient = 1;
    cfg.g_lag_in_frames = 0;
    cfg.kf_max_dist = 99;
    cfg.g_w = w;
    cfg.g_h = h;
    cfg.rc_target_bitrate = 256;
    cfg.g_threads = 2;
    check(vpx_codec_enc_init(&p_->ctx, vpx_codec_vp8_cx(), &cfg, 0));
}

VideoEncoder::~VideoEncoder()
{
    vpx_codec_err_t err = vpx_codec_destroy(&p_->ctx);
    if (err)
        std::cerr << "Error destroying VP8 encoder context: " << vpx_codec_err_to_string(err) << std::endl;
    delete p_;
}

void VideoEncoder::operator()(std::ostream& out, const char* buffer)
{
    p_->frame = buffer;
    check(vpx_codec_encode(&p_->ctx, p_->frame.get(), n_frames, 1, 0, VPX_DL_REALTIME));
    const vpx_codec_cx_pkt_t* pkt;
    vpx_codec_iter_t iter = 0;
    while (pkt = vpx_codec_get_cx_data(&p_->ctx, &iter)) {
        if (pkt->kind == VPX_CODEC_CX_FRAME_PKT) {
            const unsigned size = pkt->data.frame.sz;
            out.write((const char*)&size, sizeof(size));
            out.write((const char*)pkt->data.frame.buf, size);
        }
    }
}
