#pragma once
#include <iosfwd>

class VideoDecoder
{
    struct Impl;
    Impl* p_;

    int n_frames;

public:
    VideoDecoder();
    ~VideoDecoder();
    void operator()(std::istream& in, unsigned char* y_img, unsigned char* u_img, unsigned char* v_img);
};
