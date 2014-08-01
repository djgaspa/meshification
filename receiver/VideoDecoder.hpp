#pragma once

class VideoDecoder
{
    struct Impl;
    Impl* p_;

    int n_frames;

public:
    VideoDecoder();
    ~VideoDecoder();
    void operator()(const char* data, const int size, unsigned char* y_img, unsigned char* u_img, unsigned char* v_img);
    int get_n_frames() const { return n_frames; }
};
