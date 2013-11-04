#pragma once
#include <vector>
#include <string>

struct Data3d
{
    std::vector<unsigned> tri;
    std::vector<float> ver;
    std::vector<unsigned char> y_img, u_img, v_img;
    const int width, height;
    float center_x, center_y, focal_x, focal_y;
    double modelview[16];
    std::string name;

    Data3d(int w, int h) :
        y_img(w * h), u_img(w * h / 4), v_img(w * h / 4),
        width(w), height(h)
    {}
};
