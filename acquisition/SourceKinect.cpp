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

#include <stdexcept>
#include <opencv2/opencv.hpp>
#include "SourceKinect.hpp"

SourceKinect::SourceKinect(const int id) :
    kinect(new cv::VideoCapture(CV_CAP_OPENNI + id))
{
    if (kinect->isOpened() == false)
        throw std::runtime_error("Unable to open the Kinect device");
}

SourceKinect::~SourceKinect()
{}

void SourceKinect::grab(char* rgb, char* depth)
{
    cv::Mat rgb_mat, depth_mat;
    kinect->grab();
    kinect->retrieve(rgb_mat, CV_CAP_OPENNI_BGR_IMAGE);
    kinect->retrieve(depth_mat, CV_CAP_OPENNI_DEPTH_MAP);
    cv::cvtColor(rgb_mat, rgb_mat, CV_BGR2RGB);
    std::copy(rgb_mat.ptr(), rgb_mat.ptr() + 3 * 640 * 480, rgb);
    std::copy(depth_mat.ptr(), depth_mat.ptr() + 2 * 640 * 480, depth);
}

int SourceKinect::width() const
{
    return kinect->get(CV_CAP_PROP_FRAME_WIDTH);
}

int SourceKinect::height() const
{
    return kinect->get(CV_CAP_PROP_FRAME_HEIGHT);
}
