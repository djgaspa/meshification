#pragma once
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

#include <opencv2/opencv.hpp>

class Triangulator
{
    struct Impl;
    Impl* p_;
    const double min_area_, depth_coefficient_;

public:
    Triangulator(const cv::Mat& depth, double min_area = 10.0, double depth_coefficient = 5.0);
    ~Triangulator();
    void add_contour(const std::vector<cv::Point>& contour);
    void add_hole(const cv::Point2d&);
    void operator()();
    void draw(cv::Mat& output) const;
    std::vector<cv::Vec6f> get_triangles() const;
};
