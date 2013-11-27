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

#include <iosfwd>
#include <vector>
#include <string>
#include <opencv/cv.hpp>

class MeshBuilder
{
    const cv::Size size;
    cv::Mat indices_;
    std::vector<float> ver_;
    std::vector<unsigned> tri_;
    const std::vector<cv::Point3f>& cloud;

public:
    MeshBuilder(const std::vector<cv::Point3f>& cloud, const cv::Size& size);
    ~MeshBuilder();
    void insert(const std::vector<cv::Point>& p);
    void write(std::ostream& out) const;
    void write(const std::string& filename) const;
    const std::vector<unsigned>& get_triangles() const {
        return tri_;
    }
    const std::vector<float>& get_vertices() const {
        return ver_;
    }
};
