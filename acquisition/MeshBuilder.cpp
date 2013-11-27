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

#include <fstream>
#include <sstream>
#include <stdexcept>
#include "MeshBuilder.hpp"

MeshBuilder::MeshBuilder(const std::vector<cv::Point3f>& cloud, const cv::Size& size) :
    size(size),
    indices_(size, CV_16SC1),
    cloud(cloud)
{
    indices_.setTo(-1);
    ver_.reserve(10000);
    tri_.reserve(10000);
    if (cloud.size() < size.width * size.height) {
        std::ostringstream error;
        error << "Cloud contains " << cloud.size() << " points, but the range image size is " << size.width << 'x' << size.height << '.';
        throw std::logic_error(error.str());
    }
}

MeshBuilder::~MeshBuilder()
{}

void MeshBuilder::insert(const std::vector<cv::Point>& p)
{
    if (p[0] == p[1] || p[1] == p[2] || p[0] == p[2])
        return;
    std::vector<unsigned> idx(3);
    for (int i = 0; i < 3; ++i) {
        idx[i] = indices_.at<short>(p[i]);
        if (idx[i] == -1) {
            const auto p3d = cloud[p[i].x + size.width * p[i].y];
            ver_.push_back(p3d.x);
            ver_.push_back(p3d.y);
            ver_.push_back(p3d.z);
            idx[i] = ver_.size() / 3 - 1;
            indices_.at<short>(p[i]) = idx[i];
        }
    }
    std::copy(idx.begin(), idx.end(), std::back_insert_iterator<std::vector<unsigned> >(tri_));
}

void MeshBuilder::write(std::ostream& out) const
{
    out << "OFF\n" << ver_.size() / 3 << ' ' << tri_.size() / 3 << " 0\n";
    for (std::size_t i = 0; i < ver_.size() / 3; ++i)
        out << ver_[3 * i + 0] << ' ' << ver_[3 * i + 1] << ' ' << ver_[3 * i + 2] << '\n';
    for (std::size_t i = 0; i < tri_.size() / 3; ++i)
        out << "3 " << tri_[3 * i + 2] << ' ' << tri_[3 * i + 1] << ' ' << tri_[3 * i + 0] << '\n';
}

void MeshBuilder::write(const std::string& filename) const
{
    std::ofstream out(filename.c_str());
    if (out.is_open() == false) {
        std::cerr << "Unable to write to " << filename << std::endl;
        return;
    }
    write(out);
}
