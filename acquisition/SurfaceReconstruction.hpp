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

#include <memory>
#include <vector>
#include <string>
#include <opencv/cv.hpp>

class MeshBuilder;

class SurfaceReconstruction
{
    std::auto_ptr<MeshBuilder> mesh_builder_;
    cv::Mat mesh2d;

public:
    using Cloud = std::vector<cv::Point3f>;
    SurfaceReconstruction();
    ~SurfaceReconstruction();
    void operator()(const std::vector<cv::Vec6f>& triangles, const Cloud& cloud, const cv::Size& size);
    void write(const std::string& filename);
    const MeshBuilder& mesh() const {
        return *mesh_builder_;
    }
};
