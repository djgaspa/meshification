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
#include "SurfaceReconstruction.hpp"
#include "MeshBuilder.hpp"

static inline
bool is_inner(const cv::Point& p, const SurfaceReconstruction::Cloud& cloud, const cv::Size& size)
{
    return cloud[p.x + p.y * size.width].z != 0.0;
}

static inline
bool is_joinable(const cv::Point& p1, const cv::Point& p2, const SurfaceReconstruction::Cloud& cloud, const cv::Size& size)
{
    if (::is_inner(p1, cloud, size) == false || ::is_inner(p2, cloud, size) == false)
        return false;
    //return true;
    const auto depth_diff = std::fabs(cloud[p1.x + p1.y * size.width].z - cloud[p2.x + p2.y * size.width].z);
    return depth_diff < 0.150;
}

static inline
cv::Point intersect(const cv::Point& p1, const cv::Point& p2, const SurfaceReconstruction::Cloud& cloud, const cv::Size& size)
{
    if (std::abs(p1.x - p2.x) <= 1 && std::abs(p1.y - p2.y) <= 1)
        return ::is_joinable(p1, p2, cloud, size) == true ? p2 : p1;
    const cv::Point mid = (p1 + p2) * 0.5;
    return ::is_joinable(p1, mid, cloud, size) == true ? ::intersect(mid, p2, cloud, size) : ::intersect(p1, mid, cloud, size);
}

static inline
void extend_point(std::vector<cv::Point>& tri, const SurfaceReconstruction::Cloud& cloud, const cv::Size& size)
{
    for (int j = 0; j < 2; ++j)
        tri[1 + j] = intersect(tri[0], tri[1 + j], cloud, size);
}

static inline
void extend_edge(std::vector<cv::Point>& tri, std::vector<cv::Point>& new_tri, const SurfaceReconstruction::Cloud& cloud, const cv::Size& size)
{
    new_tri.resize(3);
    for (int j = 0; j < 2; ++j)
        new_tri[1 - j] = intersect(tri[j], tri[2], cloud, size);
    double diagonal[2];
    diagonal[0] = cv::norm(cloud[tri[0].x + tri[0].y * size.width] - cloud[new_tri[0].x + new_tri[0].y * size.width]);
    diagonal[1] = cv::norm(cloud[tri[1].x + tri[1].y * size.width] - cloud[new_tri[1].x + new_tri[1].y * size.width]);
    const int d = diagonal[0] < diagonal[1] ? 0 : 1;
    tri[2] = new_tri[d];
    new_tri[2] = tri[d];
}

static void draw_triangle(const std::vector<cv::Point>& p, cv::Mat& mat)
{
    line(mat, p[0], p[1], 255);
    line(mat, p[1], p[2], 255);
    line(mat, p[2], p[0], 255);
}

SurfaceReconstruction::SurfaceReconstruction()
{}

SurfaceReconstruction::~SurfaceReconstruction()
{}

void SurfaceReconstruction::operator()(const std::vector<cv::Vec6f>& triangles, const Cloud& cloud, const cv::Size& size)
{
    mesh_builder_.reset(new MeshBuilder(cloud, size));
    mesh2d = cv::Mat(size, CV_8UC1);
    mesh2d.setTo(0);
    for (const auto& t : triangles) {
        std::vector<cv::Point> pt(3);
        for (int j = 0; j < 3; ++j)
            pt[j] = cv::Point(t[2 * j], t[2 * j + 1]);
        int n_inner = 0;
        std::vector<bool> inner(3);
        for (int j = 0; j < 3; ++j)
            if (::is_inner(pt[j], cloud, size) == true) {
                ++n_inner;
                inner[j] = true;
            }
        if (n_inner == 0)
            continue;
        else if (n_inner == 1) {
            if (inner[1] == true) {
                std::swap(pt[0], pt[1]);
                std::swap(pt[1], pt[2]);
            } else if (inner[2] == true) {
                std::swap(pt[0], pt[2]);
                std::swap(pt[1], pt[2]);
            }
            extend_point(pt, cloud, size);
            mesh_builder_->insert(pt);
            //draw_triangle(pt, mesh2d);
        } else if (n_inner == 2) {
            if (inner[0] == false) {
                std::swap(pt[2], pt[0]);
                std::swap(pt[0], pt[1]);
            } else if (inner[1] == false) {
                std::swap(pt[2], pt[1]);
                std::swap(pt[0], pt[1]);
            }
            std::vector<cv::Point> pt2(3);
            if (::is_joinable(pt[0], pt[1], cloud, size)) {
                extend_edge(pt, pt2, cloud, size);
            } else {
                std::rotate_copy(pt.begin(), pt.begin() + 1, pt.end(), pt2.begin());
                extend_point(pt, cloud, size);
                extend_point(pt2, cloud, size);
            }
            mesh_builder_->insert(pt);
            //draw_triangle(pt, mesh2d);
            mesh_builder_->insert(pt2);
            //draw_triangle(pt2, mesh2d);
        } else {
            const bool ab = ::is_joinable(pt[0], pt[1], cloud, size);
            const bool bc = ::is_joinable(pt[1], pt[2], cloud, size);
            const bool ac = ::is_joinable(pt[0], pt[2], cloud, size);
            if ((ab && bc) || (ab && ac) || (bc && ac)) {
                mesh_builder_->insert(pt);
                //draw_triangle(pt, mesh2d);
                continue;
            }
            std::vector<cv::Point> pt2(3), pt3(3);
            if (ab) {
                std::rotate_copy(pt.begin(), pt.begin() + 2, pt.end(), pt2.begin());
                extend_point(pt2, cloud, size);
                extend_edge(pt, pt3, cloud, size);
            } else if (bc) {
                std::rotate_copy(pt.begin(), pt.begin() + 1, pt.end(), pt2.begin());
                extend_point(pt, cloud, size);
                extend_edge(pt2, pt3, cloud, size);
            } else if (ac) {
                std::rotate_copy(pt.begin(), pt.begin() + 1, pt.end(), pt2.begin());
                extend_point(pt2, cloud, size);
                std::rotate(pt.begin(), pt.begin() + 2, pt.end());
                extend_edge(pt, pt3, cloud, size);
            } else {
                std::rotate_copy(pt.begin(), pt.begin() + 1, pt.end(), pt2.begin());
                extend_point(pt2, cloud, size);
                std::rotate_copy(pt.begin(), pt.begin() + 2, pt.end(), pt3.begin());
                extend_point(pt3, cloud, size);
                extend_point(pt, cloud, size);
            }
            mesh_builder_->insert(pt);
            //draw_triangle(pt, mesh2d);
            mesh_builder_->insert(pt2);
            //draw_triangle(pt2, mesh2d);
            mesh_builder_->insert(pt3);
            //draw_triangle(pt3, mesh2d);
        }
    }
    //cv::imshow("2D Mesh", mesh2d);
}

void SurfaceReconstruction::write(const std::string& filename)
{
    if (mesh_builder_.get() != 0)
        mesh_builder_->write(filename);
    cv::imwrite("mesh2d.png", mesh2d);
}
