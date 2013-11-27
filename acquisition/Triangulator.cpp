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

#include <vector>
#include <unordered_set>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#define REAL double
#define VOID void
#define ANSI_DECLARATORS
#include "triangle.h"
#include "Triangulator.hpp"

using Cloud = std::vector<cv::Point3f>;

inline
bool isValid(const Cloud& c, const cv::Size& size, int x, int y)
{
    return x >= 0 && x < size.width && y >= 0 && y < size.height && c[size.width * y + x].z != 0;
}

inline
cv::Point3f getPoint(const Cloud& c, const cv::Size& size, int x, int y)
{
    return c[size.width * y + x];
}

inline
Eigen::Vector3f getEigenVector(const Cloud& c, const cv::Size& size,  int x, int y)
{
    const auto& p = c[size.width * y + x];
    return Eigen::Vector3f(p.x, p.y, p.z);
}

inline
void getImagePoint(const Eigen::Vector3f& p, const cv::Mat& camera_matrix, int& x, int& y)
{
    x = camera_matrix.at<double>(0) * p[0] / p[2] + camera_matrix.at<double>(2);
    y = camera_matrix.at<double>(4) * p[1] / p[2] + camera_matrix.at<double>(5);
}

int triunsuitable(REAL* triorg, REAL* tridest, REAL* triapex, REAL area)
{
    return Triangulator::triunsuitable(triorg, tridest, triapex, area);
}

Triangulator* Triangulator::current = nullptr;

int Triangulator::triunsuitable_impl(REAL* triorg, REAL* tridest, REAL* triapex, REAL area)
{
    if (area < min_area)
        return 0;
    if (cloud.empty())
        return 0;
    const float x0 = static_cast<float>(triorg[0]), y0 = static_cast<float>(triorg[1]);
    const float x1 = static_cast<float>(tridest[0]), y1 = static_cast<float>(tridest[1]);
    const float x2 = static_cast<float>(triapex[0]), y2 = static_cast<float>(triapex[1]);
    const float xm = (x0 + x1 + x2) / 3.0f;
    const float ym = (y0 + y1 + y2) / 3.0f;
    if (::isValid(cloud, size, xm, ym) == false && ::isValid(cloud, size, x0, y0) == false && ::isValid(cloud, size, x1, y1) == false && ::isValid(cloud, size, x2, y2) == false)
        return 0;
    if (::isValid(cloud, size, xm, ym) == false || ::isValid(cloud, size, x0, y0) == false || ::isValid(cloud, size, x1, y1) == false || ::isValid(cloud, size, x2, y2) == false)
        return area > 50;
    const auto p03 = ::getEigenVector(cloud, size, x0, y0);
    const auto p13 = ::getEigenVector(cloud, size, x1, y1);
    const auto p23 = ::getEigenVector(cloud, size, x2, y2);
    const auto pm3 = (p03 + p13 + p23) / 3.0f;
    int pmx, pmy;
    ::getImagePoint(pm3, camera_matrix, pmx, pmy);
    if (::isValid(cloud, size, pmx, pmy) == false)
        return 1;
    const auto actual_pm = ::getEigenVector(cloud, size, pmx, pmy);
    const auto n = (p13 - p03).cross(p23 - p03);
    const float d = -p03.dot(n);
    const float n_norm = n.norm();
    const double area3d = n_norm / 2.0;
    if (area3d < 10e-4)
        return 0;
    const float distance = std::abs((actual_pm.dot(n) + d) / n_norm);
    //const double centre_diff = (pm3 - actual_pm).norm();
    //const double sqrt3 = 1.7320508075688772;
    //const double side = std::sqrt(area3d * 4.0 / sqrt3);
    //std::cout << centre_diff << ' ' << std::abs(zm - pm3[2]) << ' ' << distance << ' ' << side << std::endl;
    //if (centre_diff < distance)
    //std::cout << "AAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
    //std::cout << "Area: " << area3d << ' ' << (centre_diff / side) << ' ' << (depth_coefficient / 100.0) << std::endl;
    //if ((centre_diff / side) > (depth_coefficient / 100.0))
    if ((distance * 1000.0) > depth_coefficient)
        //if ((pm3 - actual_pm).norm() * 1000.0 > (zm * zm * depth_coefficient))
        return 1;
    return 0;
    REAL dxoa, dxda, dxod;
    REAL dyoa, dyda, dyod;
    REAL oalen, dalen, odlen;
    REAL maxlen;

    dxoa = triorg[0] - triapex[0];
    dyoa = triorg[1] - triapex[1];
    dxda = tridest[0] - triapex[0];
    dyda = tridest[1] - triapex[1];
    dxod = triorg[0] - tridest[0];
    dyod = triorg[1] - tridest[1];
    /* Find the squares of the lengths of the triangle's three edges. */
    oalen = dxoa * dxoa + dyoa * dyoa;
    dalen = dxda * dxda + dyda * dyda;
    odlen = dxod * dxod + dyod * dyod;
    /* Find the square of the length of the longest edge. */
    maxlen = (dalen > oalen) ? dalen : oalen;
    maxlen = (odlen > maxlen) ? odlen : maxlen;

    if (maxlen > 0.05 * (triorg[0] * triorg[0] + triorg[1] * triorg[1]) + 0.02) {
        return 1;
    } else {
        return 0;
    }
}

class Segment
{
    unsigned a_, b_;
public:
    Segment(unsigned a, unsigned b) :
        a_(std::min(a, b)),
        b_(std::max(a, b))
    {}
    unsigned a() const {
        return a_;
    }
    unsigned b() const {
        return b_;
    }
    bool operator==(const Segment& s) const {
        return s.a() == a() && s.b() == b();
    }
};

struct SegmentHasher
{
    std::size_t operator()(const Segment& s) const {
        return std::hash<unsigned>()(s.a()) ^ std::hash<unsigned>()(s.b());
    }
};

struct Triangulator::Impl
{
    cv::Mat indices;
    std::unordered_set<Segment, SegmentHasher> segment_indices;
    std::vector<REAL> points, holes;
    std::vector<int> segments;
    triangulateio in, out;
    std::vector<unsigned> triangles;
    const std::vector<cv::Point3f>& cloud;
    Impl(const std::vector<cv::Point3f>& cloud, const cv::Size& size);
    ~Impl();
};

Triangulator::Impl::Impl(const std::vector<cv::Point3f>& cloud, const cv::Size& size):
    indices(size, CV_32SC1),
    cloud(cloud)
{
    indices.setTo(-1);
    in.numberofpoints = 0;
    in.pointlist = 0;
    in.pointmarkerlist = 0;
    in.numberofpointattributes = 0;
    in.numberofsegments = 0;
    in.segmentlist = 0;
    in.segmentmarkerlist = 0;
    in.numberofholes = 0;
    in.numberofregions = 0;
    out.pointlist = 0;
    out.pointmarkerlist = 0;
    out.trianglelist = 0;
    out.segmentlist = 0;
    out.segmentmarkerlist = 0;
    out.numberoftriangles = 0;
    segment_indices.rehash(1000);
}

Triangulator::Impl::~Impl()
{
    trifree(out.pointlist);
    trifree(out.pointmarkerlist);
    trifree(out.trianglelist);
    trifree(out.segmentlist);
    trifree(out.segmentmarkerlist);
}

Triangulator::Triangulator(const std::vector<cv::Point3f>& cloud, const cv::Size& size, const cv::Mat& camera_matrix, double min_area, double depth_coefficient) :
    p_(new Impl(cloud, size)),
    size(size),
    cloud(cloud),
    camera_matrix(camera_matrix),
    min_area(min_area),
    depth_coefficient(depth_coefficient)
{
    std::vector<cv::Point> frame {
        cv::Point(0, 0),
                cv::Point(size.width - 1, 0),
                cv::Point(size.width - 1, size.height - 1),
                cv::Point(0, size.height - 1)
    };
    add_contour(frame);
}

Triangulator::~Triangulator()
{
    delete p_;
}

void Triangulator::add_contour(const std::vector<cv::Point>& contour)
{
    if (contour.size() < 3)
        return;
    const size_t n_points_prev = p_->points.size() / 2, n_points = n_points_prev + contour.size();
    p_->points.reserve(n_points * 2);
    for (size_t i = 0; i < contour.size(); ++i) {
        const cv::Point& p = contour[i];
        int& idx = p_->indices.at<int>(p);
        if (idx == -1) {
            p_->points.push_back(p.x);
            p_->points.push_back(p.y);
            idx = p_->points.size() / 2 - 1;
        }
    }
    /*p_->segments.reserve(n_points * 2);
    for (size_t i = 0; i < contour.size(); ++i) {
        const int i1 = i, i2 = i == contour.size() - 1 ? 0 : i + 1;
        const int& idx1 = p_->indices.at<int>(contour[i1]);
        const int& idx2 = p_->indices.at<int>(contour[i2]);
        if (p_->segment_indices.insert(Segment(idx1, idx2)).second == true) {
            p_->segments.push_back(idx1);
            p_->segments.push_back(idx2);
        }
    }*/
    p_->in.numberofpoints = p_->points.size() / 2;
    p_->in.pointlist = &p_->points[0];
    p_->in.numberofsegments = p_->segments.size() / 2;
    p_->in.segmentlist = &p_->segments[0];
}

void Triangulator::add_hole(const cv::Point2d& p)
{
    ++p_->in.numberofholes;
    p_->holes.push_back(p.x);
    p_->holes.push_back(p.y);
    p_->in.holelist = &p_->holes[0];
}

void Triangulator::operator()()
try {
    std::ostringstream flags_stream;
    flags_stream << "QXzu";
    const std::string flags = flags_stream.str();
    current = this;
    triangulate(const_cast<char*>(flags.c_str()), &p_->in, &p_->out, 0);
} catch (int) {
    p_->out.numberoftriangles = 0;
}

static cv::Point median_point(const cv::Point* p)
{
    cv::Point pm = p[0] + p[1] + p[2];
    pm.x /= 3;
    pm.y /= 3;
    return pm;
}

void Triangulator::draw(cv::Mat& output) const
{
    for (int i = 0; i < p_->out.numberoftriangles; ++i) {
        const int* tri_p = &p_->out.trianglelist[3 * i];
        cv::Point p[3];
        for (int j = 0; j < 3; ++j) {
            const REAL* ptr = &p_->out.pointlist[2 * tri_p[j]];
            p[j] = cv::Point(ptr[0], ptr[1]);
        }
        cv::Scalar color(0, 200, 0);
        if (::isValid(cloud, size, p[0].x, p[0].y) && ::isValid(cloud, size, p[1].x, p[1].y) && ::isValid(cloud, size, p[2].x, p[2].y)) {
            color = cv::Scalar(0, 0, 200);
            const auto pm = ::median_point(p);
            if (::isValid(cloud, size, pm.x, pm.y))
                cv::circle(output, pm, 1, cv::Scalar(0, 200, 0));
        }
        if (::isValid(cloud, size, p[0].x, p[0].y) == false && ::isValid(cloud, size, p[1].x, p[1].y) == false && ::isValid(cloud, size, p[2].x, p[2].y) == false)
            color = cv::Scalar(200, 0, 0);
        cv::line(output, p[0], p[1], color);
        cv::line(output, p[1], p[2], color);
        cv::line(output, p[0], p[2], color);
    }
}

std::vector<cv::Vec6f> Triangulator::get_triangles() const
{
    std::vector<cv::Vec6f> triangles;
    triangles.reserve(p_->out.numberoftriangles);
    for (int i = 0; i < p_->out.numberoftriangles; ++i) {
        const int* tri_p = &p_->out.trianglelist[3 * i];
        cv::Point2f p[3];
        for (int j = 0; j < 3; ++j) {
            const REAL* ptr = &p_->out.pointlist[2 * tri_p[j]];
            p[j] = cv::Point2f(ptr[0], ptr[1]);
        }
        if (!std::isfinite(p[0].x) || !std::isfinite(p[0].y) ||
                !std::isfinite(p[1].x) || !std::isfinite(p[1].y) ||
                !std::isfinite(p[2].x) || !std::isfinite(p[2].y))
            continue;
        if (::isValid(cloud, size, p[0].x, p[0].y) == false &&
                ::isValid(cloud, size, p[1].x, p[1].y) == false &&
                ::isValid(cloud, size, p[2].x, p[2].y) == false)
            continue;
        cv::Vec6f t;
        for (int j = 0; j < 3; ++j) {
            const REAL* ptr = &p_->out.pointlist[2 * tri_p[j]];
            t[2 * j + 0] = ptr[0];
            t[2 * j + 1] = ptr[1];
        }
        triangles.push_back(t);
    }
    return triangles;
}

int Triangulator::triunsuitable(double* triorg, double* tridest, double* triapex, double area)
{
    if (current == nullptr)
        return 0;
    return current->triunsuitable_impl(triorg, tridest, triapex, area);
}
