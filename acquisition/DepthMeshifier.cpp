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

#include <opencv/cv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/range_image/range_image_planar.h>
#include "DepthMeshifier.hpp"
#include "Triangulator.hpp"
#include "MeshBuilder.hpp"
#include "SurfaceReconstruction.hpp"
#include "DepthFilter.hpp"
#include "../common/AsyncWorker.hpp"

void my_canny(cv::InputArray _src, cv::OutputArray _dst,
               double low_thresh, double high_thresh,
               int aperture_size, bool L2gradient )
{
    cv::Mat src = _src.getMat();
    CV_Assert( src.depth() == CV_8U );

    _dst.create(src.size(), CV_8U);
    cv::Mat dst = _dst.getMat();

    if (!L2gradient && (aperture_size & CV_CANNY_L2_GRADIENT) == CV_CANNY_L2_GRADIENT)
    {
        //backward compatibility
        aperture_size &= ~CV_CANNY_L2_GRADIENT;
        L2gradient = true;
    }

    if ((aperture_size & 1) == 0 || (aperture_size != -1 && (aperture_size < 3 || aperture_size > 7)))
        CV_Error(CV_StsBadFlag, "");

#ifdef HAVE_TEGRA_OPTIMIZATION
    if (tegra::canny(src, dst, low_thresh, high_thresh, aperture_size, L2gradient))
        return;
#endif

    const int cn = src.channels();
    cv::Mat dx(src.rows, src.cols, CV_16SC(cn));
    cv::Mat dy(src.rows, src.cols, CV_16SC(cn));

    cv::Sobel(src, dx, CV_16S, 2, 0, aperture_size, 1, 0, cv::BORDER_REPLICATE);
    cv::Sobel(src, dy, CV_16S, 0, 2, aperture_size, 1, 0, cv::BORDER_REPLICATE);

    if (low_thresh > high_thresh)
        std::swap(low_thresh, high_thresh);

    if (L2gradient)
    {
        low_thresh = std::min(32767.0, low_thresh);
        high_thresh = std::min(32767.0, high_thresh);

        if (low_thresh > 0) low_thresh *= low_thresh;
        if (high_thresh > 0) high_thresh *= high_thresh;
    }
    int low = cvFloor(low_thresh);
    int high = cvFloor(high_thresh);

    ptrdiff_t mapstep = src.cols + 2;
    cv::AutoBuffer<uchar> buffer((src.cols+2)*(src.rows+2) + cn * mapstep * 3 * sizeof(int));

    int* mag_buf[3];
    mag_buf[0] = (int*)(uchar*)buffer;
    mag_buf[1] = mag_buf[0] + mapstep*cn;
    mag_buf[2] = mag_buf[1] + mapstep*cn;
    memset(mag_buf[0], 0, /* cn* */mapstep*sizeof(int));

    uchar* map = (uchar*)(mag_buf[2] + mapstep*cn);
    memset(map, 1, mapstep);
    memset(map + mapstep*(src.rows + 1), 1, mapstep);

    int maxsize = std::max(1 << 10, src.cols * src.rows / 10);
    std::vector<uchar*> stack(maxsize);
    uchar **stack_top = &stack[0];
    uchar **stack_bottom = &stack[0];

    /* sector numbers
       (Top-Left Origin)

        1   2   3
         *  *  *
          * * *
        0*******0
          * * *
         *  *  *
        3   2   1
    */

#define CANNY_PUSH(d)    *(d) = uchar(2), *stack_top++ = (d)
#define CANNY_POP(d)     (d) = *--stack_top

    // calculate magnitude and angle of gradient, perform non-maxima supression.
    // fill the map with one of the following values:
    //   0 - the pixel might belong to an edge
    //   1 - the pixel can not belong to an edge
    //   2 - the pixel does belong to an edge
    for (int i = 0; i <= src.rows; i++)
    {
        int* _norm = mag_buf[(i > 0) + 1] + 1;
        if (i < src.rows)
        {
            short* _dx = dx.ptr<short>(i);
            short* _dy = dy.ptr<short>(i);

            if (!L2gradient)
            {
                for (int j = 0; j < src.cols*cn; j++)
                    _norm[j] = std::abs(int(_dx[j])) + std::abs(int(_dy[j]));
            }
            else
            {
                for (int j = 0; j < src.cols*cn; j++)
                    _norm[j] = int(_dx[j])*_dx[j] + int(_dy[j])*_dy[j];
            }

            if (cn > 1)
            {
                for(int j = 0, jn = 0; j < src.cols; ++j, jn += cn)
                {
                    int maxIdx = jn;
                    for(int k = 1; k < cn; ++k)
                        if(_norm[jn + k] > _norm[maxIdx]) maxIdx = jn + k;
                    _norm[j] = _norm[maxIdx];
                    _dx[j] = _dx[maxIdx];
                    _dy[j] = _dy[maxIdx];
                }
            }
            _norm[-1] = _norm[src.cols] = 0;
        }
        else
            memset(_norm-1, 0, /* cn* */mapstep*sizeof(int));

        // at the very beginning we do not have a complete ring
        // buffer of 3 magnitude rows for non-maxima suppression
        if (i == 0)
            continue;

        uchar* _map = map + mapstep*i + 1;
        _map[-1] = _map[src.cols] = 1;

        int* _mag = mag_buf[1] + 1; // take the central row
        ptrdiff_t magstep1 = mag_buf[2] - mag_buf[1];
        ptrdiff_t magstep2 = mag_buf[0] - mag_buf[1];

        const short* _x = dx.ptr<short>(i-1);
        const short* _y = dy.ptr<short>(i-1);

        if ((stack_top - stack_bottom) + src.cols > maxsize)
        {
            int sz = (int)(stack_top - stack_bottom);
            maxsize = maxsize * 3/2;
            stack.resize(maxsize);
            stack_bottom = &stack[0];
            stack_top = stack_bottom + sz;
        }

        int prev_flag = 0;
        for (int j = 0; j < src.cols; j++)
        {
#define CANNY_SHIFT 15
            const int TG22 = (int)(0.4142135623730950488016887242097*(1<<CANNY_SHIFT) + 0.5);

            int m = _mag[j];

            if (m > low)
            {
                int xs = _x[j];
                int ys = _y[j];
                int x = std::abs(xs);
                int y = std::abs(ys) << CANNY_SHIFT;

                int tg22x = x * TG22;

                if (y < tg22x)
                {
                    if (m > _mag[j-1] && m >= _mag[j+1]) goto __ocv_canny_push;
                }
                else
                {
                    int tg67x = tg22x + (x << (CANNY_SHIFT+1));
                    if (y > tg67x)
                    {
                        if (m > _mag[j+magstep2] && m >= _mag[j+magstep1]) goto __ocv_canny_push;
                    }
                    else
                    {
                        int s = (xs ^ ys) < 0 ? -1 : 1;
                        if (m > _mag[j+magstep2-s] && m > _mag[j+magstep1+s]) goto __ocv_canny_push;
                    }
                }
            }
            prev_flag = 0;
            _map[j] = uchar(1);
            continue;
__ocv_canny_push:
            if (!prev_flag && m > high && _map[j-mapstep] != 2)
            {
                CANNY_PUSH(_map + j);
                prev_flag = 1;
            }
            else
                _map[j] = 0;
        }

        // scroll the ring buffer
        _mag = mag_buf[0];
        mag_buf[0] = mag_buf[1];
        mag_buf[1] = mag_buf[2];
        mag_buf[2] = _mag;
    }

    // now track the edges (hysteresis thresholding)
    while (stack_top > stack_bottom)
    {
        uchar* m;
        if ((stack_top - stack_bottom) + 8 > maxsize)
        {
            int sz = (int)(stack_top - stack_bottom);
            maxsize = maxsize * 3/2;
            stack.resize(maxsize);
            stack_bottom = &stack[0];
            stack_top = stack_bottom + sz;
        }

        CANNY_POP(m);

        if (!m[-1])         CANNY_PUSH(m - 1);
        if (!m[1])          CANNY_PUSH(m + 1);
        if (!m[-mapstep-1]) CANNY_PUSH(m - mapstep - 1);
        if (!m[-mapstep])   CANNY_PUSH(m - mapstep);
        if (!m[-mapstep+1]) CANNY_PUSH(m - mapstep + 1);
        if (!m[mapstep-1])  CANNY_PUSH(m + mapstep - 1);
        if (!m[mapstep])    CANNY_PUSH(m + mapstep);
        if (!m[mapstep+1])  CANNY_PUSH(m + mapstep + 1);
    }

    // the final pass, form the final image
    const uchar* pmap = map + mapstep + 1;
    uchar* pdst = dst.ptr();
    for (int i = 0; i < src.rows; i++, pmap += mapstep, pdst += dst.step)
    {
        for (int j = 0; j < src.cols; j++)
            pdst[j] = (uchar)-(pmap[j] >> 1);
    }
}


static void color_edges_callback(int value, void* arg)
{
    bool& use_color_edges = *(bool*)(arg);
    use_color_edges = value == 1 ? true : false;
}

struct DepthMeshifier::Impl
{
    cv::Mat camera_matrix, dist_coeffs;
};

DepthMeshifier::DepthMeshifier(const std::string& calibration) :
    width(640), height(480),
    focal_x(540), focal_y(540),
    center_x(width / 2.0), center_y(height / 2.0),
    near_plane(500), far_plane(4000),
    min_threshold(40), max_threshold(80),
    approx_polygon(2000), min_area(100),
    dilate_erode_steps(2),
    min_contour_area(100), depth_threshold(20),
    m_is_draw_2d_enabled(false),
    use_color_edges(true),
    filter(new DepthFilter(width, height)),
    canny_worker(new AsyncWorker),
    cloud_worker(new AsyncWorker),
    p(new Impl)
{
    cv::FileStorage fs(calibration, cv::FileStorage::READ);
    if (fs.isOpened()) {
        fs["image_width"] >> width;
        fs["image_height"] >> height;
        fs["irCameraMatrix"] >> p->camera_matrix;
        focal_x = p->camera_matrix.at<double>(0, 0);
        focal_y = p->camera_matrix.at<double>(1, 1);
        center_x = p->camera_matrix.at<double>(0, 2);
        center_y = p->camera_matrix.at<double>(1, 2);
        std::cout << p->camera_matrix << std::endl;
        std::cout << "IR Cam: " << width << ' ' << height << ' ' << focal_x << ' ' << focal_y << ' ' << center_x << ' ' << center_y << std::endl;
        fs["distortion_coefficients"] >> p->dist_coeffs;
    } else
        std::cerr << "WARNING: Unable to open camera calibration file " << calibration << ". Using default (inexact) camera matrix." << std::endl;
}

DepthMeshifier::~DepthMeshifier()
{ }

void DepthMeshifier::operator()(char* buffer_rgb, char* buffer_depth, std::vector<unsigned>& tri, std::vector<float>& ver)
{
    tri.clear();
    ver.clear();
    //cv::Mat internal_edges(height, width, CV_8UC1);
    //(*filter)(buffer_rgb, (unsigned short*)buffer_depth, internal_edges);
    //cv::imshow("Internal Edges", internal_edges);
    cv::Mat depth(height, width, CV_16UC1, buffer_depth), depth8(height, width, CV_8UC1);
    if (is_background_subtraction_enabled == true) {
        if (background.empty() == true) {
            background.resize(width * height);
            std::copy(depth.begin<unsigned short>(), depth.end<unsigned short>(), background.begin());
        }
    }
    else
        background.clear();
    unsigned short depth_min = std::numeric_limits<unsigned short>::max(), depth_max = 0;
    for (int i = 0; i < width * height; ++i) {
        unsigned short& d = depth.at<unsigned short>(i);
        d = (d < near_plane || d > far_plane || (background.empty() == false && std::abs(d - background[i]) < 100)) ? 0 : d;
        if (d == 0)
            continue;
        depth_min = std::min(d, depth_min);
        depth_max = std::max(d, depth_max);
    }
    pcl::RangeImagePlanar::Ptr cloud(new pcl::RangeImagePlanar);
    cloud_worker->begin([cloud, &depth, this] {
        cloud->setDepthImage(depth.ptr<unsigned short>(), width, height, center_x, center_y, focal_x, focal_y);
    });
    const double alpha = 250.0 / (depth_max - depth_min);
    const double beta = -depth_min * alpha + 5;
    for (int i = 0; i < width * height; ++i) {
        const unsigned short& s = depth.at<unsigned short>(i);
        unsigned char& c = depth8.at<unsigned char>(i);
        c = s == 0 ? 0 : (s * alpha + beta);
    }
    cv::Mat img_color(height, width, CV_8UC3, buffer_rgb), img_gray;
    const bool canny_color = use_color_edges;
    if (canny_color == true)
        canny_worker->begin([&img_color, &img_gray, this] {
            cv::cvtColor(img_color, img_gray, CV_RGB2GRAY);
            cv::blur(img_gray, img_gray, cv::Size(3, 3));
            cv::Canny(img_gray, img_gray, min_threshold, max_threshold, 3, true);
        });
    cv::Mat mask(height, width, CV_8UC1);
    cv::threshold(depth8, mask, 0, 255, CV_THRESH_BINARY);
    cv::blur(depth8, depth8, cv::Size(3, 3));
    cv::Canny(depth8, depth8, min_threshold, max_threshold, 3, true);
    //my_canny(depth8, depth8, min_threshold, max_threshold, 3, true);
    //cv::bitwise_or(depth8, internal_edges, depth8, mask);
    if (canny_color == true) {
        canny_worker->end();
        cv::bitwise_or(depth8, img_gray, depth8, mask);
    }
    cv::dilate(depth8, depth8, cv::Mat(), cv::Point(-1, -1), dilate_erode_steps);
    cv::erode(depth8, depth8, cv::Mat(), cv::Point(-1, -1), dilate_erode_steps);
    if (false)
        cv::imshow("Edge detection", depth8);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(depth8, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    cloud_worker->end();
    Triangulator triangulate(cloud, min_area, depth_threshold);
    for (unsigned i = 0; i < contours.size(); ++i) {
        std::vector<cv::Point>& c = contours[i];
        cv::approxPolyDP(c, c, approx_polygon / 1000.0, true);
        if (cv::contourArea(c) < min_contour_area)
            continue;
        triangulate.add_contour(c);
    }
    if (false) {
        cv::Mat image_contours = cv::Mat::zeros(depth.size(), CV_8UC1);
        cv::drawContours(image_contours, contours, -1, cv::Scalar(255));
        cv::imshow("Contours", image_contours);
    }
    triangulate();
    if (m_is_draw_2d_enabled)
        triangulate.draw(img_color);
    const std::vector<cv::Vec6f> triangles = triangulate.get_triangles();
    if (triangles.empty())
        return;
    SurfaceReconstruction recon;
    recon(triangles, depth, cloud);
    const MeshBuilder& m = recon.mesh();
    tri = m.get_triangles();
    ver = m.get_vertices();
    if (false) {
        cv::Mat mask_rgb;
        cv::cvtColor(mask, mask_rgb, CV_GRAY2BGR);
        triangulate.draw(mask_rgb);
        cv::cvtColor(mask_rgb, mask_rgb, CV_RGB2BGR);
        cv::imshow("Mask", mask_rgb);
    }
}
