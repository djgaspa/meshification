/*
    Copyright (C) 2011-2013 Paolo Simone Gasparello <p.gasparello@sssup.it>

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

#include <chrono>
#include <QSharedPointer>
#include <QTimerEvent>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "QtAcquisition.hpp"
#include "DepthMeshifier.hpp"
#include "Consumer.hpp"
#include "SourceKinect.hpp"
#include "SourceKinectOpenNI.hpp"
#include "../common/AsyncWorker.hpp"

Q_DECLARE_METATYPE(std::vector<float>)

struct QtAcquisition::CameraParams
{
    cv::Mat ir_camera_matrix, ir_distortion_coefficients;
};

QtAcquisition::QtAcquisition(const int cam_id, const std::string &name, const std::string &address, QObject *parent) :
    QObject(parent),
    camera(new SourceKinectOpenNI(cam_id)),
    camera_params(new CameraParams),
    calib_file(camera->getSerial() + ".yml"),
    meshify(new DepthMeshifier(calib_file)),
    consume(new Consumer(address, name, calib_file)),
    consumer_worker(new AsyncWorker),
    width(camera->width()), height(camera->height()),
    center_x(width / 2), center_y(height / 2),
    focal_x(540), focal_y(540),
    k(5), t(3), r(9)
{
    qRegisterMetaType<RgbBuffer>("RgbBuffer");
    qRegisterMetaType<std::vector<float>>();
    cv::FileStorage fs(calib_file, cv::FileStorage::READ);
    if (fs.isOpened() == false) {
        std::ostringstream error;
        error << "Unable to open camera calibration file " << calib_file << '.' << std::endl;
        throw std::logic_error(error.str());
    }
    fs["image_width"] >> width;
    fs["image_height"] >> height;
    cv::Mat camera_matrix;
    fs["camera_matrix"] >> camera_matrix;
    focal_x = camera_matrix.at<double>(0, 0);
    focal_y = camera_matrix.at<double>(1, 1);
    center_x = camera_matrix.at<double>(0, 2);
    center_y = camera_matrix.at<double>(1, 2);
    cv::Mat mat_k, mat_t, mat_r;
    fs["distortion_coefficients"] >> mat_k;
    fs["T"] >> mat_t;
    fs["R"] >> mat_r; // row major, no need to transpose it if used as column major (OpenGL)
    for (int i = 0; i < 3; ++i)
        t[i] = -mat_t.at<double>(i);
    for (int i = 0; i < 9;++i)
        r[i] = mat_r.at<double>(i);
    for (int i = 0; i < 5; ++i)
        k[i] = mat_k.at<double>(i);
    std::cout << "RGB Cam: " << width << ' ' << height << ' ' << focal_x << ' ' << focal_y << ' ' << center_x << ' ' << center_y << std::endl;
    fs["irCameraMatrix"] >> camera_params->ir_camera_matrix;
    fs["irDistCoeffs"] >> camera_params->ir_distortion_coefficients;
}

QtAcquisition::~QtAcquisition()
{}

void QtAcquisition::timerEvent(QTimerEvent* e)
{
    const auto id = e->timerId();
    if (id == timer_id)
        process_frame();
    else if (id == timer_registration)
        registration_frame();
}

void QtAcquisition::process_frame()
{
    using clock = std::chrono::high_resolution_clock;
    std::vector<char> buffer_depth(2 * width * height), buffer_rgb(3 * width * height);
    camera->grab();
    camera->getImage(buffer_rgb.data());
    camera->getDepth(buffer_depth.data());
    const auto t0 = clock::now();
    std::vector<unsigned> tri;
    std::vector<float> ver;
    (*meshify)(buffer_rgb.data(), buffer_depth.data(), tri, ver);
    if (tri.empty() == false) {
        consumer_worker->begin([=] {
            (*consume)(ver, tri, buffer_rgb);
        });
    }
    QtModelDescriptor desc {
        width, height, center_x, center_y, focal_x, focal_y,
                QVector<float>::fromStdVector(k), QVector<float>::fromStdVector(t), QVector<float>::fromStdVector(r),
                QVector<float>::fromStdVector(ver), QVector<unsigned>::fromStdVector(tri), QVector<char>::fromStdVector(buffer_rgb)
    };
    emit update(desc);
    emit draw(std::move(buffer_rgb), width, height);
    const auto t1 = clock::now();
    const auto t = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    //decompressed_stream.read((char*)&decompressed_buffer[0], width * height * 2);
    //::compare(buffer_depth, &decompressed_buffer[0], cv::Size(width, height));
    emit message(QString("Frame: %1 #T: %2 %3 ms").arg(frame_id++).arg(tri.size() / 3).arg(t));
}

static
std::vector<cv::Point3f> genPatternObjectPoints(const cv::Size& board_size, const double patternTileSize)
{
    std::vector<cv::Point3f> corners;
    for (int i = 0; i < board_size.height; ++i)
        for (int j = 0; j < board_size.width; ++j)
            corners.push_back(cv::Point3f(float((2 * j + i % 2) * patternTileSize), float(i * patternTileSize), 0));
    return corners;
}

void QtAcquisition::registration_frame()
{
    const cv::Size pattern_size(4, 11);
    std::vector<char> buffer(2 * width * height);
    camera->grab();
    camera->getIr(buffer.data());
    cv::Mat ir(height, width, CV_16UC1, buffer.data());
    cv::Mat ir8;
    ir.convertTo(ir8, CV_8UC1, 2);
    cv::Mat view;
    cv::cvtColor(ir8, view, CV_GRAY2BGR);
    std::vector<cv::Point2f> centers;
    const bool found = cv::findCirclesGrid(view, pattern_size, centers, cv::CALIB_CB_ASYMMETRIC_GRID);
    cv::drawChessboardCorners(view, pattern_size, centers, found);
    if (found) {
        const auto obj_points = ::genPatternObjectPoints(pattern_size, marker_size);
        cv::Mat rvec, tvec;
        cv::solvePnP(obj_points, centers, camera_params->ir_camera_matrix, camera_params->ir_distortion_coefficients, rvec, tvec);
        Eigen::Vector3d r = Eigen::Map<Eigen::Vector3d>(rvec.ptr<double>());
        Eigen::Vector3d t = Eigen::Map<Eigen::Vector3d>(tvec.ptr<double>());
        Eigen::Affine3d a = Eigen::Affine3d::Identity();
        a.rotate(Eigen::AngleAxisd(r.norm (), -r.normalized())).translate(-t);
        const Eigen::Matrix4f matrix = a.matrix().cast<float>();
        std::vector<float> m(matrix.data(), matrix.data() + 16);
        consume->set_model_matrix(m);
        emit modelMatrixChanged(m);
    }
    std::vector<char> rgb(width * height * 3);
    cv::cvtColor(view, view, CV_BGR2RGB);
    std::copy(view.data, view.data + width * height * 3, rgb.begin());
    emit draw(std::move(rgb), width, height);
}

void QtAcquisition::setup()
{
    camera->startImage();
    camera->startDepth();
    timer_id = startTimer(0);
}

bool QtAcquisition::isBorderColorEnabled() const
{
    return meshify->is_color_edges_enabled();
}

bool QtAcquisition::isDraw2dEnabled() const
{
    return meshify->is_2d_draw_enabled();
}

bool QtAcquisition::isMarkerEnabled() const
{
    return is_marker_enabled;
}

double QtAcquisition::getMarkerSize() const
{
    return marker_size;
}

int QtAcquisition::nearPlane() const
{
    return meshify->near_plane;
}

int QtAcquisition::farPlane() const
{
    return meshify->far_plane;
}

int QtAcquisition::minArea() const
{
    return meshify->min_area;
}

int QtAcquisition::minContourArea() const
{
    return meshify->min_contour_area;
}

int QtAcquisition::depthThreshold() const
{
    return meshify->depth_threshold;
}

int QtAcquisition::minThreshold() const
{
    return meshify->min_threshold;
}

int QtAcquisition::maxThreshold() const
{
    return meshify->max_threshold;
}

int QtAcquisition::approxDP() const
{
    return meshify->approx_polygon;
}

int QtAcquisition::dilateErode() const
{
    return meshify->dilate_erode_steps;
}

bool QtAcquisition::isBackgroundSubtractionEnabled() const
{
    return meshify->is_background_subtraction_enabled;
}

void QtAcquisition::setBorderColorEnabled(bool e)
{
    meshify->enable_color_edges(e);
}

void QtAcquisition::setDraw2dEnabled(bool e)
{
    meshify->enable_2d_draw(e);
}

void QtAcquisition::setMarkerEnabled(bool e)
{
    if (e) {
        killTimer(timer_id);
        camera->stopAll();
        camera->startIr();
        timer_registration = startTimer(0);
    }
    else {
        killTimer(timer_registration);
        camera->stopAll();
        camera->startImage();
        camera->startDepth();
        timer_id = startTimer(0);
    }
    is_marker_enabled = e;
}

void QtAcquisition::setMarkerSize(double s)
{
    marker_size = s;
}

void QtAcquisition::saveView()
{
    consume->save_view();
}

void QtAcquisition::setNearPlane(int d)
{
    meshify->near_plane = d;
}

void QtAcquisition::setFarPlane(int d)
{
    meshify->far_plane = d;
}

void QtAcquisition::setMinArea(int a)
{
    meshify->min_area = a;
}

void QtAcquisition::setMinContourArea(int a)
{
    meshify->min_contour_area = a;
}

void QtAcquisition::setDepthThreshold(int t)
{
    meshify->depth_threshold = t;
}

void QtAcquisition::setMinThreshold(int t)
{
    meshify->min_threshold = t;
}

void QtAcquisition::setMaxThreshold(int t)
{
    meshify->max_threshold = t;
}

void QtAcquisition::setApproxDP(int a)
{
    meshify->approx_polygon = a;
}

void QtAcquisition::setDilateErode(int s)
{
    meshify->dilate_erode_steps = s;
}

void QtAcquisition::setBackgroundSubtractionEnabled(bool e)
{
    meshify->is_background_subtraction_enabled = e;
}

void QtAcquisition::setAddress(QString name, QString address)
{
    consume.reset(new Consumer(address.toStdString(), name.toStdString(), calib_file));
}
