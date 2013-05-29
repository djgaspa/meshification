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
#include <QTimerEvent>
#include "QtAcquisition.hpp"
#include "DepthMeshifier.hpp"
#include "Consumer.hpp"
#include "SourceKinect.hpp"
#include "../common/AsyncWorker.hpp"

QtAcquisition::QtAcquisition(const int cam_id, const std::string &address, const std::string &calib, QObject *parent) :
    QObject(parent),
    camera(new SourceKinect(cam_id)),
    meshify(new DepthMeshifier(calib)),
    consume(new Consumer(address, std::to_string(cam_id))),
    consumer_worker(new AsyncWorker),
    width(camera->width()), height(camera->height())
{
    qRegisterMetaType<RgbBuffer>("RgbBuffer");
}

QtAcquisition::~QtAcquisition()
{}

void QtAcquisition::timerEvent(QTimerEvent* e)
{
    if (e->timerId() != timer_id)
        return;
    process_frame();
}

void QtAcquisition::process_frame()
{
    using clock = std::chrono::high_resolution_clock;
    std::vector<char> buffer_depth(2 * width * height), buffer_rgb(3 * width * height);
    camera->grab(buffer_rgb.data(), buffer_depth.data());
    const auto t0 = clock::now();
    std::vector<unsigned> tri;
    std::vector<float> ver;
    (*meshify)(buffer_rgb.data(), buffer_depth.data(), tri, ver);
    if (tri.empty() == false) {
        consumer_worker->begin([=] {
            (*consume)(ver, tri, buffer_rgb);
        });
    }
    emit draw(buffer_rgb, width, height);
    const auto t1 = clock::now();
    const auto t = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    //decompressed_stream.read((char*)&decompressed_buffer[0], width * height * 2);
    //::compare(buffer_depth, &decompressed_buffer[0], cv::Size(width, height));
    emit message(QString("Frame: %1 #T: %2 %3 ms").arg(frame_id++).arg(tri.size() / 3).arg(t));
}

void QtAcquisition::setup()
{
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
    return consume->is_marker_tracking_enabled();
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
    consume->enable_marker_tracking(e);
}

void QtAcquisition::saveView()
{
    consume->save_view();
}

void QtAcquisition::setAddress(QString address)
{
    const std::string name = consume->get_name();
    consume.reset(new Consumer(address.toStdString(), name));
}
