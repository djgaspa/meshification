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

#include <iostream>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <chrono>
#include <future>
#include <RakNet/RakPeerInterface.h>
#include <RakNet/RakNetTypes.h>
#include <RakNet/MessageIdentifiers.h>
#include <RakNet/BitStream.h>
#include <RakNet/RakString.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include "Consumer.hpp"
#include "VideoEncoder.hpp"
#include "../common/AsyncWorker.hpp"
#include "../3dzip/3dzip/Writer.hh"

Consumer::Consumer(const std::string& address, const std::string& name, const std::string& calib) :
    ip_address(address),
    name(name),
    peer(RakNet::RakPeerInterface::GetInstance()),
    address(new RakNet::SystemAddress),
    async_video(new AsyncWorker)
{
    auto socket = RakNet::SocketDescriptor();
    peer->Startup(1, &socket, 1);
    connect();
    Eigen::Map<Eigen::Matrix4d>(modelview).setIdentity();
    std::ifstream calibration("calibration.txt");
    if (calibration.is_open())
        for (int i = 0; i < 16; ++i)
            calibration >> modelview[i];
    cv::FileStorage fs(calib, cv::FileStorage::READ);
    if (fs.isOpened() == false)
        throw std::logic_error("Unable to open calibration file "  + calib);
    cv::Mat camera_matrix, tvec, rot, K;
    fs["image_width"] >> width;
    fs["image_height"] >> height;
    fs["camera_matrix"] >> camera_matrix;
    camera_focal_x = camera_matrix.at<double>(0, 0);
    camera_focal_y = camera_matrix.at<double>(1, 1);
    camera_centre_x = camera_matrix.at<double>(0, 2);
    camera_centre_y = camera_matrix.at<double>(1, 2);
    fs["T"] >> tvec;
    fs["R"] >> rot;
    fs["distortion_coefficients"] >> K;
    for (int i = 0; i < 3; ++i)
        t[i] = -tvec.at<double>(i);
    for (int i = 0; i < 9; ++i)
        r[i] = rot.at<double>(i);
    for (int i = 0; i < 5; ++i)
        k[i] = K.at<double>(i);
}

Consumer::~Consumer()
{
    RakNet::RakPeerInterface::DestroyInstance(peer);
}

void Consumer::connect()
{
    peer->Connect(ip_address.c_str(), 12345, 0, 0);
}

void Consumer::operator()(const std::vector<float>& ver, const std::vector<unsigned>& tri, const std::vector<char>& rgb)
{
    auto packet_deleter = [this](RakNet::Packet* p) {
        peer->DeallocatePacket(p);
    };

    for (;;) {
        std::unique_ptr<RakNet::Packet, decltype(packet_deleter)> p(peer->Receive(), packet_deleter);
        if (p.get() == 0)
            break;
        const auto id = p->data[0];
        switch (id) {
        case ID_CONNECTION_REQUEST_ACCEPTED:
            if (is_connected == false)
                std::cerr << "Connection established" << std::endl;
            *address = p->systemAddress;
            encode.reset(new VideoEncoder(width, height));
            is_connected = true;
            break;
        case ID_CONNECTION_ATTEMPT_FAILED:
            std::cerr << "Unable to connect to the server" << std::endl;
            is_connected = false;
            connect();
            break;
        case ID_CONNECTION_LOST:
        case ID_DISCONNECTION_NOTIFICATION:
            connect();
            std::cerr << "Connection lost" << std::endl;
            async_video->end();
            encode.reset();
            is_connected = false;
            break;
        case ID_NO_FREE_INCOMING_CONNECTIONS:
            connect();
            std::cerr << "The server is full" << std::endl;
            is_connected  = false;
            break;
        default:
            std::cout << "Packet received " << int(id) << std::endl;
        }
    }
    if (is_connected == false)
        return;
    using clock = std::chrono::high_resolution_clock;
    const auto t0 = clock::now();
    std::string video_string;
    bool is_keyframe = false;
    async_video->begin([this, rgb, &video_string, &is_keyframe] {
        const auto t0 = clock::now();
        (*encode)(rgb.data(), [&video_string, &is_keyframe] (char* data, const int size, const bool is_kf) {
            video_string.assign(data, size);
            is_keyframe = is_kf;
        });
        const auto t1 = clock::now();
        //std::cout << "Video encoding: " << (t1 - t0).count() << std::endl;
    });
    const auto t1 = clock::now();
    const bool compression = true;
    std::stringstream model_stream(std::ios::in | std::ios::out | std::ios::binary);
    if (compression) {
        model_stream.put(1);
        VBE::Writer compress(&tri[0], tri.size() / 3, ver.size() / 3, true);
        compress.addAttrib(&ver[0], ver.size() / 3, 3, "V", 12);
        compress(model_stream);
    } else {
        model_stream.put(0);
        const int n_vertices = ver.size() / 3, n_triangles = tri.size() / 3;
        model_stream.write((const char*)&n_vertices, sizeof(n_vertices));
        model_stream.write((const char*)&ver[0], ver.size() * sizeof(float));
        model_stream.write((const char*)&n_triangles, sizeof(n_triangles));
        model_stream.write((const char*)&tri[0], tri.size() * sizeof(unsigned));
    }
    model_stream << std::flush;
    const std::string& model_string = model_stream.str();
    const auto t2 = clock::now();
    async_video->end();
    const auto t3 = clock::now();
    RakNet::BitStream network_stream;
    network_stream.Write(static_cast<RakNet::MessageID>(ID_USER_PACKET_ENUM));
    network_stream.Write(RakNet::RakString(name.c_str()));
    network_stream.Write(camera_centre_x);
    network_stream.Write(camera_centre_y);
    network_stream.Write(camera_focal_x);
    network_stream.Write(camera_focal_y);
    network_stream.Write(modelview);
    network_stream.Write(t);
    network_stream.Write(r);
    network_stream.Write(k);
    network_stream.Write(static_cast<int>(model_string.size()));
    network_stream.Write(model_string.data(), model_string.size());
    network_stream.Write(is_keyframe);
    network_stream.Write(static_cast<int>(video_string.size()));
    network_stream.Write(video_string.data(), video_string.size());
    peer->Send(&network_stream, LOW_PRIORITY, UNRELIABLE, 0, *address, false);
    const auto t4 = clock::now();
    //std::cout << "Model Size: " << model_size * 30 * 8 / 1024.0 <<  "kbps Video Size: " << video_size * 30 * 8 / 1024.0 << "kbps" << std::endl;
    //std::cout << "Mesh compression: " << (t2 - t1).count() << "\nNetwork: " << (t4 - t3).count() << "\nTotal: " << (t4 - t0).count() << std::endl;
}

void Consumer::save_view()
{
    std::ofstream calibration("calibration.txt");
    if (calibration.is_open() == false) {
        std::cerr << "WARNING: Unable to save the calibration into the file" << std::endl;
        return;
    }
    for (int i = 0; i < 16; ++i)
        calibration << modelview[i] << ' ';
}

void Consumer::set_model_matrix(const std::vector<float>& m)
{
    std::copy(m.begin(), m.end(), modelview);
}

std::vector<float> Consumer::get_model_matrix() const
{
    std::vector<float>m(16);
    std::copy(modelview, modelview + 16, m.begin());
    return m;
}
