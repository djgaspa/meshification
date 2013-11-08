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
#include <aruco/cameraparameters.h>
#include <aruco/markerdetector.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "Consumer.hpp"
#include "VideoEncoder.hpp"
#include "../common/AsyncWorker.hpp"
#include "../3dzip/3dzip/Writer.hh"

Consumer::Consumer(const string &address, const string &name) :
    ip_address(address),
    name(name),
    peer(RakNet::RakPeerInterface::GetInstance()),
    address(new RakNet::SystemAddress),
    cam_params(new aruco::CameraParameters),
    marker_detector(new aruco::MarkerDetector),
    async_video(new AsyncWorker),
    async_marker(new AsyncWorker)
{
    auto socket = RakNet::SocketDescriptor();
    peer->Startup(1, &socket, 1);
    connect();
    cam_params->readFromXMLFile("cam.yml");
    Eigen::Map<Eigen::Matrix4d>(modelview).setIdentity();
    std::ifstream calibration("calibration.txt");
    if (calibration.is_open())
        for (int i = 0; i < 16; ++i)
            calibration >> modelview[i];
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
            encode.reset(new VideoEncoder(cam_params->CamSize.width, cam_params->CamSize.height));
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
    async_video->begin([this, rgb, &video_string] {
        const auto t0 = clock::now();
        std::ostringstream video_stream(std::ios::in | std::ios::out | std::ios::binary);
        (*encode)(video_stream, rgb.data());
        video_stream << std::flush;
        video_string = video_stream.str();
        const auto t1 = clock::now();
        //std::cout << "Video encoding: " << (t1 - t0).count() << std::endl;
    });
    async_marker->begin([this, rgb] {
        const auto t0 = clock::now();
        std::vector<aruco::Marker> markers;
        cv::Mat frame(480, 640, CV_8UC3, const_cast<char*>(rgb.data()));
        //marker_detector->detect(frame, markers, *cam_params, 0.197, false);
        if (use_marker_tracking)
            marker_detector->detect(frame, markers, *cam_params, marker_size, false);
        for (auto& m : markers) {
            m.draw(frame, cv::Scalar(255, 0, 0));
            if (m.id != 45)
                continue;
            cv::Mat rot_src = m.Rvec.clone(), rot;
            rot_src.at<float>(1, 0) *= -1.0f;
            rot_src.at<float>(2, 0) *= -1.0f;
            cv::Rodrigues(rot_src, rot);
            Eigen::Matrix3d r = Eigen::Map<Eigen::Matrix3f>((float*)rot.ptr()).cast<double>();
            Eigen::Vector3d t(-m.Tvec.at<float>(0, 0), m.Tvec.at<float>(1, 0), m.Tvec.at<float>(2, 0));
            Eigen::Affine3d a = Eigen::Affine3d::Identity();
            a.rotate(r).translate(t);
            Eigen::Map<Eigen::Matrix4d> mv_matrix(modelview);
            mv_matrix = a.matrix();
        }
        const auto t1 = clock::now();
        //std::cout << "Marker detection: " << (t1 - t0).count() << std::endl;
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
    async_marker->end();
    async_video->end();
    const auto t3 = clock::now();
    RakNet::BitStream network_stream;
    network_stream.Write(static_cast<RakNet::MessageID>(ID_USER_PACKET_ENUM));
    network_stream.Write(RakNet::RakString(name.c_str()));
    network_stream.Write(cam_params->CameraMatrix.at<float>(0, 2));
    network_stream.Write(cam_params->CameraMatrix.at<float>(1, 2));
    network_stream.Write(cam_params->CameraMatrix.at<float>(0, 0));
    network_stream.Write(cam_params->CameraMatrix.at<float>(1, 1));
    network_stream.Write(modelview);
    network_stream.Write(static_cast<int>(model_string.size()));
    network_stream.Write(model_string.data(), model_string.size());
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
