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

class VideoEncoder;
class AsyncWorker;
namespace RakNet {
class RakPeerInterface;
class SystemAddress;
}

namespace aruco {
class CameraParameters;
class MarkerDetector;
}

class Consumer
{
    bool use_marker_tracking = false;
    std::string ip_address, name;
    RakNet::RakPeerInterface* peer;
    std::unique_ptr<RakNet::SystemAddress> address;
    bool is_connected = false;

    std::unique_ptr<aruco::CameraParameters> cam_params;
    std::unique_ptr<aruco::MarkerDetector> marker_detector;
    double modelview[16];

    std::unique_ptr<AsyncWorker> async_video, async_marker;
    std::unique_ptr<VideoEncoder> encode;

    void connect();

public:
    Consumer(const int w, const int h, const std::string& address, const std::string& name = "default");
    ~Consumer();
    void operator()(const std::vector<float>& ver, const std::vector<unsigned>& tri, const std::vector<char>& rgb);
    void enable_marker_tracking(const bool b) {
        use_marker_tracking = b;
    }
    void save_view();
};
