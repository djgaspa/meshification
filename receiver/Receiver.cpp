#include <iostream>
#include <sstream>
#include <list>
#include <pcl/range_image/range_image_planar.h>
#include <RakNet/RakPeerInterface.h>
#include <RakNet/MessageIdentifiers.h>
#include <RakNet/BitStream.h>
#include <RakNet/RakSleep.h>
#include <Eigen/Core>
#include "Receiver.hpp"
#include "VideoDecoder.hpp"
#include "Model.hpp"
#include "Data3d.hpp"
#include "../3dzip/3dzip/Reader.hh"
#include "../common/AsyncWorker.hpp"

struct Peer
{
    AsyncWorker worker, video_worker;
    VideoDecoder decoder;
};

Receiver::Receiver()
{}

Receiver::~Receiver()
{
    stop();
}

void Receiver::init()
{
    Model::init();
}

void Receiver::start()
{
    is_running = true;
    t = boost::thread(std::bind(&Receiver::run, this));
}

void Receiver::run()
{
    std::unique_ptr<RakNet::RakPeerInterface, decltype(&RakNet::RakPeerInterface::DestroyInstance)> peer(RakNet::RakPeerInterface::GetInstance(), &RakNet::RakPeerInterface::DestroyInstance);
    RakNet::SocketDescriptor socket(12345, 0);
    peer->Startup(3, &socket, 1);
    peer->SetMaximumIncomingConnections(3);
    peer->SetTimeoutTime(1000, RakNet::UNASSIGNED_SYSTEM_ADDRESS);
    is_running = true;
    auto packet_deleter = [&peer](RakNet::Packet* p) {
        peer->DeallocatePacket(p);
    };
    std::unordered_map<std::uint64_t, std::shared_ptr<Peer>> peers;
    while (is_running) {
        const auto ptr = peer->Receive();
        if (ptr == 0)
            continue;
        std::shared_ptr<RakNet::Packet> p(ptr, packet_deleter);
        switch (p->data[0]) {
        case ID_NEW_INCOMING_CONNECTION: {
            peers.insert(std::make_pair(p->guid.g, std::make_shared<Peer>()));
            Lock l(m);
            new_models.insert(p->guid.g);
            break;
        }
        case ID_CONNECTION_LOST:
        case ID_DISCONNECTION_NOTIFICATION: {
            peers.erase(p->guid.g);
            Lock l(m);
            delete_models.insert(p->guid.g);
            break;
        }
        case ID_USER_PACKET_ENUM: {
            auto peer = peers[p->guid.g];
            peer->worker.begin([p, peer, this] {
                const int width = 640, height = 480;
                auto data = std::make_shared<Data3d>(width, height);
                RakNet::BitStream bs(p->data, p->length, false);
                bs.IgnoreBytes(sizeof(RakNet::MessageID));
                RakNet::RakString name;
                bs.Read(name);
                data->name = name.C_String();
                bs.Read(data->center_x);
                bs.Read(data->center_y);
                bs.Read(data->focal_x);
                bs.Read(data->focal_y);
                bs.Read(data->modelview);
                int size;
                bs.Read(size);
                std::vector<char> buffer(size);
                bs.Read(buffer.data(), size);
                std::istringstream in(std::string(buffer.begin(), buffer.end()), std::ios::in | std::ios::binary);
                bs.Read(size);
                buffer.resize(size);
                bs.Read(buffer.data(), size);
                std::istringstream in_video(std::string(buffer.begin(), buffer.end()), std::ios::in | std::ios::binary);
                peer->video_worker.begin([&] {
                    peer->decoder(in_video, data->y_img.data(), data->u_img.data(), data->v_img.data());
                });
                const bool compression = (in.get() != 0);
                if (compression) {
                    VBE::Reader read;
                    read(in);
                    const int n_tri = read.getNumTri();
                    const int n_ver = read.getNumVer();
                    data->tri.resize(3 * n_tri);
                    data->ver.resize(3 * n_ver);
                    read.getTriangles(&data->tri[0]);
                    read.getAttrib("V", &data->ver[0]);
                } else {
                    int n_vertices, n_triangles;
                    in.read((char*)&n_vertices, sizeof(n_vertices));
                    data->ver.resize(3 * n_vertices);
                    in.read((char*)&data->ver[0], data->ver.size() * sizeof(float));
                    in.read((char*)&n_triangles, sizeof(n_triangles));
                    data->tri.resize(3 * n_triangles);
                    in.read((char*)&data->tri[0], data->tri.size() * sizeof(unsigned));
                }
                peer->video_worker.end();
                Lock l(m);
                updates[p->guid.g] = data;
            });
            break;
        }
        }
    }
}

void Receiver::stop()
{
    is_running = false;
    t.join();
}

void Receiver::draw()
{
    std::list<std::pair<std::uint64_t, std::shared_ptr<Data3d>>> data;
    Lock l(m);
    for (auto& g : new_models)
        models[g].reset(new Model);
    new_models.clear();
    for (auto& g : delete_models)
        models.erase(g);
    delete_models.clear();
    for (auto& u : updates)
        data.push_back(u);
    updates.clear();
    l.unlock();
    for (const auto& d : data) {
        auto it = models.find(d.first);
        if (it == models.end()) {
            std::cerr << "WARNING: Updating an unknown model" << std::endl;
            continue;
        }
        it->second->load(*d.second);
    }

    for (const auto& m : models)
        m.second->draw();
}

void Receiver::translate(const std::string& name, const double x, const double y, const double z)
{
    auto it = std::find_if(models.begin(), models.end(), [&name](const decltype(models)::value_type& m) {
        return m.second->get_name() == name;
    });
    if (it == models.end()) {
        std::cerr << "WARNING: Model " << name << " not found" << std::endl;
        return;
    }
    it->second->translate(x, y, z);
}

void Receiver::rotate(const std::string& name, const double rad, const double x, const double y, const double z)
{
    auto it = std::find_if(models.begin(), models.end(), [&name](const decltype(models)::value_type& m) {
        return m.second->get_name() == name;
    });
    if (it == models.end()) {
        std::cerr << "WARNING: Model " << name << " not found" << std::endl;
        return;
    }
    it->second->rotate(rad, x, y, z);
}

void Receiver::reset_position(const std::string &name)
{
    auto it = std::find_if(models.begin(), models.end(), [&name](const decltype(models)::value_type& m) {
        return m.second->get_name() == name;
    });
    if (it == models.end()) {
        std::cerr << "WARNING: Model " << name << " not found" << std::endl;
        return;
    }
    it->second->reset_position();
}

void Receiver::save_view() const
{
    for (const auto& m : models)
        m.second->save_view();
}
