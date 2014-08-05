#include <iostream>
#include <fstream>
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

static
void write_packet(std::ostream& out, RakNet::Packet& p)
{
    out << p.systemAddress.ToString() << ' ' << p.guid.ToString() << ' ' << p.length << ' ' << p.bitSize << ' ' << p.deleteData << ' ' << p.wasGeneratedLocally << '\n';
    out.write((const char*)p.data, p.length);
    out << '\n';
}

static
void read_packet(std::istream& in, RakNet::Packet& p)
{
    std::string system_address, guid;
    in >> system_address >> guid >> p.length >> p.bitSize >> p.deleteData >> p.wasGeneratedLocally;
    in.ignore();
    p.data = new unsigned char[p.length];
    in.read((char*)p.data, p.length);
    p.systemAddress.FromString(system_address.c_str());
    p.guid.FromString(guid.c_str());
}

static
void destroy_packet(RakNet::Packet* p)
{
    delete[] p->data;
    delete p;
}

void Receiver::run(const std::string& filename)
{
    std::unique_ptr<RakNet::RakPeerInterface, decltype(&RakNet::RakPeerInterface::DestroyInstance)> peer(RakNet::RakPeerInterface::GetInstance(), &RakNet::RakPeerInterface::DestroyInstance);
    RakNet::SocketDescriptor socket(12345, 0);
    peer->Startup(5, &socket, 1);
    peer->SetMaximumIncomingConnections(5);
    peer->SetTimeoutTime(1000, RakNet::UNASSIGNED_SYSTEM_ADDRESS);
    is_running = true;
    auto packet_deleter = [&peer](RakNet::Packet* p) {
        peer->DeallocatePacket(p);
    };
    std::ofstream record;
    if (filename.empty() == false)
        record.open(filename, std::ios::binary);
    while (is_running) {
        const auto ptr = peer->Receive();
        if (ptr == 0)
            continue;
        std::shared_ptr<RakNet::Packet> p(ptr, packet_deleter);
        if (record.is_open())
            write_packet(record, *p);
        process_packet(p);
    }
}

void Receiver::play(const std::string& filename)
{
    std::ifstream in(filename.c_str(), std::ios::binary);
    if (in.is_open() == false) {
        std::cerr << "Unable to open " << filename << std::endl;
        return;
    }
    while (is_running) {
        std::shared_ptr<RakNet::Packet> p(new RakNet::Packet, &::destroy_packet);
        ::read_packet(in, *p);
        if (!in) {
            in.clear();
            in.seekg(0);
            continue;
        }
        process_packet(p);
        const int delay = 30 / std::max(1, (int)peers.size());
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    }
}

void Receiver::connect(const std::string& address, const unsigned short port)
{
    std::unique_ptr<RakNet::RakPeerInterface, decltype(&RakNet::RakPeerInterface::DestroyInstance)> peer(RakNet::RakPeerInterface::GetInstance(), &RakNet::RakPeerInterface::DestroyInstance);
    {
        RakNet::SocketDescriptor sd;
        peer->Startup(1, &sd, 1);
        peer->SetTimeoutTime(1000, RakNet::UNASSIGNED_SYSTEM_ADDRESS);
    }
    RakNet::SystemAddress system_address;
    const auto connect = [this, &peer, &system_address, &address, &port] {
        peer->CloseConnection(system_address, false);
        peer->Connect(address.c_str(), port, 0, 0);
    };
    connect();
    auto packet_deleter = [&peer](RakNet::Packet* p) {
        peer->DeallocatePacket(p);
    };

    while (is_running) {
        const auto ptr = peer->Receive();
        if (ptr == 0)
            continue;
        std::shared_ptr<RakNet::Packet> p(ptr, packet_deleter);
        switch (p->data[0]) {
        case ID_CONNECTION_REQUEST_ACCEPTED: {
            system_address = p->systemAddress;
            peers.insert(std::make_pair(p->guid.g, std::make_shared<Peer>()));
            Lock l(m);
            new_models.insert(p->guid.g);
            break;
        }
        case ID_CONNECTION_ATTEMPT_FAILED:
        case ID_NO_FREE_INCOMING_CONNECTIONS:
            std::cerr << "Unable to connect to the source" << std::endl;
            connect();
            break;
        case ID_CONNECTION_LOST:
        case ID_DISCONNECTION_NOTIFICATION:
            if (p->systemAddress == system_address)
                connect();
            break;
        }
        process_packet(p);
    }
}

void Receiver::process_packet(std::shared_ptr<RakNet::Packet> p)
{
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
            bs.Read(data->t);
            bs.Read(data->r);
            bs.Read(data->k);
            int size;
            bs.Read(size);
            std::vector<char> buffer(size);
            bs.Read(buffer.data(), size);
            std::istringstream in(std::string(buffer.begin(), buffer.end()), std::ios::in | std::ios::binary);
            bool is_keyframe = false;
            bs.Read(is_keyframe);
            if (peer->decoder.get_n_frames() == 0 && is_keyframe == false)
                return;
            bs.Read(size);
            buffer.resize(size);
            bs.Read(buffer.data(), size);
            peer->video_worker.begin([&] {
                peer->decoder(buffer.data(), buffer.size(), data->y_img.data(), data->u_img.data(), data->v_img.data());
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

void Receiver::start(const std::string& record_filename)
{
    stop();
    is_running = true;
    t = boost::thread(std::bind(&Receiver::run, this, record_filename));
}

void Receiver::start_play(const std::string& filename)
{
    stop();
    is_running = true;
    t = boost::thread(std::bind(&Receiver::play, this, filename));
}

void Receiver::start_connect(const std::string& address, const unsigned short port)
{
    stop();
    is_running = true;
    t = boost::thread(std::bind(&Receiver::connect, this, address, port));
}

void Receiver::stop()
{
    if (is_running) {
        is_running = false;
        t.join();
    }
    models.clear();
    peers.clear();
    delete_models.clear();
    new_models.clear();
    updates.clear();
}

void Receiver::draw()
{
    std::list<std::pair<std::uint64_t, std::shared_ptr<Data3d>>> data;
    Lock l(m);
    for (auto& g : delete_models)
        models.erase(g);
    delete_models.clear();
    for (auto& g : new_models)
        models[g].reset(new Model);
    new_models.clear();
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
