#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

class Model;
struct Data3d;
struct Peer;

namespace RakNet {
struct Packet;
}

class Receiver
{
    std::thread t;
    bool is_running = false;

    using Mutex = std::mutex;
    using Lock = std::unique_lock<Mutex>;
    Mutex m;
    std::unordered_set<std::uint64_t> new_models;
    std::unordered_set<std::uint64_t> delete_models;
    std::unordered_map<std::uint64_t, std::shared_ptr<Data3d>> updates;

    std::unordered_map<std::uint64_t, std::shared_ptr<Model>> models;

    std::unordered_map<std::uint64_t, std::shared_ptr<Peer>> peers;
    void run(const std::string& filename);
    void play(const std::string& filename);
    void connect(const std::string& address, const unsigned short port);
    void process_packet(std::shared_ptr<RakNet::Packet> p);

public:
    Receiver();
    ~Receiver();
    static void init();
    void start(const std::string& record_filename = "");
    void start_play(const std::string& filename);
    void start_connect(const std::string& address, const unsigned short port = 12344);
    void stop();
    void draw();

    void translate(const std::string& name, const double x, const double y, const double z);
    void rotate(const std::string &name, const double rad, const double x, const double y, const double z);
    void reset_position(const std::string& name);
    void save_view() const;
};
