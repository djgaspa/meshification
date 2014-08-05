#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>
#include <opencv2/core/core.hpp>
#include "SourceFreenect.hpp"
#include "DepthMeshifier.hpp"
#include "Consumer.hpp"
#include "../common/AsyncWorker.hpp"

int main()
try {
    const int cam_id = 0;
    const std::string address = "127.0.0.1";
    const std::string name = "Kinect0";
    const unsigned short local_port = 12344;

    SourceFreenect camera(cam_id);
    camera.startImage();
    camera.startDepth();

    const std::string calib_file(camera.getSerial() + ".yml");
    DepthMeshifier meshify(calib_file);
    Consumer consume(address, name, calib_file, local_port);

    const int width = camera.width(), height = camera.height();
    using clock = std::chrono::high_resolution_clock;
    std::vector<char> buffer_depth(2 * width * height), buffer_rgb(3 * width * height);
    AsyncWorker consumer_worker;
    std::atomic_bool is_running(true);
    std::thread t([&is_running] {
        std::cin.get();
        is_running = false;
    });
    for (int i = 0; is_running == true; ++i) {
        camera.grab();
        camera.getImage(buffer_rgb.data());
        camera.getDepth(buffer_depth.data());
        const auto t0 = clock::now();
        std::vector<unsigned> tri;
        std::vector<float> ver;
        meshify(buffer_rgb.data(), buffer_depth.data(), tri, ver);
        if (tri.empty() == false) {
            consumer_worker.begin([=, &consume] {
                consume(ver, tri, buffer_rgb);
            });
        }
        const auto t1 = clock::now();
        const auto t = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        if (i % 60 == 0)
            std::cout << "Time: " << t << std::endl;
    }
    t.join();
}
catch (const std::exception& e) {
    std::cerr << "Fatal exception: " << e.what() << std::endl;
}
