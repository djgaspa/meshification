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

#include <boost/program_options.hpp>
#include <QApplication>
#include <opencv/cv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "AcquisitionUI.hpp"

void compare(char* orig_buffer, unsigned short* final_buffer, const cv::Size& size)
{
    cv::Mat depth_orig(size, CV_16UC1, orig_buffer), depth_final(size, CV_16UC1, final_buffer);
    const int n_points = depth_orig.total();
    for (int i = 0; i < n_points; ++i)
        if (final_buffer[i] > 9000)
            final_buffer[i] = 0;
    cv::imshow("Orig Depth", depth_orig);
    cv::imshow("Final Depth", depth_final);
    int max_error = 0;
    cv::Mat error(depth_orig.size(), CV_16UC1), error_mask(depth_orig.size(), CV_8UC1);
    for (int i = 0; i < n_points; ++i) {
        const unsigned short orig = depth_orig.at<unsigned short>(i);
        const unsigned short final = depth_final.at<unsigned short>(i);
        const bool is_valid = orig != 0 && final < 9000;
        const int e = is_valid ? std::abs(orig - final) : 0;
        error.at<unsigned short>(i) = e;
        error_mask.at<unsigned char>(i) = is_valid ? 255 : 0;
        if (e > max_error)
            max_error = e;
    }
    cv::Mat error_graph(depth_orig.size(), CV_8UC3);
    double mse = 0, valid_points = 0;
    for (int i = 0; i < n_points; ++i) {
        const bool is_valid = error_mask.at<unsigned char>(i) != 0;
        const unsigned short e = error.at<unsigned short>(i);
        if (is_valid) {
            ++valid_points;
            mse += e * e;
        }
        const float tmp = e / 40.0f;
        const float v = std::min(1.0f, e / 40.0f);
        const int r = (v > 0.5f ? 0 : v > 0.25f ? -4 * v + 2 : 1.0f) * 255;
        const int g = (v > 0.75f ? -4 * v + 4 : v > 0.25f ? 1.0f : 4 * v) * 255;
        const int b = (v > 0.75f ? 1.0f : v > 0.5f ? 4 * v - 2 : 0.0f) * 255;
        error_graph.at<cv::Vec3b>(i) = is_valid == false ? cv::Vec3b(0, 0, 0) : tmp > 1.0f ? cv::Vec3b(255, 0, 255) : cv::Vec3b(b, g, r);
    }
    std::cout << "MSE: " << mse / valid_points << " (" << valid_points << ')' << std::endl;
    cv::imshow("Error", error_graph);
}

int main(int argc, char** argv)
try {
    QApplication app(argc, argv);
    std::string address, name, camera_calibration;
    int width, height, cam_id;
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("address,a", po::value<std::string>(&address)->default_value("127.0.0.1"), "Destination address")
            ("calib,c", po::value<std::string>(&camera_calibration)->default_value("cam.yml"))
            ("width,W", po::value<int>(&width)->default_value(640), "Range image width")
            ("height,H", po::value<int>(&height)->default_value(480), "Range image height")
            ("camera_id,i", po::value<int>(&cam_id)->default_value(0), "Index of the camera")
            ("name,n", po::value<std::string>(&name)->default_value("default"));

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }
    AcquisitionUI ui;
    ui.show();
    return app.exec();
} catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
}
