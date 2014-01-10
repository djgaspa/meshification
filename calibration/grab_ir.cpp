#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>
#include "../acquisition/SourceFreenect.hpp"

int main()
try {
    SourceFreenect device(0);
    device.startIr();
    cv::Mat image(1024, 1280, CV_8UC1);
    const std::string w = "Ir Grab", t = "Image index";
    int i = 0;
    cv::namedWindow(w);
    cv::createTrackbar(t, w, &i, 64);
    while (true) {
        bool capture = false;
        const auto c = cv::waitKey(1);
        if (c == 27)
            break;
        if (c == 32)
            capture = true;
        device.grab();
        device.getIr((char*)image.data);
        cv::Mat view;
        cv::cvtColor(image, view, CV_GRAY2BGR);
        std::vector<cv::Point2f> centers;
        const cv::Size pattern_size(4, 11);
        const bool found = cv::findCirclesGrid(image, pattern_size, centers, cv::CALIB_CB_ASYMMETRIC_GRID);
        cv::drawChessboardCorners(view, pattern_size, centers, found);
        cv::imshow(w, view);
        if (capture == true && found == true) {
            std::ostringstream o;
            o << device.getSerial() << '/' << i << ".png";
            if (cv::imwrite(o.str(), image, std::vector<int>{CV_IMWRITE_PNG_COMPRESSION, 9}) == true)
                cv::setTrackbarPos(t, w, i + 1);
            else
                std::cerr << "Can't save the image. Assure that the directory " << device.getSerial() << " exists" << std::endl;
        }
    }
    device.stopAll();
    cv::destroyAllWindows();
}
catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
}
