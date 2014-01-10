#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

static
std::vector<cv::Point3f> genPatternObjectPoints(const cv::Size& board_size, const float patternTileSize)
{
    std::vector<cv::Point3f> corners;
    for (int i = 0; i < board_size.height; ++i)
        for (int j = 0; j < board_size.width; ++j)
            corners.push_back(cv::Point3f(float((2 * j + i % 2) * patternTileSize), float(i * patternTileSize), 0));
    return corners;
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " camera_serial0 camera_serial1" << std::endl;
        return 1;
    }
    cv::Mat camera_matrix1, dist_coeffs1, camera_matrix2, dist_coeffs2;
    const std::string dir1 = argv[1], dir2 = argv[2];
    const std::string calib1 = dir1 + ".yml", calib2 = dir2 + ".yml";
    cv::FileStorage in1(calib1, cv::FileStorage::READ), in2(calib2, cv::FileStorage::READ);
    if (in1.isOpened() == false) {
        std::cerr << "Unable to open calibration file " << calib1 << std::endl;
        return 1;
    }
    if (in2.isOpened() == false) {
        std::cerr << "Unable to open calibration file " << calib2 << std::endl;
        return 1;
    }
    in1["ir_camera_matrix"] >> camera_matrix1;
    in1["ir_distortion_coefficients"] >> dist_coeffs1;
    in2["ir_camera_matrix"] >> camera_matrix2;
    in2["ir_distortion_coefficients"] >> dist_coeffs2;
    const cv::Size pattern_size(4, 11);
    std::vector<cv::Point3f> points = ::genPatternObjectPoints(pattern_size, 0.032515);
    std::vector<std::vector<cv::Point2f>> image_points1, image_points2;
    for (int i = 0; i < 32; ++i) {
        std::ostringstream o;
        o << '/' << i << ".png";
        const std::string filename = o.str();
        cv::Mat image1 = cv::imread(dir1 + filename);
        cv::Mat image2 = cv::imread(dir2 + filename);
        if (image1.empty() || image2.empty())
            continue;
        std::vector<cv::Point2f> centers1, centers2;
        const bool found1 = cv::findCirclesGrid(image1, pattern_size, centers1, cv::CALIB_CB_ASYMMETRIC_GRID);
        const bool found2 = cv::findCirclesGrid(image2, pattern_size, centers2, cv::CALIB_CB_ASYMMETRIC_GRID);
        if (found1 == false || found2 == false) {
            std::cerr << "Images with index " << i << " does not contain a valid pattern" << std::endl;
            continue;
        }
        image_points1.push_back(centers1);
        image_points2.push_back(centers2);
    }
    std::vector<std::vector<cv::Point3f>> object_points(image_points1.size(), points);
    cv::Mat R, T, E, F;
    const double e = cv::stereoCalibrate(object_points, image_points1, image_points2, camera_matrix1, dist_coeffs1, camera_matrix2, dist_coeffs2, cv::Size(1280, 1024), R, T, E, F);
    std::cout << "R: " << R << "\n\nT: " << T << "\n\nCalibration error: " << e << std::endl;

    cv::Mat rvec;
    cv::Rodrigues(R, rvec);
    Eigen::Vector3d r = Eigen::Map<Eigen::Vector3d>(rvec.ptr<double>());
    Eigen::Vector3d t = Eigen::Map<Eigen::Vector3d>(T.ptr<double>());
    Eigen::Affine3d a = Eigen::Affine3d::Identity();
    a.rotate(Eigen::AngleAxisd(r.norm (), r.normalized())).translate(t);
    const Eigen::Matrix4f matrix = a.matrix().cast<float>();
    std::cout << "OpenGL transformation matrix:\n" << matrix << std::endl;
}
