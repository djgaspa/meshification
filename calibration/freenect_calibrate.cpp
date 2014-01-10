#include <iostream>
#include <fstream>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <libfreenect.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

std::mutex mutex_frame;
std::condition_variable cond_frame;
bool video_available = false;
bool depth_available = false;

static
std::vector<cv::Point3f> genPatternObjectPoints(const cv::Size& board_size, const float patternTileSize)
{
    std::vector<cv::Point3f> corners;
    for (int i = 0; i < board_size.height; ++i)
        for (int j = 0; j < board_size.width; ++j)
            corners.push_back(cv::Point3f(float((2 * j + i % 2) * patternTileSize), float(i * patternTileSize), 0));
    return corners;
}

static
void log_cb(::freenect_context*, ::freenect_loglevel, const char *msg)
{
    std::clog << "Log: " << msg << std::endl;
}

std::vector<char> buffer_video, buffer_depth;
::freenect_frame_mode buffer_mode;

static
void video_cb(freenect_device* dev, void* rgb, uint32_t timestamp)
{
    const char* buffer = (const char*)rgb;
    const auto n = ::freenect_get_current_video_mode(dev).bytes;
    std::unique_lock<std::mutex> l(mutex_frame);
    buffer_video.resize(n);
    std::copy(buffer, buffer + n, buffer_video.begin());
    video_available = true;
    cond_frame.notify_all();
}

uint16_t t_gamma[2048];
std::vector<char> depth_mid(640 * 480 * 3);
std::vector<unsigned short> depth_buffer(640 * 480);

static
void depth_cb(freenect_device* dev, void* v_depth, uint32_t timestamp)
{
    uint16_t* depth = (uint16_t*)v_depth;
    for (int i = 0; i < 640 * 480; ++i) {
        int pval = t_gamma[depth[i]];
        int lb = pval & 0xff;
        switch (pval >> 8) {
            case 0:
                depth_mid[3 * i + 0] = 255;
                depth_mid[3 * i + 1] = 255 - lb;
                depth_mid[3 * i + 2] = 255 - lb;
                break;
            case 1:
                depth_mid[3 * i + 0] = 255;
                depth_mid[3 * i + 1] = lb;
                depth_mid[3 * i + 2] = 0;
                break;
            case 2:
                depth_mid[3 * i + 0] = 255 - lb;
                depth_mid[3 * i + 1] = 255;
                depth_mid[3 * i + 2] = 0;
                break;
            case 3:
                depth_mid[3 * i + 0] = 0;
                depth_mid[3 * i + 1] = 255;
                depth_mid[3 * i + 2] = lb;
                break;
            case 4:
                depth_mid[3 * i + 0] = 0;
                depth_mid[3 * i + 1] = 255 - lb;
                depth_mid[3 * i + 2] = 255;
                break;
            case 5:
                depth_mid[3 * i + 0] = 0;
                depth_mid[3 * i + 1] = 0;
                depth_mid[3 * i + 2] = 255-lb;
                break;
            default:
                depth_mid[3 * i + 0] = 0;
                depth_mid[3 * i + 1] = 0;
                depth_mid[3 * i + 2] = 0;
                break;
        }
    }
    std::unique_lock<std::mutex> l(mutex_frame);
    std::copy(depth, depth + 640 * 480, depth_buffer.begin());
    depth_available = true;
    cond_frame.notify_all();
}

float getColorSubpix(const cv::Mat& img, cv::Point2f pt)
{
    assert(!img.empty());

    int x = (int)pt.x;
    int y = (int)pt.y;

    int x0 = cv::borderInterpolate(x,   img.cols, cv::BORDER_REFLECT_101);
    int x1 = cv::borderInterpolate(x+1, img.cols, cv::BORDER_REFLECT_101);
    int y0 = cv::borderInterpolate(y,   img.rows, cv::BORDER_REFLECT_101);
    int y1 = cv::borderInterpolate(y+1, img.rows, cv::BORDER_REFLECT_101);

    float a = pt.x - (float)x;
    float c = pt.y - (float)y;

    return ((img.at<unsigned short>(y0, x0) * (1.f - a) + img.at<unsigned short>(y0, x1) * a) * (1.f - c)
                           + (img.at<unsigned short>(y1, x0) * (1.f - a) + img.at<unsigned short>(y1, x1) * a) * c);
}

int main()
{
    for (int i = 0 ; i < 2048; ++i) {
        float v = i / 2048.0;
        v = std::pow(v, 3) * 6;
        t_gamma[i] = v * 6 * 256;
    }

    const cv::Size pattern_size(4, 11);
    //const double pattern_tile_size = 0.03035;
    const double pattern_tile_size = 32.515e-3;
    std::vector<cv::Point3f> pattern_points = ::genPatternObjectPoints(pattern_size, pattern_tile_size);
    std::vector<std::vector<cv::Point3f>> objPoints;

    ::freenect_context* ctx;
    ::freenect_init(&ctx, nullptr);
    ::freenect_set_log_callback(ctx, &log_cb);
    ::freenect_set_log_level(ctx, ::FREENECT_LOG_NOTICE);
    ::freenect_device_attributes* attr;
    const auto n = ::freenect_list_device_attributes(ctx, &attr);
    if (n == 0) {
        ::freenect_free_device_attributes(attr);
        ::freenect_shutdown(ctx);
        std::cerr << "No devices found" << std::endl;
        return 1;
    }
    const std::string camera_serial = attr[0].camera_serial;
    std::cout << "Camera serial: " << camera_serial << std::endl;
    ::freenect_free_device_attributes(attr);
    ::freenect_device* dev;
    ::freenect_open_device(ctx, &dev, 0);
    ::freenect_video_format video_format = ::FREENECT_VIDEO_RGB;
    ::freenect_resolution resolution = ::FREENECT_RESOLUTION_MEDIUM;

    const auto mode = ::freenect_find_video_mode(resolution, video_format);
    cv::Size video_size(mode.width, mode.height);
    ::freenect_set_video_mode(dev, mode);
    ::freenect_set_video_callback(dev, &video_cb);
    ::freenect_start_video(dev);

    auto depth_mode = ::freenect_find_depth_mode(::FREENECT_RESOLUTION_MEDIUM, ::FREENECT_DEPTH_11BIT);
    cv::Size depth_size(depth_mode.width, depth_mode.height);
    ::freenect_set_depth_mode(dev, depth_mode);
    ::freenect_set_depth_callback(dev, &depth_cb);
    ::freenect_start_depth(dev);

    const std::string w = "Real-time Image - " + camera_serial, w2 = "Dataset - " + camera_serial;
    cv::namedWindow(w);
    cv::Mat gray, view(video_size, CV_8UC3), depth(depth_size, CV_16UC1);
    int selected_view = 0, angle_tilt = 0;
    bool tracking = false;
    bool video_mode_changed = false;
    std::mutex mutex_mode_changed;
    std::atomic_bool is_running { true };
    std::thread t([&]() {
        int accelCount = 0;
        while (is_running) {
            if (::freenect_process_events(ctx) != 0)
                std::cerr << "Process events error" << std::endl;
            if (accelCount++ >= 2000) {
                accelCount = 0;
                ::freenect_update_tilt_state(dev);
                ::freenect_raw_tilt_state* state = ::freenect_get_tilt_state(dev);
                double dx,dy,dz;
                ::freenect_get_mks_accel(state, &dx, &dy, &dz);
                printf("\r raw acceleration: %4d %4d %4d  mks acceleration: %4f %4f %4f", state->accelerometer_x, state->accelerometer_y, state->accelerometer_z, dx, dy, dz);
                fflush(stdout);
            }
            if (video_mode_changed == true) {
                ::freenect_stop_video(dev);
                ::freenect_stop_depth(dev);
                const auto mode = ::freenect_find_video_mode(resolution, video_format);
                video_size = cv::Size(mode.width, mode.height);
                ::freenect_set_video_mode(dev, mode);
                ::freenect_start_video(dev);
                if (resolution != ::FREENECT_RESOLUTION_HIGH || video_format != ::FREENECT_VIDEO_IR_8BIT)
                    ::freenect_start_depth(dev);
                ::video_available = false;
                std::unique_lock<std::mutex> l(mutex_mode_changed);
                video_mode_changed = false;
            }
        }
        ::freenect_stop_video(dev);
        ::freenect_stop_depth(dev);
    });

    std::vector<cv::Mat> images;
    std::vector<std::vector<cv::Point2f>> centers;
    cv::Mat camera_matrix, distortion_coefficients;
    std::vector<cv::Mat> rvecs, tvecs;
    double reprojection_error = 0.0;

    std::vector<cv::Mat> stereo_rgb, stereo_ir;
    std::vector<std::vector<cv::Point2f>> stereo_rgb_centers, stereo_ir_centers;
    cv::Mat stereo_tmp_rgb;
    std::vector<cv::Point2f> stereo_tmp_centers;

    auto update_secondary_view = [&] () {
        const int i = selected_view;
        if (i >= images.size())
            return;
        cv::Mat view;
        cv::undistort(images[i], view, camera_matrix, distortion_coefficients);
        std::ostringstream text;
        text << i + 1 << " / " << images.size();
        const int font = cv::FONT_HERSHEY_PLAIN;
        cv::putText(view, text.str(), cv::Point(50, 100), font, 1.0, cv::Scalar(0, 255, 0));
        std::vector<cv::Point2f> reprojected_points;
        cv::projectPoints(objPoints[i], rvecs[i], tvecs[i], camera_matrix, distortion_coefficients, reprojected_points);
        double re = 0.0;
        for (int j = 0; j < centers[i].size(); ++j) {
            const double e = cv::norm(centers[i][j] - reprojected_points[j]);
            re += e * e;
        }
        text.str("");
        text << "Reprojection error: " << std::sqrt(re / centers[i].size());
        cv::putText(view, text.str(), cv::Point(50, 150), font, 1.0, cv::Scalar(0, 255, 0));
        cv::imshow(w2, view);
    };

    auto reset = [&] () {
        images.clear();
        centers.clear();
        camera_matrix.release();
        distortion_coefficients.release();
        rvecs.clear();
        tvecs.clear();
        reprojection_error = 0.0;
        cv::destroyWindow(w2);
    };

    cv::Size rgb_camera_size, ir_camera_size;
    cv::Mat rgb_camera_matrix, rgb_distortion_coefficients, ir_camera_matrix, ir_distortion_coefficients;
    double rgb_reprojection_error = 0.0, ir_reprojection_error = 0.0;
    double alpha = 0.0, beta = 0.0;
    cv::Mat R, T;

    auto load = [&] () {
        cv::FileStorage f(camera_serial + ".yml", cv::FileStorage::READ);
        if (f.isOpened() == false)
            return;
        f["rgb_camera_size"] >> rgb_camera_size;
        f["ir_camera_size"] >> ir_camera_size;
        f["rgb_camera_matrix"] >> rgb_camera_matrix;
        f["ir_camera_matrix"] >> ir_camera_matrix;
        f["rgb_distortion_coefficients"] >> rgb_distortion_coefficients;
        f["ir_distortion_coefficients"] >> ir_distortion_coefficients;
        f["rgb_reprojection_error"] >> rgb_reprojection_error;
        f["ir_reprojection_error"] >> ir_reprojection_error;
        f["alpha"] >> alpha;
        f["beta"] >> beta;
        f["R"] >> R;
        f["T"] >> T;
    };

    auto save = [&] () {
        cv::FileStorage f(camera_serial + ".yml", cv::FileStorage::WRITE);
        if (f.isOpened() == false)
            return;
        const std::time_t now = std::time(nullptr);
        f << "camera_serial" << camera_serial;
        f << "datetime" << std::ctime(&now);
        f << "rgb_camera_size" << rgb_camera_size;
        f << "rgb_camera_matrix" << rgb_camera_matrix;
        f << "rgb_distortion_coefficients" << rgb_distortion_coefficients;
        f << "rgb_reprojection_error" << rgb_reprojection_error;
        f << "ir_camera_size" << ir_camera_size;
        f << "ir_camera_matrix" << ir_camera_matrix;
        f << "ir_distortion_coefficients" << ir_distortion_coefficients;
        f << "ir_reprojection_error" << ir_reprojection_error;
        f << "alpha" << alpha;
        f << "beta" << beta;
        f << "R" << R;
        f << "T" << T;
        f << "image_width" << 640;
        f << "image_height" << 480;
        cv::Mat midres_rgb_camera_matrix = rgb_camera_matrix.clone();
        midres_rgb_camera_matrix.at<double>(0, 0) /= 2.0;
        midres_rgb_camera_matrix.at<double>(1, 1) /= 2.0;
        midres_rgb_camera_matrix.at<double>(0, 2) /= 2.0;
        midres_rgb_camera_matrix.at<double>(1, 2) /= 2.0;
        f << "camera_matrix" << midres_rgb_camera_matrix;
        f << "distortion_coefficients" << rgb_distortion_coefficients;
        cv::Mat midres_ir_camera_matrix = ir_camera_matrix.clone();
        midres_ir_camera_matrix.at<double>(0, 0) /= 2.0;
        midres_ir_camera_matrix.at<double>(1, 1) /= 2.0;
        midres_ir_camera_matrix.at<double>(0, 2) = midres_ir_camera_matrix.at<double>(0, 2) / 2.0 - 4.8;
        midres_ir_camera_matrix.at<double>(1, 2) = midres_ir_camera_matrix.at<double>(1, 2) / 2.0 - 14.0 - 3.9;
        f << "irCameraMatrix" << midres_ir_camera_matrix;
        f << "irDistCoeffs" << ir_distortion_coefficients;
    };

    std::vector<cv::Point2f> depth_calibration_points;

    auto calibrate = [&] () {
        if (centers.empty() == true)
            return;
        objPoints.resize(centers.size(), pattern_points);
        reprojection_error = cv::calibrateCamera(objPoints, centers, video_size, camera_matrix, distortion_coefficients, rvecs, tvecs);
        update_secondary_view();
        for (int i = 0; i < objPoints.size(); ++i) {
            Eigen::Vector3d r = Eigen::Map<Eigen::Vector3d>(rvecs[i].ptr<double>());
            Eigen::Vector3d t = Eigen::Map<Eigen::Vector3d>(tvecs[i].ptr<double>());
            Eigen::Affine3d a = Eigen::Affine3d::Identity();
            a.translate(t).rotate(Eigen::AngleAxisd(r.norm (), r.normalized()));
            const Eigen::Matrix4f m = a.matrix().cast<float>();
            for (int j = 0; j < centers[i].size(); ++j) {
                const cv::Point3f& p(objPoints[i][j]);
                Eigen::Vector4f v = m * Eigen::Vector4f(p.x, p.y, p.z, 1);
            }
        }
        if (video_format == ::FREENECT_VIDEO_RGB) {
            rgb_camera_size = video_size;
            rgb_camera_matrix = camera_matrix;
            rgb_distortion_coefficients = distortion_coefficients;
            rgb_reprojection_error = reprojection_error;
        }
        else {
            ir_camera_size = video_size;
            ir_camera_matrix = camera_matrix;
            ir_distortion_coefficients = distortion_coefficients;
            ir_reprojection_error = reprojection_error;
        }
    };

    load();
    bool is_motor_activated = false;

    while (true) {
        bool capture_intrinsic = false, capture_baseline = false, capture_stereo_rgb = false, capture_stereo_ir = false;
        const auto c = cv::waitKey(1);
        if (c == 27) {
            is_running = false;
            break;
        }
        else if (c == 'f') {
            video_format = video_format == ::FREENECT_VIDEO_IR_8BIT ? ::FREENECT_VIDEO_RGB : ::FREENECT_VIDEO_IR_8BIT;
            std::unique_lock<std::mutex> l(mutex_mode_changed);
            video_mode_changed = true;
            reset();
        }
        else if (c == 'r') {
            resolution = resolution == ::FREENECT_RESOLUTION_HIGH ? ::FREENECT_RESOLUTION_MEDIUM : ::FREENECT_RESOLUTION_HIGH;
            std::unique_lock<std::mutex> l(mutex_mode_changed);
            video_mode_changed = true;
            reset();
        }
        else if (c == 32)
            capture_intrinsic = true;
        else if (c == 'z')
            capture_baseline = true;
        else if (c == 'v')
            capture_stereo_rgb = true;
        else if (c == 'b')
            capture_stereo_ir = true;
        else if (c == 'm')
            is_motor_activated = !is_motor_activated;
        else if (c == '+') {
            selected_view = images.empty() ? 0 : std::min(selected_view + 1, int(images.size()) - 1);
            update_secondary_view();
        }
        else if (c == '-') {
            selected_view = std::max(0, selected_view - 1);
            update_secondary_view();
        }
        else if (c == 65535 && images.empty() == false) {
            images.erase(images.begin() + selected_view);
            centers.erase(centers.begin() + selected_view);
            selected_view = images.empty() ? 0 : std::min(selected_view, int(images.size()) - 1);
            calibrate();
        }
        else if (c == 'w')
            angle_tilt = std::min(angle_tilt + 1, 30);
        else if (c == 's')
            angle_tilt = 0;
        else if (c == 'x')
            angle_tilt = std::max(angle_tilt - 1, -30);
        else if (c == 't')
            tracking = !tracking;
        else if (c == 'l')
            load();
        else if (c == 'p')
            save();

        if (is_motor_activated)
            ::freenect_set_tilt_degs(dev, angle_tilt);

        std::unique_lock<std::mutex> l(mutex_frame);
        if (cond_frame.wait_for(l, std::chrono::milliseconds(20), [] {
                                return video_available == true || depth_available == true;
    }) == true) {
            if (depth_available == true) {
                cv::Mat(depth_size, CV_16UC1, depth_buffer.data()).copyTo(depth);
                cv::Mat depth_view(depth_size, CV_8UC3, depth_mid.data());
                cv::imshow("Depth", depth_view);
                depth_available = false;
            }
            if (video_available == true && ::freenect_find_video_mode(resolution, video_format).bytes == buffer_video.size()) {
                if (video_format == ::FREENECT_VIDEO_IR_8BIT) {
                    cv::Mat mat(video_size, CV_8UC1, buffer_video.data());
                    mat.copyTo(gray);
                    cv::cvtColor(gray, view, CV_GRAY2BGR);
                }
                else if (video_format == ::FREENECT_VIDEO_RGB) {
                    cv::Mat mat(video_size, CV_8UC3, buffer_video.data());
                    cv::cvtColor(mat, view, CV_RGB2BGR);
                    cv::cvtColor(mat, gray, CV_RGB2GRAY);
                }
                video_available = false;
            }
        }
        l.unlock();

        std::vector<cv::Point2f> p;
        const bool found = tracking && cv::findCirclesGrid(gray, pattern_size, p, cv::CALIB_CB_ASYMMETRIC_GRID);
        double current_reprojection_error = 0.0;
        cv::Mat rvec, tvec;
        if (found) try {
            cv::Mat& cam = video_format == ::FREENECT_VIDEO_RGB ? rgb_camera_matrix : ir_camera_matrix;
            cv::Mat& dist = video_format == ::FREENECT_VIDEO_RGB ? rgb_distortion_coefficients : ir_distortion_coefficients;
            cv::solvePnP(pattern_points, p, cam, dist, rvec, tvec);
            std::vector<cv::Point2f> reprojected_points;
            cv::projectPoints(pattern_points, rvec, tvec, cam, dist, reprojected_points);
            for (int j = 0; j < p.size(); ++j) {
                const double e = cv::norm(p[j] - reprojected_points[j]);
                current_reprojection_error += e * e;
            }
            current_reprojection_error = std::sqrt(current_reprojection_error / p.size());
        } catch (const std::exception&) {
        }

        ::freenect_set_led(dev, tracking ? found ? ::LED_GREEN : ::LED_RED : ::LED_YELLOW);
        {
            cv::Mat m = view.clone();
            cv::drawChessboardCorners(m, pattern_size, p, found);
            std::ostringstream text;
            text << "Average reprojection error: " << reprojection_error;
            cv::putText(m, text.str(), cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 255, 0));
            text.str("");
            text << "Current reprojection error: " << current_reprojection_error;
            cv::putText(m, text.str(), cv::Point(10, 60), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 255, 0));
            cv::imshow(w, m);
        }

        if (found == false)
            continue;

        if (capture_intrinsic == true) {
            centers.push_back(p);
            images.push_back(view.clone());
            selected_view = images.size() - 1;
            calibrate();
        }
        if (capture_baseline == true && resolution == ::FREENECT_RESOLUTION_HIGH && video_format == ::FREENECT_VIDEO_IR_8BIT) {
            cv::Mat depth_color(depth_size, CV_8UC3, depth_mid.data());
            cv::Mat ir;
            cv::resize(view, ir, cv::Size(0, 0), 0.5, 0.5, cv::INTER_LANCZOS4);
            for (int x = 0; x < 640 - 8; ++x)
                for (int y = 0; y < 480; ++y) {
                    const cv::Vec3b d = depth_color.at<cv::Vec3b>(y, x);
                    if (d[0] == 0 && d[1] == 0 && d[2] == 0)
                        continue;
                    cv::Vec3b& c = ir.at<cv::Vec3b>(y + 14 + 3, x + 4);
                    c = c * 0.5 + d * 0.5;
                }
            cv::imshow("Baseline and offset calibration", ir);

            std::cout << "Pose estimation reprojection error: " << current_reprojection_error << " px" << std::endl;
            Eigen::Vector3d r = Eigen::Map<Eigen::Vector3d>(rvec.ptr<double>());
            Eigen::Vector3d t = Eigen::Map<Eigen::Vector3d>(tvec.ptr<double>());
            Eigen::Affine3d a = Eigen::Affine3d::Identity();
            a.translate(t).rotate(Eigen::AngleAxisd(r.norm (), r.normalized()));
            const Eigen::Matrix4f m = a.matrix().cast<float>();

            auto kd_to_z = [] (double kd, double alpha, double beta) {
                return 1.0 / (alpha * kd + beta);
            };

            for (int i = 0; i < p.size(); ++i) {
                cv::Point2f point(p[i].x / 2.0 - 4.8, p[i].y / 2.0 - 14.0 - 3.9);
                const auto kd = ::getColorSubpix(depth, point);
                if (kd == 0.0)
                    continue;
                const auto& o = pattern_points[i];
                const auto v = m * Eigen::Vector4f(o.x, o.y, o.z, 1);
                depth_calibration_points.push_back(cv::Point2f(kd, 1.0f / v[2]));
            }
            cv::Mat line;
            cv::fitLine(depth_calibration_points, line, CV_DIST_L2, 0.0, 0.01, 0.01);
            alpha = line.at<float>(1) / line.at<float>(0);
            beta = -alpha * line.at<float>(2) + line.at<float>(3);
            std::cout << "z = (" << alpha << " x + " << beta << ")^(-1)" << std::endl;

            const std::string points_filename = camera_serial + "_depth_points.txt";
            const std::string script_filename = camera_serial + "_script.gnuplot";
            const std::string graph_filename = camera_serial + "_depth_graph.png";
            std::ofstream out(points_filename.c_str());
            double error = 0.0;
            for (const auto& p : depth_calibration_points) {
                const double e = 1.0 / p.y - kd_to_z(p.x, alpha, beta);
                error += e * e;
                out << p.x << ' ' << p.y << ' ' << e << '\n';
            }
            out << std::flush;
            std::cout << "Reprojection error calibrated: " << 1000 * std::sqrt(error / depth_calibration_points.size()) << " mm" << std::endl;

            std::ofstream out_script(script_filename);
            out_script << "set terminal png size 1024, 768\n"
                          "set output '" << graph_filename << "'\n"
                          "set title 'Depth/disparity relation'\n"
                          "set xlabel 'Disparity'\n"
                          "set ylabel 'Depth'\n"
                          "set x2tics\n"
                          "y(x) = 1.0 / (" << alpha << " * x + " << beta << ')' <<
                          "\ne(x) = a * x + b\n"
                          "fit e(x) '" << points_filename << "' using (1.0 / $2):(abs($3)) via a, b\n"
                          "f(x) = (x - b) / a"
                          "\nz(x) = " << alpha << " * x + " << beta <<
                          "\nplot '" << points_filename << "' using 1:(1.0 / $2) title 'Points', y(x) title 'Fitting curve'"
                          ", '' using 1:2 t 'Inverse Z Points' axes x1y2, z(x) t 'Inverse Z line' axes x1y2"
                          ", '' using (abs($3)):(1.0 / $2) t 'Residuals' axes x2y1, f(x) t 'Residual MDA' axes x2y1" << std::endl;
            if (std::system(("gnuplot " + script_filename).c_str()) == 0) {
                const auto graph = cv::imread(graph_filename);
                cv::imshow("Depth - Disparity Graph", graph);
            }
            else
                std::cerr << "Unable to create depth calibration graph" << std::endl;
        }
        if (capture_stereo_rgb) {
            stereo_tmp_rgb = view.clone();
            stereo_tmp_centers = p;
            std::cout << "Stereo RGB Taken" << std::endl;
        }
        if (capture_stereo_ir) {
            stereo_rgb.push_back(stereo_tmp_rgb);
            stereo_rgb_centers.push_back(stereo_tmp_centers);
            stereo_ir.push_back(view.clone());
            stereo_ir_centers.push_back(p);
            if (stereo_rgb_centers.size() != stereo_ir_centers.size())
                std::cerr << "Error stereo samples are missing" << std::endl;
            std::cout << "Stereo calibration on " << stereo_ir_centers.size() << " patterns" << std::endl;
            std::vector<std::vector<cv::Point3f>> objPoints(stereo_ir_centers.size(), pattern_points);
            cv::Mat E, F;
            reprojection_error = cv::stereoCalibrate(objPoints, stereo_rgb_centers, stereo_ir_centers, rgb_camera_matrix, rgb_distortion_coefficients, ir_camera_matrix, ir_distortion_coefficients, cv::Size(1280, 1024), R, T, E, F);
        }
    }
    t.join();
    ::freenect_set_led(dev, ::LED_BLINK_GREEN);
    ::freenect_close_device(dev);
    ::freenect_shutdown(ctx);
}
