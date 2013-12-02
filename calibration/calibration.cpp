#include <memory>
#include <iostream>
#include <sstream>
#include <fstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../acquisition/SourceKinectOpenNI.hpp"

// parallel sensor usage:
// RGB+DEPTH => OK
// RGB+IR => FAIL
// DEPTH+IR => OK

// sensor switching:
// IR->RGB => slow
// RGB->IR => fast

/*//////////////////////////////////////////////////////////////////////
// types
//////////////////////////////////////////////////////////////////////*/

enum class CalibrationMode
{
    Ir,
    Rgb,
    Stereo,
    Check
};

/*//////////////////////////////////////////////////////////////////////
// constants
//////////////////////////////////////////////////////////////////////*/

const cv::Size patternSize(4, 11);
const float patternTileSize = 0.0231f; // in meters

const cv::Size frameSize(640, 480); // TODO make use of it!

const char *frameNamePrimary = "Primary Frame";
const char *frameNameSecondary = "Secondary Frame";

const cv::Scalar textColorRed(0, 0, 255);
const cv::Scalar textColorBlue(255, 0, 0);
const cv::Scalar textColorWhite(255, 255, 255);
const int textFont = cv::FONT_HERSHEY_SIMPLEX;
const float textScale = 1.0f;
const int textIndent = 10;
const int textHeight = 30;

/*//////////////////////////////////////////////////////////////////////
// globals
//////////////////////////////////////////////////////////////////////*/

// openNI context and generator nodes
std::unique_ptr<Source> kinect;

// IR camera point correspondences (multiple views)
std::vector<std::vector<cv::Point3f>> irObjPoints;
std::vector<std::vector<cv::Point2f>> irImgPoints;

// RGB camera point correspondences (multiple views)
std::vector<std::vector<cv::Point3f>> rgbObjPoints;
std::vector<std::vector<cv::Point2f>> rgbImgPoints;

// IR+RGB stereo correspondences
std::vector<std::vector<cv::Point3f>> stereoObjPoints;
std::vector<std::vector<cv::Point2f>> stereoIrImgPoints;
std::vector<std::vector<cv::Point2f>> stereoRgbImgPoints;

// IR camera calibration data
cv::Mat irCameraMatrix;
cv::Mat irDistCoeffs;

// RGB camera calibration data
cv::Mat rgbCameraMatrix;
cv::Mat rgbDistCoeffs;

// stereo calibration
cv::Mat R, T, E, F, rvec;

// image buffers
cv::Mat irMat16(frameSize, CV_16UC1);
cv::Mat irMat8(frameSize, CV_8UC1);
cv::Mat irMat8Rgb(frameSize, CV_8UC3);
cv::Mat irMat8RgbUndist(frameSize, CV_8UC1);

cv::Mat rgbMat(frameSize, CV_8UC3); // note: this is actually BGR
cv::Mat rgbMatGray(frameSize, CV_8UC1);
cv::Mat rgbMatUndist(frameSize, CV_8UC3);

cv::Mat depthMat16(frameSize, CV_16UC1);
cv::Mat depthMat8(frameSize, CV_8UC1);

cv::Mat overlayMat(frameSize, CV_8UC3);
cv::Mat overlayCalibMat(frameSize, CV_8UC3);

/*//////////////////////////////////////////////////////////////////////
// functions
//////////////////////////////////////////////////////////////////////*/

void acquireIr(float alpha)
{
    cv::Mat mat(frameSize, CV_16UC1);
    kinect->getIr((char*)mat.data);
    mat.copyTo(irMat16);
    irMat16.convertTo(irMat8, CV_8UC1, alpha);
    cv::cvtColor(irMat8, irMat8Rgb, CV_GRAY2RGB);
}

void acquireDepth(float alpha)
{
    cv::Mat mat(frameSize, CV_16UC1);
    kinect->getDepth((char*)mat.data);
    mat.copyTo(depthMat16);
    depthMat16.convertTo(depthMat8, CV_8UC1, alpha);
}

void acquireRgb()
{
    cv::Mat rgbTemp(frameSize, CV_8UC3);
    kinect->getImage((char*)rgbTemp.data);
    cv::cvtColor(rgbTemp, rgbMat, CV_RGB2BGR);
    cv::cvtColor(rgbTemp, rgbMatGray, CV_RGB2GRAY);
}

void textLine(cv::Mat &img, const std::string& line, int lineNumber, cv::Scalar color)
{
    cv::putText(img, line, cv::Point(textIndent, lineNumber * textHeight), textFont, textScale, color);
}

void overlay(cv::Mat &rgb, cv::Mat &gray, cv::Mat &overlay)
{
    unsigned char *rgbData = (unsigned char *) rgb.data;
    unsigned char *grayData = (unsigned char *) gray.data;
    unsigned char *overlayData = (unsigned char *) overlay.data;

    int n = frameSize.width * frameSize.height;
    int valid_points = 0;
    std::vector<float> histogram(256);
    for (int i = 0; i < n; ++i) {
        if (grayData[i] == 0)
            continue;
        ++histogram[grayData[i]];
        ++valid_points;
    }
    for (int i = 1; i < 256; ++i)
        histogram[i] += histogram[i - 1];
    for (int i = 1; i < 256; ++i)
        if (histogram[i] != 0)
            histogram[i] = (valid_points - histogram[i]) * 255 / valid_points;
    for (int i = 0; i < n; ++i) {
        if (grayData[i] == 0) {
            overlayData[3 * i + 0] = rgbData[3 * i + 0];
            overlayData[3 * i + 1] = rgbData[3 * i + 1];
            overlayData[3 * i + 2] = rgbData[3 * i + 2];
        } else {
            overlayData[3 * i + 0] = rgbData[3 * i + 0] * 0.4;
            overlayData[3 * i + 1] = rgbData[3 * i + 1] * 0.4 + histogram[grayData[i]] * 0.6;
            overlayData[3 * i + 2] = rgbData[3 * i + 2] * 0.4 + histogram[grayData[i]] * 0.6;
        }
    }
}

void initCaptureMode(bool initImage, bool initDepth, bool initIr)
{
    kinect->stopAll();
    if (initImage)
        kinect->startImage();
    if (initDepth)
        kinect->startDepth();
    if (initIr)
        kinect->startIr();
}

void reconstruct(cv::Mat &depthMat, cv::Mat &cameraMatrix, cv::Mat &distCoeffsMat, std::vector<cv::Point3f> &points)
{
    int width = depthMat.size().width;
    int height = depthMat.size().height;

    unsigned short *depth = (unsigned short *) depthMat.data;

    // undistort points (only once)
    static std::vector<cv::Point2f> imgCoordsDist(width * height);
    static std::vector<cv::Point2f> imgCoordsUndist(width * height);
    static bool firstCall = true;

    if (firstCall) {
        for (int u = 0; u < width; ++u) {
            for (int v = 0; v < height; ++v) {
                int i = u + v * width;
                imgCoordsDist[i] = cv::Point2f(u, v);
            }
        }
        cv::undistortPoints(cv::Mat(imgCoordsDist), imgCoordsUndist, cameraMatrix, distCoeffsMat);
        firstCall = false;
    }

    // reconstruct 3d coordinates
    for (int u = 0; u < width; ++u) {
        for (int v = 0; v < height; ++v) {
            int i = u + v * width;
            float xh = imgCoordsUndist[i].x;
            float yh = imgCoordsUndist[i].y;

            float Z = depth[i] / 1000.0f; // TODO pythagoras

            points[i] = cv::Point3f(xh * Z, yh * Z, Z);
        }
    }
}

void mapDepthToRgb(cv::Mat &src, cv::Mat &dst, cv::Mat rvec, cv::Mat T, cv::Mat irCameraMatrix, cv::Mat irDistCoeffs, cv::Mat rgbCameraMatrix, cv::Mat rgbDistCoeffs)
{
    // TODO
    static int n = frameSize.width * frameSize.height;
    static int width = frameSize.width;
    static std::vector<cv::Point3f> reconstructedPoints(n);
    static std::vector<cv::Point2f> projectedPoints(n);

    // reconstruct 3d points
    reconstruct(src, irCameraMatrix, irDistCoeffs, reconstructedPoints);

    // project 3d points to rgb coordinates
    cv::projectPoints(cv::Mat(reconstructedPoints), -rvec, -T, rgbCameraMatrix, rgbDistCoeffs, projectedPoints);

    // create calibrated depth map
    unsigned short *srcData = (unsigned short *) src.data;
    unsigned short *dstData = (unsigned short *) dst.data;
    for (int i=0; i<n; i++) {
        int u = (int) projectedPoints[i].x;
        int v = (int) projectedPoints[i].y;
        int j = u + v * width;
        if (j >= 0 && j < n && srcData[i] > 0) {
            if (dstData[j] == 0) {
                dstData[j] = srcData[i];
            } else {
                if (dstData[j] > srcData[i]) {
                    dstData[j] = srcData[i];
                }
            }
        }
    }
}

void genPatternObjectPoints(const cv::Size& board_size, std::vector<cv::Point3f>& corners)
{
    corners.clear();
    for (int i = 0; i < board_size.height; ++i)
        for (int j = 0; j < board_size.width; ++j)
            corners.push_back(cv::Point3f(float((2 * j + i % 2) * patternTileSize), float(i * patternTileSize), 0));
}

void saveCalibration(const char *fname)
{
    const auto t = std::time(nullptr);
    const auto tm = std::localtime(&t);
    char date_time[1024];
    std::strftime(date_time, 1024, "%c", tm);
    cv::FileStorage fs(fname, cv::FileStorage::WRITE);
    fs << "calibration_time" << date_time;
    fs << "serial_number" << kinect->getSerial();
    fs << "image_width" << frameSize.width;
    fs << "image_height" << frameSize.height;
    if (!irCameraMatrix.empty()) fs << "irCameraMatrix" << irCameraMatrix;
    if (!irDistCoeffs.empty()) fs << "irDistCoeffs" << irDistCoeffs;
    if (!rgbCameraMatrix.empty()) fs << "camera_matrix" << rgbCameraMatrix;
    if (!rgbDistCoeffs.empty()) fs << "distortion_coefficients" << rgbDistCoeffs;
    if (!R.empty()) fs << "R" << R;
    if (!T.empty()) fs << "T" << T;
    std::cout << "Saved calibration file " << fname << std::endl;
}

void loadCalibration(const char *fname)
{
    cv::FileStorage fs(fname, cv::FileStorage::READ);
    fs["irCameraMatrix"] >> irCameraMatrix;
    fs["irDistCoeffs"] >> irDistCoeffs;
    fs["camera_matrix"] >> rgbCameraMatrix;
    fs["distortion_coefficients"] >> rgbDistCoeffs;
    fs["R"] >> R;
    fs["T"] >> T;

    rvec.create(1, 3, CV_64F);
    if (R.empty() == false)
        Rodrigues(R, rvec);

    std::cout << "irCameraMatrix\n" << irCameraMatrix;
    std::cout << "\nrgbCameraMatrix\n" << rgbCameraMatrix;
    std::cout << "\nT\n" << T;
    std::cout << "\nrvec\n" << rvec << std::endl;
}

int main()
{
    kinect.reset(new SourceKinectOpenNI);
    kinect->startIr();
    std::cout << "Calibrating the device: " << kinect->getSerial() << std::endl;

    // start in IR mode
    CalibrationMode mode = CalibrationMode::Ir;
    initCaptureMode(false, false, true);

    int alpha = 2000;

    std::stringstream sstream;

    // create windows
    cv::namedWindow(frameNamePrimary);
    cv::createTrackbar("IR alpha", frameNamePrimary, &alpha, 10000);

    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> corners;

    std::vector<cv::Mat> rvecs; // unneeded
    std::vector<cv::Mat> tvecs;

    genPatternObjectPoints(patternSize, objectPoints);

    for (;;) {
        bool capture = false;
        const int key = cv::waitKey(1);
        if (key == 27)
            break;
        switch (key) {
        case 'i':
            if (mode != CalibrationMode::Ir) {
                mode = CalibrationMode::Ir;
                initCaptureMode(false, false, true);
            }
            break;
        case 'r':
            if (mode != CalibrationMode::Rgb) {
                mode = CalibrationMode::Rgb;
                initCaptureMode(true, false, false);
            }
            break;
        case 's':
            saveCalibration((kinect->getSerial() + ".yml").c_str());
            break;
        case 'l':
            loadCalibration((kinect->getSerial() + ".yml").c_str());
            break;
        case 'q':
            if (mode != CalibrationMode::Stereo) {
                mode = CalibrationMode::Stereo;
                initCaptureMode(true, false, false);
            }
            break;
        case 'c':
            if (mode != CalibrationMode::Check) {
                mode = CalibrationMode::Check;
                initCaptureMode(true, true, false);
            }
        case 32:
            capture = true;
            break;
        }

        // acquire sensor data
        kinect->grab();
        switch (mode) {
        case CalibrationMode::Ir:
            acquireIr(alpha / 1000.0f);
            break;
        case CalibrationMode::Stereo:
        case CalibrationMode::Rgb:
            acquireRgb();
            break;
        case CalibrationMode::Check:
            acquireDepth(255.0f / 4000.0f);
            acquireRgb();
            break;
        }

        // calibration
        switch (mode) {
        case CalibrationMode::Ir: {
            const bool cornersFound = cv::findCirclesGrid(irMat8, patternSize, corners, cv::CALIB_CB_ASYMMETRIC_GRID);
            cv::drawChessboardCorners(irMat8Rgb, patternSize, cv::Mat(corners), cornersFound);
            if (cornersFound && capture) {
                irObjPoints.push_back(objectPoints);
                irImgPoints.push_back(corners);
                const double error = cv::calibrateCamera(
                            irObjPoints, irImgPoints,
                            frameSize,
                            irCameraMatrix, irDistCoeffs,
                            rvecs, tvecs
                            );
                std::cout << "irCameraMatrix\n" << irCameraMatrix;
                std::cout << "\nirDistCoeffs\n" << irDistCoeffs << std::endl;
                if (!irCameraMatrix.empty()) {
                    cv::undistort(irMat8Rgb, irMat8RgbUndist, irCameraMatrix, irDistCoeffs);
                    textLine(irMat8RgbUndist, "undistorted using calibration", 1, textColorBlue);
                    if (irObjPoints.size() > 0) {
                        sstream.str(""); sstream << "number of views : " << irImgPoints.size();
                        textLine(irMat8RgbUndist, sstream.str(), 2, textColorBlue);

                        sstream.str(""); sstream << "reprojection error : " << error;
                        textLine(irMat8RgbUndist, sstream.str(), 3, textColorBlue);
                    }
                    cv::imshow(frameNameSecondary, irMat8RgbUndist);
                }
            }
            textLine(irMat8Rgb, "tracking", 3, textColorRed);
            textLine(rgbMat, cornersFound ? "press 'SPACE' to capture view" : "show me the calibration pattern", 2, textColorBlue);
            textLine(irMat8Rgb, "mode : IR", 1, textColorBlue);
            cv::imshow(frameNamePrimary, irMat8Rgb);

            break;
        }
        case CalibrationMode::Rgb: {
            const bool cornersFound = cv::findCirclesGrid(rgbMat, patternSize, corners, cv::CALIB_CB_ASYMMETRIC_GRID);
            cv::drawChessboardCorners(rgbMat, patternSize, cv::Mat(corners), cornersFound);
            if (cornersFound && capture) {
                rgbObjPoints.push_back(objectPoints);
                rgbImgPoints.push_back(corners);
                const double error = cv::calibrateCamera(
                            rgbObjPoints, rgbImgPoints,
                            frameSize,
                            rgbCameraMatrix, rgbDistCoeffs,
                            rvecs, tvecs
                            );
                std::cout << "rgbCameraMatrix\n" << rgbCameraMatrix;
                std::cout << "\nrgbDistCoeffs\n" << rgbDistCoeffs << std::endl;
                if (!rgbCameraMatrix.empty()) {
                    cv::undistort(rgbMat, rgbMatUndist, rgbCameraMatrix, rgbDistCoeffs);

                    textLine(rgbMatUndist, "undistorted using calibration", 1, textColorBlue);
                    if (rgbObjPoints.size() > 0) {
                        sstream.str(""); sstream << "number of views : " << rgbImgPoints.size();
                        textLine(rgbMatUndist, sstream.str(), 2, textColorBlue);

                        sstream.str(""); sstream << "reprojection error : " << error;
                        textLine(rgbMatUndist, sstream.str(), 3, textColorBlue);
                    }

                    cv::imshow(frameNameSecondary, rgbMatUndist);
                }
            }
            textLine(rgbMat, "tracking", 3, textColorRed);
            textLine(rgbMat, cornersFound ? "press 'SPACE' to capture view" : "show me the calibration pattern", 2, textColorBlue);
            textLine(rgbMat, "mode : RGB", 1, textColorBlue);
            cv::imshow(frameNamePrimary, rgbMat);

            break;
        }
        case CalibrationMode::Stereo: {
            const bool cornersFound = cv::findCirclesGrid(rgbMat, patternSize, corners, cv::CALIB_CB_ASYMMETRIC_GRID);
            cv::drawChessboardCorners(rgbMat, patternSize, cv::Mat(corners), cornersFound);
            if (cornersFound && capture) {
                // try to find points in rgb image as well
                initCaptureMode(false, false, true);
                for (int i = 0; i < 10; ++i)
                    kinect->grab();
                acquireIr(alpha / 1000.0f);

                std::vector<cv::Point2f> irCorners;
                const bool cornersFound = cv::findCirclesGrid(irMat8, patternSize, irCorners, cv::CALIB_CB_ASYMMETRIC_GRID);
                cv::drawChessboardCorners(irMat8Rgb, patternSize, cv::Mat(irCorners), cornersFound);

                if (cornersFound) { // we have found a set of correspondences!
                    stereoObjPoints.push_back(objectPoints);
                    stereoIrImgPoints.push_back(irCorners);
                    stereoRgbImgPoints.push_back(corners);

                    const double error = cv::stereoCalibrate(
                                stereoObjPoints,
                                stereoRgbImgPoints, stereoIrImgPoints,
                                rgbCameraMatrix, rgbDistCoeffs,
                                irCameraMatrix, irDistCoeffs,
                                frameSize, R, T,
                                E, F
                                );
                    cv::Rodrigues(R, rvec);

                    std::cout << "R\n" << R;
                    std::cout << "\nT\n" << T << std::endl;

                    if (stereoObjPoints.size() > 0) {
                        sstream.str(""); sstream << "number of views : " << stereoObjPoints.size();
                        textLine(irMat8Rgb, sstream.str(), 2, textColorBlue);

                        sstream.str(""); sstream << "reprojection error : " << error;
                        textLine(irMat8Rgb, sstream.str(), 3, textColorBlue);
                    }
                } else {
                    textLine(irMat8Rgb, "pattern not found, try again!", 4, textColorRed);
                }
                textLine(irMat8Rgb, "view from IR cam", 1, textColorBlue);

                cv::imshow(frameNamePrimary, irMat8Rgb);
                cv::waitKey(1); // force repaint because ...

                // ... this is going to take a lot of time
                initCaptureMode(true, false, false);
            }
            textLine(rgbMat, "tracking", 3, textColorRed);
            textLine(rgbMat, cornersFound ? "press 'SPACE' to capture view" : "show me the calibration pattern", 2, textColorBlue);
            textLine(rgbMat, "mode : Stereo", 1, textColorBlue);
            cv::imshow(frameNameSecondary, rgbMat);

            break;
        }
        case CalibrationMode::Check: {
            // uncalibrated overlay
            overlay(rgbMat, depthMat8, overlayMat);
            textLine(overlayMat, "mode : Check", 1, textColorWhite);
            textLine(overlayMat, "uncalibrated overlay", 2, textColorWhite);

            // calibrated overlay
            cv::Mat depthMat16Calib(frameSize, CV_16UC1);
            cv::Mat depthMat8Calib(frameSize, CV_8UC1);
            depthMat16Calib.setTo(0);

            mapDepthToRgb(depthMat16, depthMat16Calib, rvec, T, irCameraMatrix, irDistCoeffs, rgbCameraMatrix, rgbDistCoeffs);
            depthMat16Calib.convertTo(depthMat8Calib, CV_8UC1, 255.0f / 4000.0f);
            overlay(rgbMat, depthMat8Calib, overlayCalibMat);
            textLine(overlayCalibMat, "calibrated overlay", 2, textColorWhite);

            cv::imshow(frameNamePrimary, overlayMat);
            cv::imshow(frameNameSecondary, overlayCalibMat);
        }
            break;
        }
    }
}
