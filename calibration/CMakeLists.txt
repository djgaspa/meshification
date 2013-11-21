add_executable(calibration calibration.cpp
Kinect.hpp Kinect.cpp)
target_link_libraries(calibration ${OpenCV_LIBS} OpenNI usb-1.0)
