add_executable(calibration calibration.cpp
Kinect.hpp Kinect.cpp)
target_link_libraries(calibration ${OpenCV_LIBS} ${OPENNI_LIBRARY} ${USB_LIBRARY})
