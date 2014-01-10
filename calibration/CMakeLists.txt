add_executable(calibration calibration.cpp
../acquisition/SourceKinectOpenNI.hpp ../acquisition/SourceKinectOpenNI.cpp)
target_link_libraries(calibration ${OpenCV_LIBS} ${OPENNI_LIBRARY} ${USB_LIBRARY})

find_path(FREENECT_INCLUDE_DIR libfreenect/libfreenect.h)
include_directories(${FREENECT_INCLUDE_DIR}/libfreenect)
find_library(FREENECT_LIBRARY freenect)
add_executable(freenect_calibrate freenect_calibrate.cpp)
target_link_libraries(freenect_calibrate ${FREENECT_LIBRARY} ${OpenCV_LIBS})

add_executable(grab_ir grab_ir.cpp
../acquisition/SourceFreenect.hpp ../acquisition/SourceFreenect.cpp)
target_link_libraries(grab_ir ${FREENECT_LIBRARY} ${OpenCV_LIBS})

add_executable(stereo_calibration stereo_calibration.cpp)
target_link_libraries(stereo_calibration ${OpenCV_LIBS})
