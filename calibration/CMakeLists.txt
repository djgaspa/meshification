add_executable(calibration calibration.cpp
../acquisition/SourceKinectOpenNI.hpp ../acquisition/SourceKinectOpenNI.cpp)
target_link_libraries(calibration ${OpenCV_LIBS} ${OPENNI_LIBRARY} ${USB_LIBRARY})
