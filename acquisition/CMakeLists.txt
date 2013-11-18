find_package(Boost COMPONENTS program_options REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Qt4 COMPONENTS QtCore QtGui QtXml QtOpenGl)
find_library(QGLVIEWER_LIBRARY NAMES qglviewer-qt4 QGLViewer QGLViewer2)
find_library(GLEW_LIBRARY NAMES GLEW glew32)

include(${QT_USE_FILE})

set_source_files_properties(triangle.cpp PROPERTIES COMPILE_DEFINITIONS "TRILIBRARY;EXTERNAL_TEST;ANSI_DECLARATORS")
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
QT4_WRAP_UI(UI_SRC AcquisitionUI.ui)
QT4_WRAP_CPP(CPP_SRC QtAcquisition.hpp AcquisitionUI.hpp Viewer.hpp)
QT4_ADD_RESOURCES(RC_SRC shaders.qrc)
add_executable(acquisition ${CPP_SRC} ${UI_SRC} ${RC_SRC} main.cpp
    DepthMeshifier.hpp DepthMeshifier.cpp
    Triangulator.hpp Triangulator.cpp
    triangle.h triangle.cpp
    SurfaceReconstruction.hpp SurfaceReconstruction.cpp
    MeshBuilder.hpp MeshBuilder.cpp
    Consumer.hpp Consumer.cpp
    VideoEncoder.hpp VideoEncoder.cpp
    ../common/AsyncWorker.hpp ../common/AsyncWorker.cpp
    SourceRaw.hpp SourceRaw.cpp
    SourceKinect.hpp SourceKinect.cpp
    SourceKinectOpenNI.hpp SourceKinectOpenNI.cpp
    SourceOni.hpp SourceOni.cpp
    DepthFilter.hpp DepthFilter.cpp
    QtAcquisition.hpp QtAcquisition.cpp
    AcquisitionUI.hpp AcquisitionUI.cpp
    Viewer.hpp Viewer.cpp
)
target_link_libraries(acquisition ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OPENNI_LIBRARY} 3dzip ${VPX_LIBRARY} ${RAKNET_LIBRARY} ${ARUCO_LIBRARY} ${Boost_LIBRARIES}
    ${QT_LIBRARIES} ${QGLVIEWER_LIBRARY} ${GLEW_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
if (WIN32)
        target_link_libraries(acquisition ws2_32)
endif()

install(TARGETS acquisition RUNTIME DESTINATION .)
