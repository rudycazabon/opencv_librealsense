cmake_minimum_required (VERSION 3.5 FATAL_ERROR)

project      (opencv_librealsense)
find_package (OpenCV REQUIRED)
find_package (OpenGL REQUIRED)
find_package (glfw3  REQUIRED)
find_package (X11)


include(CheckCXXCompilerFlag)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lopencv_core -lopencv_highgui -lopencv_objdetect -lopencv_imgproc -lopencv_features2d -lopencv_ml -lopencv_calib3d -lopencv_video -lglfw -lusb-1.0 -lGL -lGLU")

link_directories    (${OpenCV_LIBRARY_DIRS} $(OpenGL_LIBRARY_DIRS) $(GLFW_LIBRARY_DIRS) /usr/local/lib /usr/lib/x86_64-linux-gnu)
include_directories (${OpenCV_INCLUDE_DIRS} $(OpenGL_INCLUDE_DIRS) $(GLFW_INCLUDE_DIRS) ~/librealsense/include ~/librealsense/examples ~/librealsense/examples/third_party/glfw/include )
add_definitions     (${OpenCV_DEFINITIONS})


set  (project_SOURCES opencv_librealsense.cpp)
set  (project_HEADERS )
set  (project_FORMS   )


ADD_EXECUTABLE  (opencv_librealsense ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (opencv_librealsense ${OpenCV_LIBS} /usr/local/lib/librealsense.so /usr/lib/x86_64-linux-gnu/libglfw.so.3.1 libGL.so libGLU.so)