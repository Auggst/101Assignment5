cmake_minimum_required (VERSION 3.10)
project (GraphicsHome  VERSION 2.0)

#specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(OpenCV REQUIRED)

#include Dir
include_directories(/home/wzs/codes/RealTimeRendering/include)


#add the executable
add_executable (GraphicsHome main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp)
target_link_libraries(GraphicsHome ${OpenCV_LIBRARIES})