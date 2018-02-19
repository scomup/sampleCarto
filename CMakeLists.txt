cmake_minimum_required(VERSION 2.8.12)  # Ships with Ubuntu 14.04 (Trusty)

project(sample_carto)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall   -O3 -march=native")

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall   -O3 -march=native")


find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Ceres REQUIRED COMPONENTS SparseLinearAlgebraLibrary)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)


IF(OPENCV_DEBUG)
 find_package(OpenCV REQUIRED)
 add_definitions(-DOPENCV_DEBUG)
 include_directories(${OpenCV_INCLUDE_DIRS})
 message(WARNING "use opencv debug")
 ELSE()
 message(WARNING "do not use opencv debug")
ENDIF()

#find_package(Lua REQUIRED)
set(LUA_INCLUDE_DIR /usr/include/lua5.2)
set(LUA_LIBRARIES /usr/lib/x86_64-linux-gnu/liblua5.2.so /usr/lib/x86_64-linux-gnu/libm.so)

add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs  message_filters laser_geometry  rosbag)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_BINARY_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${LUA_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB_RECURSE ALL_SRCS "*.cc")


add_executable(${PROJECT_NAME}  ${ALL_SRCS})

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME}
${LUA_LIBRARIES}
${catkin_LIBRARIES}
${Boost_LIBRARIES}
${CERES_LIBRARIES}
${OpenCV_LIBRARIES}
dl
)

