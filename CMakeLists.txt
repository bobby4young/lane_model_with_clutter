# enable C++11 standard
set(CMAKE_CXX_STANDARD 11)  

cmake_minimum_required(VERSION 2.8.9)
project(lane_model)

#Bring the headers into the project
include_directories(include)
include_directories(src/KernelDensityEstimation)

#the file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "src/*.cpp" "src/KernelDensityEstimation/src/*.hpp")
#file(GLOB KDE_SOURCES "src/KernelDensityEstimation/src/*.hpp")
#file(GLOB SOURCES "src/KernelDensityEstimation/KernelDensityEstimation")

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package( OpenCV REQUIRED )
FIND_PACKAGE( Eigen3 REQUIRED )
add_executable(lane_model ${SOURCES})
include_directories(${EIGEN3_INCLUDE_DIR})

target_link_libraries( lane_model ${OpenCV_LIBS} Threads::Threads)

ENABLE_TESTING()

add_subdirectory(test)
