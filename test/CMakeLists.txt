cmake_minimum_required(VERSION 2.8)

project(testLaneModel)

find_package( OpenCV REQUIRED )

add_executable(testBoxProperty testBoxProperty.cpp ../src/ObjectShape.cpp ../src/Utility.cpp) 

target_link_libraries( testBoxProperty ${OpenCV_LIBS} )

add_test(box_property testBoxProperty)


