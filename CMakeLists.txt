cmake_minimum_required(VERSION 3.5)

project(RMvision)

SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Threads)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(camera_device/include
                    Armor/
                    extools/
                    Serial/
                    wthread/)

add_executable(RMvision main.cpp
               Armor/armorDetector.cpp
               Armor/armorClassifier.cpp
               Armor/spinDetector.cpp
               Serial/serialport.cpp
               camera_device/src/mindcamera.cpp
               extools/solver.cpp
               extools/tools.cpp
               extools/predictor.cpp
               wthread/wthread.cpp)

link_directories(/usr/lib)

target_link_libraries(RMvision ${OpenCV_LIBS})
target_link_libraries(RMvision MVSDK)
target_link_libraries(RMvision pthread)

