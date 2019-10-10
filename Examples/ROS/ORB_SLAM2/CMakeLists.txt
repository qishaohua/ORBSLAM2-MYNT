cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
FIND_PACKAGE( octomap REQUIRED )# octomap
find_package(Boost COMPONENTS system)# boost c++库
find_package( PCL 1.7 REQUIRED )# 点云显示
find_package(mynteyed)#小觅相机库

#显示小觅相机库是否存在
if(mynteyed_FOUND)
   message(STATUS "MYNT-EYE-D-SDK found.")
else()
   message(STATUS "MYNT-EYE-D-SDK not found.")
endif()


include_directories(
${PROJECT_SOURCE_DIR}#ROS工程路径
${PROJECT_SOURCE_DIR}/../../../#ORB-SLAM2工程路径
${PROJECT_SOURCE_DIR}/../../../include#ORB-SLAM2工程头文件路径
#eigen库只有头文件，因此不需要包含源文件
${Pangolin_INCLUDE_DIRS}#Pangolin库
${PCL_INCLUDE_DIRS}#点云库
${Boost_INCLUDE_DIRS}#Boost库
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${Boost_LIBRARIES}
${PCL_LIBRARIES}
${OCTOMAP_LIBRARIES} # octomap
${NCNN_TARGET_LIB}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2_pc.so
)



# RGB-D 相机节点
rosbuild_add_executable(RGBD
src/ros_mynteye_d_rgbd.cc
)

target_link_libraries(RGBD
${LIBS}
)

