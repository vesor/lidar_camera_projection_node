cmake_minimum_required(VERSION 2.8)
project(lidar_camera_proj)

add_definitions(
        -std=c++11
        -std=c++0x
        -DPCL_NO_PRECOMPILE
        -DELPP_THREAD_SAFE
)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
#enable for camera frame
# cv_bridge
)

set(OpenCV_INCLUDE_DIR /usr/local/opencv330/include)
set(OpenCV_LIB_DIR /usr/local/opencv330/lib)
set(OpenCV_LIBS opencv_core opencv_video opencv_imgproc opencv_highgui opencv_imgcodecs opencv_calib3d)

if(FALSE) #opencv2
set(OpenCV_INCLUDE_DIR /usr/local/opencv2413/include)
set(OpenCV_LIB_DIR /usr/local/opencv2413/lib)
set(OpenCV_LIBS opencv_calib3d
  opencv_contrib
  opencv_core
  opencv_features2d
  opencv_flann
  opencv_gpu
  opencv_highgui
  opencv_imgproc
  opencv_legacy
  opencv_ml
  opencv_nonfree
  opencv_objdetect
  opencv_ocl
  opencv_photo
  opencv_stitching
  opencv_superres
  opencv_video
  opencv_videostab
  opencv_viz)
endif()


set(PCL_INCLUDE_DIR /usr/local/include/pcl-1.8)
set(PCL_LIB_DIR /usr/local/lib)
set(PCL_LIBS pcl_common pcl_kdtree pcl_sample_consensus pcl_io pcl_features)


set(CMAKE_BUILD_TYPE DEBUG)

set(INCLUDE_DIRS

)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp roslib std_msgs sensor_msgs cv_bridge
)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/build_external/
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIR}
  "/usr/include/eigen3"
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/build_external
  ${PCL_LIB_DIR}
  ${OpenCV_LIB_DIR}
)

file(GLOB SRC_CODE
    "src/*.cpp"
)

add_executable(${PROJECT_NAME} ${SRC_CODE})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS})

