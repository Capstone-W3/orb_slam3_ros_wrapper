cmake_minimum_required(VERSION 2.8)
project(orb_slam3_ros_wrapper)

# Change this to your installation of ORB-SLAM3. Default is ~/
set(ORB_SLAM3_DIR
   $ENV{HOME}/trash_parent_repo/ORB_SLAM3
)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package (catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   cv_bridge
   image_transport
   tf
   sensor_msgs
   message_generation
)

find_package(OpenCV REQUIRED)
MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3)
MESSAGE("Eigen 3 VERSION:")
MESSAGE(${Eigen3_VERSION})

find_package(Pangolin REQUIRED)

# adding for point cloud viewer and mapper
find_package( PCL 1.7 REQUIRED )
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  sensor_msgs
  message_generation
  tf
  image_transport
  cv_bridge
  )

catkin_package (
   CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs message_runtime
   LIBRARIES {PROJECT_NAME}
)

include_directories(
   ${ORB_SLAM3_DIR}
   ${ORB_SLAM3_DIR}/include
   ${ORB_SLAM3_DIR}/include/CameraModels
   ${PROJECT_SOURCE_DIR}/include
   ${EIGEN3_INCLUDE_DIR}
   ${catkin_INCLUDE_DIRS}
   ${Pangolin_INCLUDE_DIRS}
   ${PCL_INCLUDE_DIRS}
   ${catkin_INCLUDE_DIRS}
)

# Build ROS nodes
set(LIBS
   ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
   ${EIGEN3_LIBS}
   ${catkin_LIBRARIES}
   ${Pangolin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
)

# grid mapping mono pub node
add_executable (${PROJECT_NAME}_mono_pub_gridmap
   src/ros_mono_pub.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_mono_pub_gridmap
   ${LIBS}
)

# grid mapping mono sub node
add_executable (${PROJECT_NAME}_mono_sub_gridmap
   src/ros_mono_sub.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_mono_sub_gridmap
   ${LIBS}
)

# mono node
add_executable (${PROJECT_NAME}_mono
   src/mono_node.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_mono
   ${LIBS}
)

# mono-inertial node
add_executable (${PROJECT_NAME}_mono_inertial
   src/mono_inertial_node.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_mono_inertial
   ${LIBS}
)

# stereo node
add_executable (${PROJECT_NAME}_stereo
   src/stereo_node.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_stereo
   ${LIBS}
)

# stereo-inertial node
add_executable (${PROJECT_NAME}_stereo_inertial
   src/stereo_inertial_node.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_stereo_inertial
   ${LIBS}
)

# RGB-D node
add_executable (${PROJECT_NAME}_rgbd
   src/rgbd_node.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_rgbd
   ${LIBS}
)
