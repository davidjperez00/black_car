cmake_minimum_required(VERSION 3.0.2)
project(black_car)
# set(OpenCV_DIR /usr/share/OpenCV)
# set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.3)
# set(CUDNN_INCLUDE_DIR /usr/local/cuda/include)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_runtime geometry_msgs sensor_msgs image_transport message_generation)
add_message_files(
	DIRECTORY
	msg
	FILES
	CarObject.msg
  	CarObjects.msg
	TTC_Data.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
	CATKIN_DEPENDS roscpp rospy message_runtime
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
scripts/safety_node.py
scripts/tf_broadcaster_node.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
	find_package(roslaunch REQUIRED)
	roslaunch_add_file_check(launch)
endif()


# set(Torch_DIR /home/dperez/.local/lib/python3.6/site-packages/torch/share/cmake/Torch)
# find_package(Torch REQUIRED)
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
#   ${Torch_INCLUDE_DIRS}
)
# add_executable(real_car_lane_follow src/fastLaneFollow.cpp)
# add_dependencies(real_car_lane_follow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(real_car_lane_follow ${catkin_LIBRARIES} ) 