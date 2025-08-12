cmake_minimum_required(VERSION 3.0.2)
project(tf_writer)

set(CMAKE_CXX_STANDARD 14)
# add_compile_options(-std=c++11)


include_directories(
  # ${OpenCV_INCLUDE_DIRS}
  )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  # std_msgs
  tf2
  tf2_ros
  # geometry_msgs
  message_generation
)

# add_message_files(
# FILES
# AprilTagDetection.msg
# AprilTagDetectionArray.msg
# result.msg
# )

generate_messages(
  DEPENDENCIES
  # std_msgs
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ./include
  ${PROJECT_SOURCE_DIR}/include
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy  geometry_msgs message_generation message_runtime
# std_msgs
  # DEPENDS system_lib
)

# ADD_EXECUTABLE(imu_writer src/imu_writer.cpp)
# TARGET_LINK_LIBRARIES(imu_writer ${catkin_LIBRARIES}  ${GUROBI_LIBRARIES} ${OpenCV_LIBS})

# ADD_DEPENDENCIES(imu_writer ${${PROJECT_NAME}_EXPORTED_TARGETS})


# ADD_EXECUTABLE(tf_writer src/tf_writer.cpp)
# TARGET_LINK_LIBRARIES(tf_writer ${catkin_LIBRARIES}  ${GUROBI_LIBRARIES} ${OpenCV_LIBS})

# ADD_DEPENDENCIES(tf_writer ${${PROJECT_NAME}_EXPORTED_TARGETS})