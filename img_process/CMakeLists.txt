cmake_minimum_required(VERSION 2.8.3)
project(img_process)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp std_msgs
  std_srvs
  sensor_msgs
  cv_bridge
  camera_info_manager
  geometry_msgs
  message_generation
)

# # pkg-config libraries
find_package(PkgConfig REQUIRED)
find_package(OpenCV)

# MKL 头文件路径
include_directories(/opt/intel/oneapi/mkl/latest/include)
link_directories(/opt/intel/oneapi/mkl/latest/lib)

add_message_files(
FILES
AprilTagDetection.msg
AprilTagDetectionArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  std_msgs
)

# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_generation message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/src
)

# # Declare a cpp executable
add_executable(receive_img src/receive_img_1.cpp)
target_link_libraries(receive_img ${catkin_LIBRARIES} )

add_executable(find_circle src/find_circle.cpp)
target_link_libraries(find_circle ${catkin_LIBRARIES} ${OpenCV_LIBS})

# add_executable(receive_img_test_time_delay src/receive_img_test_time_delay.cpp)
# target_link_libraries(receive_img_test_time_delay ${catkin_LIBRARIES} ${OpenCV_LIBS})