cmake_minimum_required(VERSION 3.0.2)
project(estimator)

set(CMAKE_CXX_STANDARD 14)
# add_compile_options(-std=c++11)

include(./FindGUROBI.cmake)
include_directories(
  ${GUROBI_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  geometry_msgs
  message_generation
)
find_package(OpenCV)

# 寻找PCL库
find_package(PCL 1.10 REQUIRED COMPONENTS common io)
# PCL头文件路径
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# MKL 头文件路径
set(MKL_DIR /opt/intel/oneapi/mkl/latest/lib/cmake/mkl)
# find_package(MKL REQUIRED)
include_directories(/opt/intel/oneapi/mkl/latest/include)
link_directories(/opt/intel/oneapi/mkl/latest/lib)
# link_libraries(mkl_intel_lp64 mkl_sequential mkl_core)


add_message_files(
FILES
AprilTagDetection.msg
AprilTagDetectionArray.msg
result.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ./src/libs
  ./include
  ${PROJECT_SOURCE_DIR}/include
  # ${PCL_INCLUDE_DIRS}
  # ${MKL_ROOT_DIR}/include
)

catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_generation message_runtime

  # DEPENDS system_lib
)

ADD_EXECUTABLE(estimator src/estimator.cpp)
TARGET_LINK_LIBRARIES(estimator 
                       ${catkin_LIBRARIES}  
                       ${GUROBI_LIBRARIES} 
                       ${PCL_LIBRARIES} 
                       libmkl_rt.so
                      )

ADD_EXECUTABLE(ideal_trace_pub src/ideal_trace_pub.cpp)
TARGET_LINK_LIBRARIES(ideal_trace_pub ${catkin_LIBRARIES})

# ADD_EXECUTABLE(mip src/mip.cpp)
# TARGET_LINK_LIBRARIES(mip ${catkin_LIBRARIES} ${GUROBI_LIBRARIES})

ADD_EXECUTABLE(translate_xsens_msg src/translate_xsens_msg.cpp)
TARGET_LINK_LIBRARIES(translate_xsens_msg ${catkin_LIBRARIES})

ADD_DEPENDENCIES(estimator ${${PROJECT_NAME}_EXPORTED_TARGETS})
