cmake_minimum_required(VERSION 2.8.3)
project(pi3_crazyflie_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   rpyt.msg
   quadrotor_state.msg
   full_quadrotor_state.msg
   angles.msg
   acc.msg
   motor_msg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  /usr/local/cuda-5.5/targets/x86_64-linux/include include ${catkin_INCLUDE_DIRS}
)

add_library(lwpr STATIC
  src/lwpr/lwpr.c 
  src/lwpr/lwpr_aux.c 
  src/lwpr/lwpr_binio.c 
  src/lwpr/lwpr_math.c 
  src/lwpr/lwpr_mem.c 
  src/lwpr/lwpr_xml.c
)
target_link_libraries(lwpr ${catkin_LIBRARIES})

add_executable(quadrotor_simulator src/quadrotor_simulator.cpp)
target_link_libraries(quadrotor_simulator ${catkin_LIBRARIES})

## CUDA Compilation

find_package(CUDA REQUIRED)
set (CUDA_NVCC_FLAGS "-arch=sm_30" CACHE STRING "nvcc flags" FORCE)

#SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)

cuda_add_executable(pi3_main_quad src/pi3_main_quad.cu)
target_link_libraries(pi3_main_quad lwpr /usr/local/cuda/targets/x86_64-linux/lib/libcurand.so ${catkin_LIBRARIES})
