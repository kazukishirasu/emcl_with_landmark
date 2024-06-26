cmake_minimum_required(VERSION 3.0.2)
project(emcl)

set(CMAKE_CXX_FLAGS "-std=c++17 -O3 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geometry_msgs
  tf2
  tf2_ros
  yolov5_pytorch_ros
  std_msgs
  message_generation
  laser_geometry
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mcl
# CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} )

add_library(Mcl src/Mcl.cpp)
add_library(ExpResetMcl src/ExpResetMcl.cpp)
add_library(ExpResetMcl2 src/ExpResetMcl2.cpp)
add_library(Particle src/Particle.cpp)
add_library(OdomModel src/OdomModel.cpp)
add_library(Pose src/Pose.cpp)
add_library(Scan src/Scan.cpp)
add_library(LikelihoodFieldMap src/LikelihoodFieldMap.cpp)
add_library(x_means src/x_means.cpp)
add_library(gmm src/gmm.cpp)

add_executable(mcl_node src/mcl_node.cpp)
add_executable(emcl_node src/emcl_node.cpp)
add_executable(emcl2_node src/emcl2_node.cpp)
add_executable(register_landmark_node src/register_landmark_node.cpp)
add_executable(visualize_landmark_node src/visualize_landmark_node.cpp)

add_dependencies(mcl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(emcl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(emcl2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mcl_node Mcl Particle Pose Scan OdomModel LikelihoodFieldMap ${catkin_LIBRARIES})
target_link_libraries(emcl_node Mcl ExpResetMcl Particle Pose Scan OdomModel LikelihoodFieldMap ${catkin_LIBRARIES})
target_link_libraries(emcl2_node Mcl ExpResetMcl2 Particle Pose Scan OdomModel LikelihoodFieldMap ${catkin_LIBRARIES})
target_link_libraries(mcl_node Mcl Particle Pose Scan OdomModel LikelihoodFieldMap yaml-cpp ${catkin_LIBRARIES})
target_link_libraries(emcl_node Mcl ExpResetMcl Particle Pose Scan OdomModel LikelihoodFieldMap yaml-cpp ${catkin_LIBRARIES})
target_link_libraries(emcl2_node Mcl ExpResetMcl2 Particle Pose Scan OdomModel LikelihoodFieldMap yaml-cpp ${catkin_LIBRARIES})
# target_link_libraries(register_landmark_node yaml-cpp x_means ${catkin_LIBRARIES})
target_link_libraries(register_landmark_node yaml-cpp gmm ${catkin_LIBRARIES})
target_link_libraries(visualize_landmark_node yaml-cpp ${catkin_LIBRARIES})