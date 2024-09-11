cmake_minimum_required(VERSION 2.8.3)
project(exploration_manager)

set(BUILD_DEPS ON)
set(USE_SCIP OFF)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)  # OR-Tools typically requires C++17 or newer
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(ortools_DIR "/opt/ortools/")
set(absl_DIR "/opt/ortools/lib/cmake/absl")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
#find_package(protobuf REQUIRED CONFIG)
#find_package(ortools CONFIG REQUIRED)

include(FetchContent)
FetchContent_Declare(
        or-tools
        GIT_REPOSITORY https://github.com/google/or-tools.git
        GIT_TAG        v9.0
)
set(BUILD_DEPS ON)
set(BUILD_DOC OFF)
set(BUILD_SAMPLES OFF)
set(BUILD_EXAMPLES OFF)
FetchContent_MakeAvailable(or-tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
  lkh_tsp_solver
  lkh_mtsp_solver
  plan_manage
  cv_bridge
  message_generation
)

add_message_files(
  FILES
  # SentGoals.msg
  # DeletedGoals.msg
  DroneState.msg
  PairOpt.msg
  PairOptResponse.msg
  HGrid.msg
  GridTour.msg
)

generate_messages(
  DEPENDENCIES
  # std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES exploration_manager
 CATKIN_DEPENDS plan_env path_searching bspline bspline_opt active_perception traj_utils lkh_tsp_solver lkh_mtsp_solver plan_manage message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  /opt/ortools/include
)


add_executable(exploration_node
  src/exploration_node.cpp 
  src/fast_exploration_fsm.cpp
  src/fast_exploration_manager.cpp
  )
target_link_libraries(exploration_node 
  ${catkin_LIBRARIES}
  -ldw
  ortools::ortools
  )

add_executable(ground_node
  src/ground_node.cpp 
  src/fast_exploration_manager.cpp
  )
target_link_libraries(ground_node
  ${catkin_LIBRARIES}
  -ldw
  ortools::ortools
  )

add_executable(test_vrp
  src/test_vrp.cpp 
  )
target_link_libraries(test_vrp
  ${catkin_LIBRARIES}
  ortools::ortools
  )
