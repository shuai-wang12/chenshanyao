cmake_minimum_required(VERSION 3.0.2)
project(hybrid_a)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(Boost 1.58 REQUIRED )

find_package(casadi REQUIRED)
# Eigen
include(cmake/FindEigen.cmake)
include_directories(${EIGEN_INCLUDE_DIR})

# glog
include(cmake/glog.cmake)
include_directories(${GLOG_INCLUDE_DIRS})

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  nav_msgs
)

find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES hybrid_a
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


# set the Ipopt library and include directory
set(LIBRARY_DIRS /usr/local/lib)
if(UNIX AND NOT APPLE)
    set(IPOPT_INCLUDE_DIRS /usr/include/coin)
elseif(APPLE)
    set(IPOPT_INCLUDE_DIRS /usr/local/include/coin-or)
endif()

# set casadi include directory
set(CASADI_INCLUDE_DIR /usr/local/include/casadi)

# find casadi library
find_library(CASADI_LIBRARY
    NAMES casadi
    HINTS ${CASADI_INCLUDE_DIR}/../lib $ENV{CASADI_PREFIX}/lib)
if(CASADI_LIBRARY)
    set(CASADI_LIBRARIES ${CASADI_LIBRARIES} ${CASADI_LIBRARY})
endif()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${LIBRARY_DIRS})

## Declare a C++ library
add_library(
  PATH_SEARCHER_LIB SHARED
  src/planner.cpp
  src/publish_test.cpp
  src/environments.cpp
  src/cbs/cl_cbs.cpp
  src/hybrid_a/hybrid_a.cpp
  src/hybrid_a/middlePark.cpp
  src/hybrid_a/rs_path.cpp
  src/hybrid_a/dubins.cpp
  src/voronoi/bucketedqueue.cpp
  src/voronoi/dynamicvoronoi.cpp
  src/voronoi/voronoiedge.cpp
  src/traj/discretized_traj.cpp
  src/traj/traj_nlp.cpp
  src/traj/traj_optimizer.cpp
  src/traj/traj_planner.cpp
  src/visual_rviz/color.cpp
  src/visual_rviz/plot.cpp
  src/single_planner.cpp
  src/multi_opti.cpp
  src/clipper2/clipper.engine.cpp
  src/clipper2/clipper.offset.cpp
  src/clipper2/clipper.rectclip.cpp
)

add_executable(${PROJECT_NAME}_node main.cpp)

target_link_libraries(${PROJECT_NAME}_node
  PATH_SEARCHER_LIB
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARIES}
  yaml-cpp
  casadi
)
