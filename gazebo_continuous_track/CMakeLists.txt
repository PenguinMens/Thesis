cmake_minimum_required(VERSION 3.5)
project(gazebo_continuous_track)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${gazebo_dev_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${ament_index_cpp_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${gazebo_dev_LIBRARIES} ${rclcpp_LIBRARIES} ${ament_index_cpp_LIBRARIES})
set(LIBS ${ament_cmake_LIBRARIES} ${gazebo_dev_LIBRARIES} ${rclcpp_LIBRARIES} ${ament_index_cpp_LIBRARIES})

# Include directory of ODE (gazebo/ode/ode.h, etc)
set(gazebo_ode_dev_INCLUDE_DIRS "")
foreach(gazebo_dev_INCLUDE_DIR ${gazebo_dev_INCLUDE_DIRS})
  list(APPEND gazebo_ode_dev_INCLUDE_DIRS ${gazebo_dev_INCLUDE_DIR}/gazebo)
endforeach()

## Declare a C++ library
add_library(ContinuousTrack src/gazebo_continuous_track.cpp)
add_library(ContinuousTrackSimple src/gazebo_continuous_track_simple.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ContinuousTrack ${LIBS})
target_link_libraries(ContinuousTrackSimple ${LIBS})

# Installation
install(TARGETS ContinuousTrack ContinuousTrackSimple
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_dependencies(ament_cmake gazebo_dev rclcpp ament_index_cpp)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(ContinuousTrack ContinuousTrackSimple ${LIBS})

ament_package()
