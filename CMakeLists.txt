cmake_minimum_required(VERSION 3.5)
project(kml_parser)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(LibKML REQUIRED)
find_package(GeographicLib REQUIRED)

set(INCLUDE_DIRS /usr/local/include/)
set(LIBRARY_DIRS /usr/lib/x86_64-linux-gnu/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
                    ${INCLUDE_DIRS})
link_directories(${LIBRARY_DIRS})

add_executable(kml_parser src/kml_parser.cpp)
target_link_libraries(kml_parser ${LIBRARY_DIRS}/libkmlbase.so ${LIBRARY_DIRS}/libkmldom.so ${LIBRARY_DIRS}/libkmlengine.so ${GeographicLib_LIBRARIES})
ament_target_dependencies(kml_parser rclcpp std_msgs nav_msgs)
install(TARGETS kml_parser DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
