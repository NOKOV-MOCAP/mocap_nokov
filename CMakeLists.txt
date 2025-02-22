cmake_minimum_required(VERSION 3.5)
project(mocap_nokov)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (nokov_sdk_INCLUDE_DIRS deps/nokov_sdk/include)
set (nokov_sdk_LIB_DIRS deps/nokov_sdk/lib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if (${ARCHITECTURE} MATCHES "arm")
  set(nokov_sdk_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/${nokov_sdk_LIB_DIRS}/armhf/libnokov_sdk.so)    
elseif (${ARCHITECTURE} MATCHES "aarch")
  set(nokov_sdk_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/${nokov_sdk_LIB_DIRS}/aarch64/libnokov_sdk.so)  
else()
  set(nokov_sdk_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/${nokov_sdk_LIB_DIRS}/x64/libnokov_sdk.so)
endif()

message(STATUS ${nokov_sdk_LIBRARIES})

include_directories(include
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nokov_sdk_INCLUDE_DIRS}
)

add_subdirectory(src)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

# add_executable(mocap_node src/mocap_node.cpp)
# target_include_directories(mocap_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# install(TARGETS mocap_node
#   DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

if (${ARCHITECTURE} MATCHES "arm") 
  install(DIRECTORY launch config deps/nokov_sdk/include deps/nokov_sdk/lib/armhf
    DESTINATION share/${PROJECT_NAME})
elseif (${ARCHITECTURE} MATCHES "aarch")
  install(DIRECTORY launch config deps/nokov_sdk/include deps/nokov_sdk/lib/aarch64
    DESTINATION share/${PROJECT_NAME})
else()
  install(DIRECTORY launch config deps/nokov_sdk/include deps/nokov_sdk/lib/x64
    DESTINATION share/${PROJECT_NAME})
endif()

ament_package()

