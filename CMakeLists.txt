cmake_minimum_required(VERSION 3.5)
project(mocap_nokov)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Require c++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (nokov_sdk_INCLUDE_DIRS deps/nokov_sdk/include)
set (nokov_sdk_LIB_DIRS deps/nokov_sdk/lib)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  geometry_msgs
  nav_msgs
  dynamic_reconfigure
  tf
)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if (${ARCHITECTURE} MATCHES "armhf")
  set(nokov_sdk_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/${nokov_sdk_LIB_DIRS}/armhf/libSeekerSDKClient_armhf.so)    
elseif (${ARCHITECTURE} MATCHES "aarch")
  set(nokov_sdk_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/${nokov_sdk_LIB_DIRS}/aarch64/libSeekerSDKClient_aarch.so)  
else()
  set(nokov_sdk_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/${nokov_sdk_LIB_DIRS}/x64/libSeekerSDKClient.so)
endif()

message(STATUS ${nokov_sdk_LIBRARIES})

#find_library(nokov_sdk_LIBRARIES SeekerSDKClient ${nokov_sdk_LIB_DIRS} NO_CACHE)
#set(nokov_sdk_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/${nokov_sdk_LIB_DIRS}/libSeekerSDKClient.so)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/MocapNokov.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  dynamic_reconfigure
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${nokov_sdk_INCLUDE_DIRS}
)

add_subdirectory(src)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
