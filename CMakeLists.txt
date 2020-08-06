cmake_minimum_required(VERSION 2.8.3)
project(pcl_fusion)
SET(CMAKE_BUILD_TYPE RelWithDebInfo)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories("include")

# Library
cs_add_library(pcl_fusion_lib
  src/fused_pointcloud.cpp
)

target_link_libraries(pcl_fusion_lib
  ${CATKIN_LIBRARIES}
)

# Executable
cs_add_executable(pcl_fusion_node
  src/pcl_fusion_node.cpp
)

target_link_libraries(pcl_fusion_node
  pcl_fusion_lib
)

cs_install()
cs_export()
  
