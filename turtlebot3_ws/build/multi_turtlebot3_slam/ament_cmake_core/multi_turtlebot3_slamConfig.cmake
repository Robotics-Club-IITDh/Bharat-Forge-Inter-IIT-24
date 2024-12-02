# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_multi_turtlebot3_slam_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED multi_turtlebot3_slam_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(multi_turtlebot3_slam_FOUND FALSE)
  elseif(NOT multi_turtlebot3_slam_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(multi_turtlebot3_slam_FOUND FALSE)
  endif()
  return()
endif()
set(_multi_turtlebot3_slam_CONFIG_INCLUDED TRUE)

# output package information
if(NOT multi_turtlebot3_slam_FIND_QUIETLY)
  message(STATUS "Found multi_turtlebot3_slam: 0.0.0 (${multi_turtlebot3_slam_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'multi_turtlebot3_slam' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${multi_turtlebot3_slam_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(multi_turtlebot3_slam_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${multi_turtlebot3_slam_DIR}/${_extra}")
endforeach()
