# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_zinger_viz_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED zinger_viz_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(zinger_viz_FOUND FALSE)
  elseif(NOT zinger_viz_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(zinger_viz_FOUND FALSE)
  endif()
  return()
endif()
set(_zinger_viz_CONFIG_INCLUDED TRUE)

# output package information
if(NOT zinger_viz_FIND_QUIETLY)
  message(STATUS "Found zinger_viz: 0.1.0 (${zinger_viz_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'zinger_viz' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${zinger_viz_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(zinger_viz_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${zinger_viz_DIR}/${_extra}")
endforeach()
