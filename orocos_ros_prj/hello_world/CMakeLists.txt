#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(hello_world)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  )


# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# Rest of the configuration is in src/
add_subdirectory( src )
#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library libhello_world-types-<target>.so
# and installs in the directory lib/orocos/@target@/hello_world/types/
#
# The header will go in include/orocos/hello_world/types/hello_world/hello_world-types.hpp
# So you can #include <hello_world/hello_world-types.hpp>
#
#orocos_typegen_headers(include/hello_world/hello_world-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement in your toplevel CMakeFile.txt !



#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros
  DEPENDS rtt_std_msgs
)
