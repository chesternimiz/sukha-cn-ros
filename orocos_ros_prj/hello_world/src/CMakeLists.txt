
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libhello_world-<target>.so
# and installs in the directory lib/orocos/hello_world/
#
orocos_component(hello_world hello_world-component.hpp hello_world-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/hello_world/ directory
#
orocos_install_headers(hello_world-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libhello_world-plugin-<target>.so
# and installs in the directory lib/orocos/hello_world/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(hello_world-plugin hello_world-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libhello_world-service-<target>.so
# and installs in the directory lib/orocos/hello_world/plugins/
#
orocos_service(hello_world-service hello_world-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libhello_world-support-<target>.so and installs it in
# lib/
#
orocos_library(hello_world-support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


