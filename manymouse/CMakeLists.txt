cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

## Set Paths
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(MANYMOUSE_SOURCES src/linux_evdev.c src/macosx_hidmanager.c src/macosx_hidutilities.c src/manymouse.c src/windows_wminput.c src/x11_xinput2.c)
set(MANYMOUSE_HEADERS include/manymouse.h)

rosbuild_add_library(${PROJECT_NAME}	${MANYMOUSE_HEADERS}
										${MANYMOUSE_SOURCES}
										${MANYMOUSE_HEADERS_MOC})

