cmake_minimum_required(VERSION 3.16.0)
set(ENV{IDF_COMPONENT_MANAGER} "1")

# BTPDevice (the communication: links, BTP, terminal, key, telemetry) and its
# two dependencies, BTP and TinyShell, come from sibling checkouts --
# c:/git/BTPDevice, c:/git/BTP, c:/git/TinyShell -- the same layout the
# [env:native] symlinks use.
foreach(_dep BTPDevice BTP TinyShell)
    get_filename_component(_dir "${CMAKE_CURRENT_LIST_DIR}/../${_dep}" ABSOLUTE)
    if(NOT EXISTS "${_dir}/CMakeLists.txt")
        message(FATAL_ERROR "bally_OS expects ${_dep} checked out next to it, at ${_dir}")
    endif()
    list(APPEND EXTRA_COMPONENT_DIRS "${_dir}")
endforeach()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(BallyRobot_IDF)
