cmake_minimum_required(VERSION 3.13)
#set(CMAKE_VERBOSE_MAKEFILE ON) -DCMAKE_VERBOSE_MAKEFILE=ON --debug-output --trace
project(advanved-microtech CXX C)

include(FetchContent)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT MSP_COMPILER)
  add_subdirectory(mock)
endif()

add_subdirectory(libs)
if (NOT MSP_COMPILER)
  add_subdirectory(example)
endif()

add_subdirectory(exercise1)
add_subdirectory(exercise2)
add_subdirectory(exercise3)
add_subdirectory(exercise4)
#add_subdirectory(exercise5)

