cmake_minimum_required(VERSION 3.0)
 
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})
 
find_package(pinocchio REQUIRED)

file(GLOB C_SOURCES ./src/*.c)
file(GLOB CPP_SOURCES ./src/*.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})
 
link_directories($ENV{WEBOTS_HOME}/lib/controller)
 
set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
message(STATUS "Sources: ${SOURCES}")

 
include_directories(
    /usr/local/webots/include/controller/c
    /usr/local/webots/include/controller/cpp
    ./include
    /usr/local/include/eigen3
    ${PINOCCHIO_INCLUDE_DIRS}
)

 
add_executable(${PROJECT} ${SOURCES})

target_link_libraries(${PROJECT}
    ${LIBRARIES}
    ${PINOCCHIO_LIBRARIES}
)
 
# Copy the target executable at the right location.
add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR})
 