cmake_minimum_required(VERSION 2.4.6)

project(frilibrary_ros)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
FILE(GLOB SRC_FILES src/*.cpp)
add_library(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} rt)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# Generating Config.cmake for projects outside ros
# SET(V_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)
# SET(V_LIB_DIRS ${PROJECT_SOURCE_DIR}/lib)
# SET(V_LIBS pthread dl m ${PROJECT_NAME})
# CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.in ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake @ONLY)
