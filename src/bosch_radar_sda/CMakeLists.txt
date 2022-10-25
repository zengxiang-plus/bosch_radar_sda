cmake_minimum_required(VERSION 2.8.3)

project(bosch_radar_sda)

add_compile_options(-std=c++14)
add_compile_options(-Wall -Wno-unused-variable)

# plusai cmake module path
set(PLUSAI_CMAKE_MODULE_PATH "/opt/plusai/var/cmake/Modules" CACHE PATH "")
list(APPEND CMAKE_MODULE_PATH ${PLUSAI_CMAKE_MODULE_PATH})
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

include_directories(${CATKIN_DEVEL_PREFIX}/include)

# jsoncpp
find_package(JsonCpp REQUIRED)
include_directories(${JsonCpp_INCLUDE_DIRS})
message(STATUS "JsonCpp_INCLUDE_DIRS: ${JsonCpp_INCLUDE_DIRS}")
message(STATUS "JsonCpp_LIBRARIES: ${JsonCpp_LIBRARIES}")

find_package(catkin REQUIRED COMPONENTS
  can_common
  roscpp
  rospy
  std_msgs
  geometry_msgs
  radar_msgs
  bosch_radar_sda_msgs
)

# import gtest after catkin, see https://plusai.slack.com/archives/C2LD3K1L5/p1611622999024200
# GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
message(STATUS "GTEST_LIBRARIES: ${GTEST_LIBRARIES}")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES can_node
  CATKIN_DEPENDS can_common roscpp rospy std_msgs bosch_radar_sda_msgs radar_msgs
#  DEPENDS system_lib
)

if(DEFINED ENV{drive_root})
    set(drive_DIR "$ENV{drive_root}")
else()
    set(drive_DIR "/opt/plusai/")
endif()
list(APPEND CMAKE_PREFIX_PATH "${drive_DIR}")

# PlusCommon
find_package(PlusaiCommon REQUIRED)
include_directories(${PlusaiCommon_INCLUDE_DIRS})
link_directories(${PlusaiCommon_LIBRARY_DIRS})

# GLog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})
message(STATUS "GLOG_LIBRARIES: ${GLOG_LIBRARIES}")

# GFlags
find_package(Gflags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})
message(STATUS "GFLAGS_LIBRARIES: ${GFLAGS_LIBRARIES}")

message(">>> CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH true)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

file(GLOB SOURCES "src/*.cpp")

add_library(${PROJECT_NAME}
  ${SOURCES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  plusai_common
  ${GFLAGS_LIBRARIES}
  ${JsonCpp_LIBRARIES}
  ${GLOG_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  tools/radar_sda_can_node.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

# install
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# unit test 
function(create_test testName)
    message(STATUS "Creating test ${testName}")
    add_executable(test_${testName}
        EXCLUDE_FROM_ALL
        test/test_${testName}.cpp)
    target_link_libraries(test_${testName}
        ${PROJECT_NAME}
        ${drive_LIBRARIES}
        ${GTEST_BOTH_LIBRARIES}
        ${GLOG_LIBRARIES})
    if(ENABLE_COVERAGE EQUAL "ON")
        target_link_libraries(test_${testName} gcov)
    endif()
    add_test(NAME test/${PROJECT_NAME}/test_${testName}
        COMMAND test_${testName}
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/test/${PROJECT_NAME}")

    set_target_properties(test_${testName}
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/test/${PROJECT_NAME}")
    add_dependencies(check
        test_${testName})
endfunction()

add_custom_target(check
    COMMAND ${CMAKE_CTEST_COMMAND})
add_custom_target(rerun_failed_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --rerun-failed)
enable_testing()

create_test(radar_decoder)
create_test(radar_encoder)