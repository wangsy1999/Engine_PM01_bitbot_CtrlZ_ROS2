cmake_minimum_required(VERSION 3.8)
project(bitbot_engine)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(BITBOT_TEST ON CACHE BOOL "Build tests")

set(BITBOT_DEPENDENCY_USE_PROXY TRUE)
set(BITBOT_DEPENDENCY_USE_LOCAL_FILE TRUE)
set(BITBOT_DEPENDENCY_LOCAL_FILE_PATH ${CMAKE_SOURCE_DIR}/dependencies)
include(dependencies/bitbot.cmake)
BitbotAddDependency(
  NAME           bitbot_kernel
  FILE_NAME      "bitbot_kernel-main.zip"
  FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
  USE_LOCAL_FILE TRUE
)

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen version=${Eigen3_VERSION}")
set(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DPROJECT_ROOT_DIR="${PROJECT_ROOT_DIR}")
message("PROJECT_ROOT_DIR: ${PROJECT_ROOT_DIR}")
include_directories(${PROJECT_SOURCE_DIR}/CtrlZ)
option(BUILD_DOC "Build documentation" OFF)

if(WIN32)
  set(ONNXRUNTIME_ROOT "F:/onnxruntime-win-x64-1.17.3")
elseif(UNIX)
    set(ONNXRUNTIME_ROOT "/home/wsy/rl/onnxruntime-linux-x64-1.21.0")
else()
    message(FATAL_ERROR "Unsupported platform")
endif()

if(DEFINED VCPKG_INSTALLED_DIR)
    message("vcpkgDIR=${VCPKG_INSTALLED_DIR}")
    message("using vcpkg onnxruntime")
    set(ONNXRUNTIME_ROOT "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}")
endif()
# --- 修复开始 ---
message(STATUS "ONNX Path: ${ONNXRUNTIME_ROOT}")

include_directories(${ONNXRUNTIME_ROOT}/include)

find_library(ONNXRUNTIME_LIB onnxruntime 
    PATHS ${ONNXRUNTIME_ROOT}/lib 
    NO_DEFAULT_PATH
)

if(NOT ONNXRUNTIME_LIB)
    message(WARNING "Could not find libonnxruntime.so in ${ONNXRUNTIME_ROOT}/lib. Check your path!")
else()
    message(STATUS "Found ONNX Runtime lib: ${ONNXRUNTIME_LIB}")
endif()
# --- 修复结束 ---
add_subdirectory(CtrlZ)
find_package(yaml-cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(interface_protocol REQUIRED)
include_directories("/usr/include/eigen3")

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/device BITBOT_ENGINE_DEVICE_SRC)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/bus BITBOT_ENGINE_BUS_SRC)
set(BITBOT_ENGINE_SRC
  ${BITBOT_ENGINE_DEVICE_SRC}
  ${BITBOT_ENGINE_BUS_SRC}
  ${BITBOT_ENGINE_KERNEL_SRC}
)

add_library(bitbot_engine ${BITBOT_ENGINE_SRC})
target_include_directories(bitbot_engine PUBLIC
  $<BUILD_INTERFACE:${THIRD_PARTY_INCLUDE}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bitbot_engine
  rclcpp
  std_msgs
  ament_index_cpp
interface_protocol
)
target_link_libraries(bitbot_engine BitbotKernel)

add_executable(main_app 
              main.cpp
              user_func.h
              user_func.cpp
              types.hpp)
target_include_directories(main_app PUBLIC
  include
  ${OVINF_INCLUDED}
  ${OpenVINO_INCLUDE_DIRS}
)
ament_target_dependencies(bitbot_engine
  rclcpp
  sensor_msgs
  interface_protocol
)
target_link_libraries(main_app
  bitbot_engine
  CtrlZ
  ${ONNXRUNTIME_LIB}
)



install(TARGETS
  bitbot_engine main_app
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
