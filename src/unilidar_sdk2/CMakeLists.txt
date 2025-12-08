cmake_minimum_required(VERSION 3.10)
project(unilidar_sdk2)

find_package(ament_cmake REQUIRED)

set(SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/unitree_lidar_sdk")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(SDK_ARCH "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|x64)$")
  set(SDK_ARCH "x86_64")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(SDK_LIB "${SDK_ROOT}/lib/${SDK_ARCH}/libunilidar_sdk2.a")

if(NOT EXISTS "${SDK_LIB}")
  message(FATAL_ERROR "Missing UniLidar library: ${SDK_LIB}")
endif()

add_library(unilidar_sdk2 STATIC IMPORTED GLOBAL)
set_target_properties(unilidar_sdk2 PROPERTIES
  IMPORTED_LOCATION "${SDK_LIB}"
  INTERFACE_INCLUDE_DIRECTORIES "${SDK_ROOT}/include"
)

add_library(unilidar_sdk2::unilidar_sdk2 ALIAS unilidar_sdk2)

install(FILES "${SDK_LIB}" DESTINATION lib)
install(DIRECTORY "${SDK_ROOT}/include/" DESTINATION include)

ament_export_targets(unilidar_sdk2 HAS_LIBRARY_TARGET)
ament_export_include_directories("${SDK_ROOT}/include")
ament_export_libraries(unilidar_sdk2)

ament_package()
