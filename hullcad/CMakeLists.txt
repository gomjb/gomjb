cmake_minimum_required(VERSION 3.20)
project(hullcad VERSION 0.3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
get_property(_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT _multi_config AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
include(GNUInstallDirs)

option(HULLCAD_ENABLE_CUDA "Build the CUDA compute backend" OFF)
option(HULLCAD_BUILD_APP "Build the Qt 6 desktop application (needs Qt 6.5+ on CMAKE_PREFIX_PATH)" ON)

if(MSVC)
  # Sources are UTF-8 (Korean UI text); standard-conforming mode; correct __cplusplus for Qt.
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:/utf-8> $<$<COMPILE_LANGUAGE:CXX>:/permissive->
                      $<$<COMPILE_LANGUAGE:CXX>:/Zc:__cplusplus> $<$<COMPILE_LANGUAGE:CXX>:/EHsc>)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

find_package(Threads REQUIRED)

if(HULLCAD_ENABLE_CUDA)
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 20)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 75 86 89)
  endif()
  find_package(CUDAToolkit REQUIRED)
endif()

add_subdirectory(modules)

add_executable(hullcad_demo apps/hullcad_demo.cpp)
target_link_libraries(hullcad_demo PRIVATE hullcad)

enable_testing()
add_executable(hullcad_tests tests/test_main.cpp)
target_link_libraries(hullcad_tests PRIVATE hullcad)
add_test(NAME hullcad_tests COMMAND hullcad_tests)

if(HULLCAD_BUILD_APP)
  find_package(Qt6 6.5 QUIET COMPONENTS Widgets Svg SvgWidgets Concurrent)
  if(Qt6_FOUND)
    add_subdirectory(app)
  else()
    message(STATUS "Qt 6 not found: the desktop app (hullcad_app) is skipped. "
                   "Set CMAKE_PREFIX_PATH to a Qt 6 kit, e.g. C:/Qt/6.8.3/msvc2022_64")
  endif()
endif()
