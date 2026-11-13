cmake_minimum_required(VERSION 3.20)
project(hullcad VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(HULLCAD_ENABLE_CUDA "Build the CUDA compute backend" OFF)

find_package(Threads REQUIRED)

add_library(hullcad_core
  src/bspline.cpp
  src/hull_param.cpp
  src/mesh.cpp
  src/parallel.cpp
  src/backend_cpu.cpp
  src/hydrostatics.cpp
  src/lines_plan.cpp
)
target_include_directories(hullcad_core PUBLIC include)
target_link_libraries(hullcad_core PUBLIC Threads::Threads)
if(MSVC)
  target_compile_options(hullcad_core PRIVATE /W4)
else()
  target_compile_options(hullcad_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(HULLCAD_ENABLE_CUDA)
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 20)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 75 86 89)
  endif()
  find_package(CUDAToolkit REQUIRED)
  target_sources(hullcad_core PRIVATE src/backend_cuda.cu)
  target_compile_definitions(hullcad_core PUBLIC HULLCAD_HAVE_CUDA=1)
  target_link_libraries(hullcad_core PUBLIC CUDA::cudart)
endif()

add_executable(hullcad_demo apps/hullcad_demo.cpp)
target_link_libraries(hullcad_demo PRIVATE hullcad_core)

enable_testing()
add_executable(hullcad_tests tests/test_main.cpp)
target_link_libraries(hullcad_tests PRIVATE hullcad_core)
add_test(NAME hullcad_tests COMMAND hullcad_tests)
