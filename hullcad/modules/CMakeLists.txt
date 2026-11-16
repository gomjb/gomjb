# One library per functional module. Each module sees only its own headers and those of the
# modules it links to, so the dependency direction below is enforced by the compiler:
#
#   core <- cad <- naval
#           cad <- fairing
#           cad <- structure <- production_design <- planning <- production <- project
#                         yard <- production_design
#
# hullcad (INTERFACE) bundles every module for the applications.

function(hullcad_module name)
  cmake_parse_arguments(M "" "" "SOURCES;DEPS" ${ARGN})
  set(target hullcad_${name})
  if(M_SOURCES)
    add_library(${target} ${M_SOURCES})
    target_include_directories(${target} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/${name}/include)
    target_link_libraries(${target} PUBLIC ${M_DEPS})
    if(MSVC)
      target_compile_options(${target} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/W4>)
    else()
      target_compile_options(${target} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wpedantic>)
    endif()
  else()
    add_library(${target} INTERFACE)
    target_include_directories(${target} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/${name}/include)
    target_link_libraries(${target} INTERFACE ${M_DEPS})
  endif()
  add_library(hullcad::${name} ALIAS ${target})
endfunction()

# 공통 기반: vector math, thread pool, dates and working-day calendars
hullcad_module(core
  SOURCES core/src/parallel.cpp core/src/calendar.cpp
  DEPS Threads::Threads)

# 선형 CAD: B-spline curves/surfaces, parametric hull form, mesh, lines plan
hullcad_module(cad
  SOURCES cad/src/bspline.cpp cad/src/hull_param.cpp cad/src/mesh.cpp cad/src/lines_plan.cpp
  DEPS hullcad::core)

# 선형 순정: penalised curve fairing, curvature analysis, faired hull sections
hullcad_module(fairing
  SOURCES fairing/src/fairing.cpp
  DEPS hullcad::cad hullcad::core)

# 선박 계산: hydrostatics and stability on multithreaded CPU / CUDA backends
hullcad_module(naval
  SOURCES naval/src/backend_cpu.cpp naval/src/hydrostatics.cpp
  DEPS hullcad::core hullcad::cad)
if(HULLCAD_ENABLE_CUDA)
  target_sources(hullcad_naval PRIVATE naval/src/backend_cuda.cu)
  target_compile_definitions(hullcad_naval PUBLIC HULLCAD_HAVE_CUDA=1)
  target_link_libraries(hullcad_naval PUBLIC CUDA::cudart)
endif()

# 구조 설계: frame table, reference-based structural members, hull sections
hullcad_module(structure
  SOURCES structure/src/structure.cpp
  DEPS hullcad::cad)

# 조선소 설비: facility limits and utility supply capacity (data only)
hullcad_module(yard)

# 생산 설계: zones, weight model, yard checks, block division, quantity takeoff
hullcad_module(production_design
  SOURCES production_design/src/blocks.cpp production_design/src/process.cpp
  DEPS hullcad::structure hullcad::yard hullcad::core)

# 중일정 관리: erection sequence and backward stage schedule
hullcad_module(planning
  SOURCES planning/src/schedule.cpp
  DEPS hullcad::production_design hullcad::core)

# 생산 관리: utility estimate, daily load, peaks and supply capacity check
hullcad_module(production
  SOURCES production/src/utilities.cpp production/src/daily_load.cpp
  DEPS hullcad::planning hullcad::production_design hullcad::yard)

# 프로젝트 설정: combined yard profile and INI files
hullcad_module(project
  SOURCES project/src/yard_profile.cpp
  DEPS hullcad::production hullcad::planning hullcad::production_design hullcad::core)

add_library(hullcad INTERFACE)
target_link_libraries(hullcad INTERFACE
  hullcad::core hullcad::cad hullcad::fairing hullcad::naval hullcad::structure hullcad::yard
  hullcad::production_design hullcad::planning hullcad::production hullcad::project)
