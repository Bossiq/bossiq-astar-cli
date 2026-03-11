cmake_minimum_required(VERSION 3.20)

project(bossiq-astar-cli
  VERSION 0.1.0
  DESCRIPTION "Header-only A* library + CLI ASCII visualizer"
  LANGUAGES CXX
)

# ── C++17 ─────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Header-only INTERFACE library ─────────────
add_library(astar_lib INTERFACE)
target_include_directories(astar_lib INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ── CLI executable ────────────────────────────
add_executable(astar src/main.cpp)
target_link_libraries(astar PRIVATE astar_lib)
target_compile_definitions(astar PRIVATE
  PROJECT_VERSION="${PROJECT_VERSION}"
)

# ── Compiler warnings ────────────────────────
if (MSVC)
  target_compile_options(astar PRIVATE /W4 /permissive-)
else()
  target_compile_options(astar PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ── Testing ───────────────────────────────────
include(CTest)
enable_testing()
add_test(NAME selftest COMMAND astar --selftest)

# ── Install ───────────────────────────────────
install(TARGETS astar RUNTIME DESTINATION bin)
install(TARGETS astar_lib EXPORT astarTargets)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT astarTargets
  FILE astarTargets.cmake
  NAMESPACE astar::
  DESTINATION lib/cmake/astar
)
