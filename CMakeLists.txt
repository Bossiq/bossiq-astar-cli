cmake_minimum_required(VERSION 3.20)

project(bossiq-astar-cli
  VERSION 0.1.0
  DESCRIPTION "Header-only A* library + CLI ASCII visualizer"
  LANGUAGES CXX
)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Executable target
add_executable(astar src/main.cpp)
target_include_directories(astar PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Warnings
if (MSVC)
  target_compile_options(astar PRIVATE /W4 /permissive-)
else()
  target_compile_options(astar PRIVATE -Wall -Wextra -Wpedantic)
endif()
