# bossiq-astar-cli

[![CI](https://github.com/Bossiq/bossiq-astar-cli/actions/workflows/ci.yml/badge.svg)](https://github.com/Bossiq/bossiq-astar-cli/actions)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

**Header-only C++17 A\* pathfinding library** with an ASCII CLI demo that visualises the shortest path on a 2-D grid.

```
S*........
.*........
.*.....#..
.*....#...
#*###.####
..***.....
....*.....
.....*....
......*...
.......***G
```

---

## Features

| Feature | Detail |
|---|---|
| **Header-only library** | Drop `include/astar.hpp` into any C++17 project |
| **Templatized** | Plug in custom heuristics (Manhattan, Chebyshev, Euclidean, or your own) |
| **4-dir & 8-dir movement** | `neighbors4` / `neighbors8` built in |
| **Zero-alloc neighbours** | Returns `std::array` instead of `std::vector` |
| **Validated inputs** | Gracefully returns `std::nullopt` for blocked/out-of-bounds start or goal |
| **CLI demo** | Coloured ASCII grid, file loading, step-by-step animation |
| **Cross-platform** | Builds on Linux, macOS, and Windows (GCC, Clang, MSVC) |
| **CI** | GitHub Actions on all three platforms |

---

## Quick Start

### Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

### Run

```bash
# Default 10x10 demo
./build/astar

# With step-by-step animation
./build/astar --animate

# Load a custom map
./build/astar --file maps/maze.txt

# Run self-tests
./build/astar --selftest

# Show version
./build/astar --version
```

### Map File Format

Plain text. `.` = passable, `#` = wall, `S` = start, `G` = goal.

```
S.....
.###..
......
..###.
.....G
```

---

## Use as a Library

### 1. Copy the header

Copy `include/astar.hpp` into your project's include path.

### 2. Or use CMake `FetchContent`

```cmake
include(FetchContent)
FetchContent_Declare(astar
  GIT_REPOSITORY https://github.com/Bossiq/bossiq-astar-cli.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(astar)

target_link_libraries(your_target PRIVATE astar_lib)
```

### 3. Example usage

```cpp
#include "astar.hpp"

astar::Grid grid(20, 20);
grid.set_block({5, 3});
grid.set_block({5, 4});

// Default: Manhattan heuristic, 4-directional
auto path = astar::search(grid, {0, 0}, {19, 19});

if (path) {
    for (auto& p : *path)
        std::cout << "(" << p.x << "," << p.y << ") ";
}

// Custom: Chebyshev heuristic for 8-directional grids
auto path8 = astar::search(grid, {0, 0}, {19, 19}, astar::chebyshev);
```

---

## API Reference

### `astar::Point`
```cpp
struct Point { int x, y; };
```

### `astar::Grid`
```cpp
Grid(int width, int height);
bool in_bounds(Point p) const noexcept;
bool passable(Point p) const noexcept;           // false for out-of-bounds
void set_block(Point p, bool blocked = true);
std::array<Point,4> neighbors4(Point p, int& count) const noexcept;
std::array<Point,8> neighbors8(Point p, int& count) const noexcept;
```

### `astar::search`
```cpp
template <typename Heuristic = decltype(manhattan), typename Neighbors = ...>
std::optional<std::vector<Point>>
search(const Grid& grid, Point start, Point goal,
       Heuristic heuristic = manhattan,
       Neighbors neighborFn = nullptr);
```

### Built-in Heuristics
- `astar::manhattan(a, b)` — 4-directional (returns `int`)
- `astar::chebyshev(a, b)` — 8-directional (returns `int`)
- `astar::euclidean(a, b)` — any direction (returns `double`)

---

## Project Structure

```
bossiq-astar-cli/
  include/
    astar.hpp          # Header-only A* library
  src/
    main.cpp           # CLI demo
  .github/workflows/
    ci.yml             # GitHub Actions CI
  CMakeLists.txt       # Build system
  .clang-format        # Code style
  LICENSE              # MIT
```

---

## Self-Tests

The `--selftest` flag runs 8 built-in correctness tests:

1. Open grid pathfinding
2. Start == Goal (trivial path)
3. Blocked grid (no path)
4. Out-of-bounds start
5. Blocked start cell
6. Wall with gap
7. Large 100x100 grid (performance)
8. Path endpoint validation

```bash
./build/astar --selftest
```

---

## License

MIT — see [LICENSE](LICENSE).
