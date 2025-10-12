# bossiq-astar-cli

Header-only **A\*** pathfinding library with a simple ASCII CLI demo.  
Stack: C++17, CMake, MSVC (x64), Windows.

## Build (Windows, Visual Studio 2022)

Open **x64 Native Tools Command Prompt for VS 2022** and run:

```bat
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
