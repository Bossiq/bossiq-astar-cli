/// @file main.cpp
/// @brief CLI demo for the header-only A* library.

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <cstdlib>
#include "astar.hpp"

// Project version (injected by CMake)
#ifndef PROJECT_VERSION
#define PROJECT_VERSION "0.1.0"
#endif

using astar::Point;
using astar::Grid;

// ANSI helpers
#ifdef _WIN32
#include <windows.h>
static void enable_ansi() {
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD mode = 0;
    GetConsoleMode(h, &mode);
    SetConsoleMode(h, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
}
#else
static void enable_ansi() {} // POSIX terminals support ANSI natively
#endif

namespace clr {
    inline const char* reset  = "\033[0m";
    inline const char* bold   = "\033[1m";
    inline const char* red    = "\033[91m";
    inline const char* green  = "\033[92m";
    inline const char* yellow = "\033[93m";
    inline const char* cyan   = "\033[96m";
    inline const char* dim    = "\033[2m";
}

// Usage / version

static void print_usage() {
    std::cout
        << clr::bold << "astar" << clr::reset
        << " -- header-only A* pathfinding demo  v" PROJECT_VERSION "\n"
        "\n"
        << clr::yellow << "USAGE:" << clr::reset << "\n"
        "  astar                  Run a 10x10 grid demo\n"
        "  astar --file MAP.txt   Load a grid from a text file\n"
        "  astar --animate        Run the demo with step-by-step expansion\n"
        "  astar --selftest       Run built-in correctness tests\n"
        "  astar --version        Show version\n"
        "  astar --help           Show this help\n"
        "\n"
        << clr::yellow << "MAP FILE FORMAT:" << clr::reset << "\n"
        "  . = passable   # = wall   S = start   G = goal\n"
        "  Example:\n"
        "    S....\n"
        "    .###.\n"
        "    .....G\n";
}

static void print_version() {
    std::cout << "astar v" PROJECT_VERSION "\n";
}

// Grid rendering

static void render_grid(const Grid& g, const std::vector<Point>& path,
                        Point s, Point t, bool use_color = true)
{
    std::vector<bool> on_path(static_cast<std::size_t>(g.w) * g.h, false);
    for (auto& p : path)
        on_path[static_cast<std::size_t>(p.y) * g.w + p.x] = true;

    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            Point p{ x, y };
            bool is_start = (p == s);
            bool is_goal  = (p == t);
            bool is_wall  = !g.passable(p);
            bool is_path  = on_path[static_cast<std::size_t>(y) * g.w + x];

            if (use_color) {
                if (is_start)     std::cout << clr::bold << clr::cyan   << "S" << clr::reset;
                else if (is_goal) std::cout << clr::bold << clr::cyan   << "G" << clr::reset;
                else if (is_wall) std::cout << clr::red   << "#" << clr::reset;
                else if (is_path) std::cout << clr::green << "*" << clr::reset;
                else              std::cout << clr::dim   << "." << clr::reset;
            } else {
                if (is_start)     std::cout << 'S';
                else if (is_goal) std::cout << 'G';
                else if (is_wall) std::cout << '#';
                else if (is_path) std::cout << '*';
                else              std::cout << '.';
            }
        }
        std::cout << '\n';
    }
}

// Step-by-step animated visualization

static void render_animated(const Grid& g, const std::vector<Point>& path,
                            Point s, Point t)
{
    std::cout << clr::bold << "\n--- A* expansion animation ---\n" << clr::reset;
    for (std::size_t step = 1; step <= path.size(); ++step) {
        // Clear screen (ANSI)
        std::cout << "\033[2J\033[H";
        std::cout << clr::bold << "Step " << step << " / " << path.size() << clr::reset << "\n\n";

        std::vector<Point> partial(path.begin(), path.begin() + static_cast<long>(step));
        render_grid(g, partial, s, t);

        std::cout << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
    }
    std::cout << "\n" << clr::green << "Path complete! Length: " << path.size() << clr::reset << "\n";
}

// Load grid from text file

struct LoadResult {
    Grid  grid;
    Point start{ -1, -1 };
    Point goal { -1, -1 };
};

static std::optional<LoadResult> load_grid_file(const std::string& filename) {
    std::ifstream in(filename);
    if (!in.is_open()) {
        std::cerr << clr::red << "Error: cannot open file '" << filename << "'\n" << clr::reset;
        return std::nullopt;
    }

    std::vector<std::string> rows;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty()) rows.push_back(line);
    }

    if (rows.empty()) {
        std::cerr << clr::red << "Error: file is empty\n" << clr::reset;
        return std::nullopt;
    }

    int h = static_cast<int>(rows.size());
    int w = static_cast<int>(rows[0].size());

    LoadResult lr;
    lr.grid = Grid(w, h);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w && x < static_cast<int>(rows[y].size()); ++x) {
            char c = rows[y][static_cast<std::size_t>(x)];
            if (c == '#') {
                lr.grid.set_block({ x, y }, true);
            } else if (c == 'S' || c == 's') {
                lr.start = { x, y };
            } else if (c == 'G' || c == 'g') {
                lr.goal = { x, y };
            }
        }
    }

    if (lr.start.x < 0 || lr.goal.x < 0) {
        std::cerr << clr::red << "Error: map must contain 'S' (start) and 'G' (goal)\n" << clr::reset;
        return std::nullopt;
    }

    return lr;
}

// Self-test suite

static int run_selftest() {
    int passed = 0;
    int failed = 0;

    auto check = [&](const char* name, bool ok) {
        if (ok) {
            std::cout << clr::green << "  PASS " << clr::reset << name << "\n";
            ++passed;
        } else {
            std::cout << clr::red   << "  FAIL " << clr::reset << name << "\n";
            ++failed;
        }
    };

    std::cout << clr::bold << "Running self-tests...\n" << clr::reset;

    // 1. Open 3x3 grid
    {
        Grid g(3, 3);
        auto res = astar::search(g, { 0, 0 }, { 2, 2 });
        check("open 3x3 grid", res.has_value() && res->size() == 5);
    }

    // 2. Start == Goal
    {
        Grid g(5, 5);
        auto res = astar::search(g, { 2, 2 }, { 2, 2 });
        check("start == goal", res.has_value() && res->size() == 1 && (*res)[0] == Point{ 2, 2 });
    }

    // 3. Completely blocked goal
    {
        Grid g(5, 5);
        g.set_block({ 1, 0 }); g.set_block({ 0, 1 });
        g.set_block({ 1, 1 });
        auto res = astar::search(g, { 0, 0 }, { 4, 4 });
        check("blocked -- no path", !res.has_value());
    }

    // 4. Out-of-bounds start
    {
        Grid g(5, 5);
        auto res = astar::search(g, { -1, 0 }, { 4, 4 });
        check("out-of-bounds start", !res.has_value());
    }

    // 5. Blocked start
    {
        Grid g(5, 5);
        g.set_block({ 0, 0 });
        auto res = astar::search(g, { 0, 0 }, { 4, 4 });
        check("blocked start", !res.has_value());
    }

    // 6. Wall with gap (the default demo scenario)
    {
        Grid g(10, 10);
        for (int x = 0; x < 10; ++x) {
            if (x == 5) continue;
            g.set_block({ x, 4 });
        }
        auto res = astar::search(g, { 0, 0 }, { 9, 9 });
        check("wall with gap", res.has_value() && res->size() > 0);
    }

    // 7. Large grid performance (100x100 open)
    {
        Grid g(100, 100);
        auto t0  = std::chrono::steady_clock::now();
        auto res = astar::search(g, { 0, 0 }, { 99, 99 });
        auto t1  = std::chrono::steady_clock::now();
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        check("100x100 open grid (< 1 s)", res.has_value() && ms < 1000);
    }

    // 8. Path starts at start and ends at goal
    {
        Grid g(5, 5);
        Point s{ 0, 0 }, t{ 4, 4 };
        auto res = astar::search(g, s, t);
        bool ok = res.has_value() && res->front() == s && res->back() == t;
        check("path endpoints correct", ok);
    }

    std::cout << "\n" << clr::bold << passed << " passed, " << failed << " failed.\n" << clr::reset;
    return failed == 0 ? 0 : 1;
}

// Main

int main(int argc, char** argv) {
    enable_ansi();

    bool animate = false;
    std::string map_file;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h" || arg == "/?") {
            print_usage();
            return 0;
        }
        if (arg == "--version" || arg == "-v") {
            print_version();
            return 0;
        }
        if (arg == "--selftest") {
            return run_selftest();
        }
        if (arg == "--animate") {
            animate = true;
            continue;
        }
        if ((arg == "--file" || arg == "-f") && i + 1 < argc) {
            map_file = argv[++i];
            continue;
        }

        std::cerr << clr::red << "Unknown argument: " << arg << "\n" << clr::reset;
        print_usage();
        return 1;
    }

    // Set up grid
    Grid  g(10, 10);
    Point start{ 0, 0 };
    Point goal { 9, 9 };

    if (!map_file.empty()) {
        auto lr = load_grid_file(map_file);
        if (!lr) return 1;
        g     = std::move(lr->grid);
        start = lr->start;
        goal  = lr->goal;
    } else {
        // Default demo: horizontal wall with a gap at x=5
        for (int x = 0; x < 10; ++x) {
            if (x == 5) continue;
            g.set_block({ x, 4 }, true);
        }
    }

    // Run A*
    auto t0   = std::chrono::steady_clock::now();
    auto path = astar::search(g, start, goal);
    auto t1   = std::chrono::steady_clock::now();
    auto us   = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

    if (!path) {
        std::cout << clr::red << "No path found.\n" << clr::reset;
        return 0;
    }

    std::cout << clr::bold << "Path length: " << path->size()
              << clr::reset << clr::dim << "  (" << us << " us)\n" << clr::reset;

    if (animate) {
        render_animated(g, *path, start, goal);
    } else {
        render_grid(g, *path, start, goal);
    }

    return 0;
}
