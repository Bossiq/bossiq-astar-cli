#include <iostream>
#include <string>
#include <vector>
#include "astar.hpp"

using astar::Point;
using astar::Grid;

static void print_usage() {
    std::cout <<
        R"(astar — header-only A* demo
Usage:
  astar --help        Show this help
  astar --selftest    Run a trivial self-test
  astar               Run a small 10x10 grid demo and print the path
)";
}

static void render_with_path(const Grid& g, const std::vector<Point>& path, Point s, Point t) {
    std::vector<std::string> canvas(g.h, std::string(g.w, '.'));
    for (int y = 0; y < g.h; ++y)
        for (int x = 0; x < g.w; ++x)
            if (!g.passable({ x,y })) canvas[y][x] = '#';
    for (auto& p : path) canvas[p.y][p.x] = '*';
    canvas[s.y][s.x] = 'S';
    canvas[t.y][t.x] = 'G';

    for (auto& row : canvas) std::cout << row << '\n';
}

int main(int argc, char** argv) {
    if (argc > 1) {
        std::string arg = argv[1];
        if (arg == "--help" || arg == "-h" || arg == "/?") {
            print_usage();
            return 0;
        }
        if (arg == "--selftest") {
            Grid g(3, 3);
            auto res = astar::search(g, { 0,0 }, { 2,2 });
            if (!res || res->empty()) {
                std::cerr << "SELFTEST FAILED\n";
                return 1;
            }
            std::cout << "SELFTEST OK\n";
            return 0;
        }
    }

    Grid g(10, 10);
    for (int x = 0; x < 10; ++x) {
        if (x == 5) continue; // gap
        g.set_block({ x, 4 }, true);
    }
    Point start{ 0,0 };
    Point goal{ 9,9 };

    auto path = astar::search(g, start, goal);
    if (!path) {
        std::cout << "No path found.\n";
        return 0;
    }

    std::cout << "Path length: " << path->size() << "\n";
    render_with_path(g, *path, start, goal);
    return 0;
}
