#pragma once
#include <vector>
#include <queue>
#include <unordered_map>
#include <optional>
#include <cmath>
#include <functional>
#include <algorithm>

namespace astar {

    struct Point {
        int x{};
        int y{};
        bool operator==(const Point& o) const noexcept { return x == o.x && y == o.y; }
    };

    struct PointHash {
        std::size_t operator()(const Point& p) const noexcept {
            return (static_cast<std::size_t>(p.x) << 32) ^ static_cast<std::size_t>(p.y);
        }
    };

    struct Grid {
        int w{ 0 }, h{ 0 };
        std::vector<unsigned char> cells;

        Grid() = default;
        Grid(int width, int height) : w(width), h(height), cells(w* h, 0) {}

        bool in_bounds(Point p) const noexcept {
            return p.x >= 0 && p.y >= 0 && p.x < w && p.y < h;
        }
        bool passable(Point p) const noexcept {
            return cells[p.y * w + p.x] == 0;
        }
        void set_block(Point p, bool blocked = true) {
            if (in_bounds(p)) cells[p.y * w + p.x] = blocked ? 1 : 0;
        }

        std::vector<Point> neighbors4(Point p) const {
            static const int dx[4] = { 1,-1,0,0 };
            static const int dy[4] = { 0,0,1,-1 };
            std::vector<Point> out;
            out.reserve(4);
            for (int i = 0; i < 4; ++i) {
                Point n{ p.x + dx[i], p.y + dy[i] };
                if (in_bounds(n) && passable(n)) out.push_back(n);
            }
            return out;
        }
    };

    inline double manhattan(Point a, Point b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

    inline std::optional<std::vector<Point>> search(const Grid& grid, Point start, Point goal) {
        using PQItem = std::pair<double, Point>;
        auto cmp = [](const PQItem& a, const PQItem& b) { return a.first > b.first; };
        std::priority_queue<PQItem, std::vector<PQItem>, decltype(cmp)> open(cmp);

        std::unordered_map<Point, Point, PointHash> came_from;
        std::unordered_map<Point, double, PointHash> g;

        g[start] = 0.0;
        open.push({ manhattan(start, goal), start });

        while (!open.empty()) {
            auto [fcur, cur] = open.top(); (void)fcur;
            open.pop();

            if (cur == goal) {
                std::vector<Point> path;
                for (Point p = goal;;) {
                    path.push_back(p);
                    if (p == start) break;
                    p = came_from[p];
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (auto& n : grid.neighbors4(cur)) {
                double tentative = g[cur] + 1.0;
                auto it = g.find(n);
                if (it == g.end() || tentative < it->second) {
                    came_from[n] = cur;
                    g[n] = tentative;
                    double f = tentative + manhattan(n, goal);
                    open.push({ f, n });
                }
            }
        }
        return std::nullopt;
    }

}
