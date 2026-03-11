/// @file astar.hpp
/// @brief Header-only A* pathfinding library.
/// @copyright 2025-2026 Marian Oboroceanu — MIT License

#pragma once
#include <vector>
#include <array>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <optional>
#include <cmath>
#include <functional>
#include <algorithm>
#include <type_traits>

namespace astar {

// ──────────────────────────────────────────────
//  Point
// ──────────────────────────────────────────────

/// @brief A 2-D integer coordinate.
struct Point {
    int x{};
    int y{};
    bool operator==(const Point& o) const noexcept { return x == o.x && y == o.y; }
    bool operator!=(const Point& o) const noexcept { return !(*this == o); }
};

/// @brief Hash functor for Point (Cantor-pairing based).
struct PointHash {
    std::size_t operator()(const Point& p) const noexcept {
        // Cantor pairing — much better distribution than shift-XOR
        auto a = static_cast<std::size_t>(p.x >= 0 ? 2 * p.x : -2 * p.x - 1);
        auto b = static_cast<std::size_t>(p.y >= 0 ? 2 * p.y : -2 * p.y - 1);
        return (a + b) * (a + b + 1) / 2 + b;
    }
};

// ──────────────────────────────────────────────
//  Grid
// ──────────────────────────────────────────────

/// @brief A dense 2-D grid of passable / blocked cells.
struct Grid {
    int w{ 0 }, h{ 0 };
    std::vector<unsigned char> cells; ///< 0 = passable, 1 = blocked

    Grid() = default;

    /// @brief Construct a grid of given size (all cells passable).
    Grid(int width, int height)
        : w(width), h(height), cells(static_cast<std::size_t>(w) * h, 0) {}

    /// @brief Check whether a point is inside the grid.
    bool in_bounds(Point p) const noexcept {
        return p.x >= 0 && p.y >= 0 && p.x < w && p.y < h;
    }

    /// @brief Check whether a point is passable. Returns false for out-of-bounds points.
    bool passable(Point p) const noexcept {
        if (!in_bounds(p)) return false;
        return cells[static_cast<std::size_t>(p.y) * w + p.x] == 0;
    }

    /// @brief Block or unblock a cell. Ignores out-of-bounds points.
    void set_block(Point p, bool blocked = true) {
        if (in_bounds(p))
            cells[static_cast<std::size_t>(p.y) * w + p.x] = blocked ? 1 : 0;
    }

    /// @brief Return the 4-connected passable neighbours (up/down/left/right).
    ///
    /// Uses a fixed-size std::array to avoid heap allocation.
    /// @param[out] count  Number of valid neighbours written into the array.
    std::array<Point, 4> neighbors4(Point p, int& count) const noexcept {
        static constexpr int dx[4] = { 1, -1,  0,  0 };
        static constexpr int dy[4] = { 0,  0,  1, -1 };
        std::array<Point, 4> out{};
        count = 0;
        for (int i = 0; i < 4; ++i) {
            Point n{ p.x + dx[i], p.y + dy[i] };
            if (passable(n)) out[static_cast<std::size_t>(count++)] = n;
        }
        return out;
    }

    /// @brief Return the 8-connected passable neighbours (includes diagonals).
    /// @param[out] count  Number of valid neighbours written into the array.
    std::array<Point, 8> neighbors8(Point p, int& count) const noexcept {
        static constexpr int dx[8] = { 1, -1,  0,  0,  1,  1, -1, -1 };
        static constexpr int dy[8] = { 0,  0,  1, -1,  1, -1,  1, -1 };
        std::array<Point, 8> out{};
        count = 0;
        for (int i = 0; i < 8; ++i) {
            Point n{ p.x + dx[i], p.y + dy[i] };
            if (passable(n)) out[static_cast<std::size_t>(count++)] = n;
        }
        return out;
    }
};

// ──────────────────────────────────────────────
//  Heuristic helpers
// ──────────────────────────────────────────────

/// @brief Manhattan distance — admissible heuristic for 4-directional movement.
inline int manhattan(Point a, Point b) noexcept {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

/// @brief Chebyshev distance — admissible heuristic for 8-directional movement.
inline int chebyshev(Point a, Point b) noexcept {
    return std::max(std::abs(a.x - b.x), std::abs(a.y - b.y));
}

/// @brief Euclidean distance (returns double) — admissible for any movement.
inline double euclidean(Point a, Point b) noexcept {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

// ──────────────────────────────────────────────
//  A* search  (templatized heuristic + neighbors)
// ──────────────────────────────────────────────

/// @brief Run A* from @p start to @p goal on @p grid.
///
/// @tparam Heuristic  Callable `(Point, Point) -> arithmetic`, must be admissible.
/// @tparam Neighbors  Callable `(const Grid&, Point, int&) -> array-like of Point`.
///
/// @return The shortest path (including start and goal) or @c std::nullopt if
///         no path exists, or if start/goal are invalid.
///
/// Default heuristic is Manhattan; default neighbor function is 4-connected.
template <
    typename Heuristic  = decltype(manhattan),
    typename Neighbors  = std::nullptr_t          // sentinel → use Grid::neighbors4
>
inline std::optional<std::vector<Point>>
search(const Grid& grid, Point start, Point goal,
       Heuristic heuristic = manhattan,
       [[maybe_unused]] Neighbors neighborFn = nullptr)
{
    // ── Validate inputs ──────────────────────
    if (!grid.passable(start) || !grid.passable(goal))
        return std::nullopt;

    // Trivial case
    if (start == goal)
        return std::vector<Point>{ start };

    // ── Cost type deduction ──────────────────
    using CostType = std::decay_t<decltype(heuristic(start, goal))>;
    using PQItem   = std::pair<CostType, Point>;

    auto cmp = [](const PQItem& a, const PQItem& b) { return a.first > b.first; };
    std::priority_queue<PQItem, std::vector<PQItem>, decltype(cmp)> open(cmp);

    std::unordered_map<Point, Point, PointHash>    came_from;
    std::unordered_map<Point, CostType, PointHash> g;
    std::unordered_set<Point, PointHash>           closed;

    g[start] = CostType{ 0 };
    open.push({ heuristic(start, goal), start });

    while (!open.empty()) {
        auto [fcur, cur] = open.top();
        open.pop();
        (void)fcur;

        // ── Skip already-expanded nodes (closed set) ──
        if (closed.count(cur))
            continue;
        closed.insert(cur);

        // ── Goal reached — reconstruct path ──
        if (cur == goal) {
            std::vector<Point> path;
            for (Point p = goal; ; ) {
                path.push_back(p);
                if (p == start) break;
                p = came_from[p];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // ── Expand neighbours ──
        // Use the provided neighbor function, or fall back to Grid::neighbors4
        if constexpr (std::is_same_v<Neighbors, std::nullptr_t>) {
            int ncount = 0;
            auto nbrs = grid.neighbors4(cur, ncount);
            for (int i = 0; i < ncount; ++i) {
                const Point& n = nbrs[static_cast<std::size_t>(i)];
                CostType tentative = g[cur] + CostType{ 1 };
                auto it = g.find(n);
                if (it == g.end() || tentative < it->second) {
                    came_from[n] = cur;
                    g[n]         = tentative;
                    open.push({ tentative + heuristic(n, goal), n });
                }
            }
        } else {
            int ncount = 0;
            auto nbrs = neighborFn(grid, cur, ncount);
            for (int i = 0; i < ncount; ++i) {
                const Point& n = nbrs[static_cast<std::size_t>(i)];
                CostType tentative = g[cur] + CostType{ 1 };
                auto it = g.find(n);
                if (it == g.end() || tentative < it->second) {
                    came_from[n] = cur;
                    g[n]         = tentative;
                    open.push({ tentative + heuristic(n, goal), n });
                }
            }
        }
    }

    return std::nullopt;   // no path
}

} // namespace astar
