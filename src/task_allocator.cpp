#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "fleet/fleet.hpp"

namespace fleet {
std::string to_string(Strategy s) {
  switch (s) {
    case Strategy::nearest:
      return "nearest";
    case Strategy::distance:
      return "distance";
    case Strategy::congestion:
      return "congestion";
  }
  throw std::invalid_argument("unknown strategy");
}
Strategy strategy_from(const std::string& s) {
  if (s == "nearest")
    return Strategy::nearest;
  if (s == "distance")
    return Strategy::distance;
  if (s == "congestion")
    return Strategy::congestion;
  throw std::invalid_argument("strategy must be nearest, distance, or congestion");
}
// Rectangular Hungarian algorithm; own implementation, rows <= columns.
std::vector<int> linear_assignment(const std::vector<std::vector<double>>& a) {
  if (a.empty())
    return {};
  int n = static_cast<int>(a.size()), m = static_cast<int>(a[0].size());
  if (n > m)
    throw std::invalid_argument("assignment requires columns >= rows");
  for (const auto& row : a) {
    if (static_cast<int>(row.size()) != m)
      throw std::invalid_argument("ragged costs");
    for (double x : row)
      if (!std::isfinite(x))
        throw std::invalid_argument("nonfinite cost");
  }
  std::vector<double> u(n + 1), v(m + 1);
  std::vector<int> p(m + 1), way(m + 1);
  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(m + 1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(m + 1, false);
    do {
      used[j0] = true;
      int i0 = p[j0], j1 = 0;
      double delta = std::numeric_limits<double>::infinity();
      for (int j = 1; j <= m; ++j)
        if (!used[j]) {
          double cur = a[i0 - 1][j - 1] - u[i0] - v[j];
          if (cur < minv[j]) {
            minv[j] = cur;
            way[j] = j0;
          }
          if (minv[j] < delta) {
            delta = minv[j];
            j1 = j;
          }
        }
      for (int j = 0; j <= m; ++j)
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else
          minv[j] -= delta;
      j0 = j1;
    } while (p[j0] != 0);
    do {
      int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }
  std::vector<int> answer(n, -1);
  for (int j = 1; j <= m; ++j)
    if (p[j])
      answer[p[j] - 1] = j - 1;
  return answer;
}
std::vector<std::pair<int, int>> allocate(const Map& map, const std::vector<Robot>& robots,
                                          const std::vector<Task>& tasks, Strategy strategy,
                                          double lambda, int window, int candidate_limit) {
  if (!std::isfinite(lambda) || lambda < 0 || window < 1 || candidate_limit < 1)
    throw std::invalid_argument("invalid allocation settings");
  std::vector<int> idle, pending;
  for (int i = 0; i < static_cast<int>(robots.size()); ++i)
    if (robots[i].task < 0 && robots[i].state != RobotState::unavailable)
      idle.push_back(i);
  std::sort(idle.begin(), idle.end(), [&](int a, int b) { return robots[a].id < robots[b].id; });
  for (int i = 0; i < static_cast<int>(tasks.size()); ++i)
    if (tasks[i].state == TaskState::pending)
      pending.push_back(i);
  std::sort(pending.begin(), pending.end(),
            [&](int a, int b) { return tasks[a].id < tasks[b].id; });
  if (static_cast<int>(pending.size()) > candidate_limit)
    pending.resize(candidate_limit);
  if (idle.empty() || pending.empty())
    return {};
  // Occupancy is the mean number of other robots at each cell in the next W ticks.
  std::vector<double> occupancy(map.size(), 0);
  if (strategy == Strategy::congestion && lambda > 0)
    for (const auto& r : robots)
      for (int t = 1; t <= window; ++t)
        occupancy[position(r.path, t)] += 1.0 / window;
  std::vector<std::vector<double>> costs(idle.size(),
                                         std::vector<double>(pending.size() + idle.size(), 1e9));
  for (std::size_t i = 0; i < idle.size(); ++i)
    for (std::size_t j = 0; j < pending.size(); ++j) {
      const auto& r = robots[idle[i]];
      const auto& t = tasks[pending[j]];
      int d1 = map.distance(r.cell, t.pickup), d2 = map.distance(t.pickup, t.delivery);
      if (d1 == unreachable || d2 == unreachable) {
        costs[i][j] = 1e12;
        continue;
      }
      double penalty = 0;
      if (strategy == Strategy::congestion && lambda > 0) {
        auto route = map.route(r.cell, t.pickup), leg = map.route(t.pickup, t.delivery);
        route.insert(route.end(), leg.begin() + 1, leg.end());
        for (int c : route) {
          double own = 0;
          for (int tick = 1; tick <= window; ++tick)
            if (position(r.path, tick) == c)
              own += 1.0 / window;
          penalty += std::max(0.0, occupancy[c] - own);
        }
      }
      costs[i][j] = d1 + d2 + lambda * penalty;
    }
  std::vector<int> assignment;
  if (strategy == Strategy::nearest) {
    std::vector<bool> taken(pending.size(), false);
    for (std::size_t i = 0; i < idle.size(); ++i) {
      int best = -1;
      double cost = 1e9;
      for (std::size_t j = 0; j < pending.size(); ++j)
        if (!taken[j] && costs[i][j] < cost) {
          cost = costs[i][j];
          best = static_cast<int>(j);
        }
      assignment.push_back(best);
      if (best >= 0)
        taken[best] = true;
    }
  } else
    assignment = linear_assignment(costs);
  std::vector<std::pair<int, int>> result;
  for (std::size_t i = 0; i < assignment.size(); ++i) {
    int j = assignment[i];
    if (j >= 0 && j < static_cast<int>(pending.size()) && costs[i][j] < 1e9)
      result.emplace_back(idle[i], pending[j]);
  }
  return result;
}
}  // namespace fleet
