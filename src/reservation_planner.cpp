#include <algorithm>
#include <chrono>
#include <queue>
#include <stdexcept>
#include <tuple>

#include "fleet/fleet.hpp"

namespace fleet {
Reservations::Reservations(int cells, int horizon, const std::vector<Path>& paths)
    : cells_(cells), horizon_(horizon), paths_(paths) {
  if (cells < 1 || horizon < 1 || horizon > 256)
    throw std::invalid_argument("invalid reservation dimensions");
  rebuild();
}
void Reservations::rebuild() {
  owner_.assign(static_cast<std::size_t>(horizon_ + 1) * cells_, -1);
  for (int r = 0; r < static_cast<int>(paths_.size()); ++r)
    for (int t = 0; t <= horizon_; ++t) {
      int c = position(paths_[r], t);
      if (c < 0 || c >= cells_)
        throw std::invalid_argument("reservation outside map");
      auto& slot = owner_[t * cells_ + c];
      if (slot != -1)
        throw std::invalid_argument("conflicting reservation input: robot=" + std::to_string(r) +
                                    " other=" + std::to_string(slot) +
                                    " cell=" + std::to_string(c) + " time=" + std::to_string(t));
      slot = r;
      if (t > 0)
        for (int other = 0; other < r; ++other)
          if (c == position(paths_[other], t - 1) &&
              position(paths_[r], t - 1) == position(paths_[other], t))
            throw std::invalid_argument("edge conflict in reservation input");
    }
}
bool Reservations::vertex(int r, int c, int t) const {
  if (c < 0 || c >= cells_ || t < 0 || t > horizon_)
    return false;
  int owner = owner_[t * cells_ + c];
  return owner == -1 || owner == r;
}
bool Reservations::step(int r, int from, int to, int t) const {
  if (t < 1 || !vertex(r, to, t))
    return false;
  int other = owner_[(t - 1) * cells_ + to];
  return other == -1 || other == r || position(paths_[other], t) != from;
}
bool Reservations::hold(int r, int c, int t) const {
  for (; t <= horizon_; ++t)
    if (!vertex(r, c, t))
      return false;
  return true;
}
bool Reservations::replace(int r, const Path& path) {
  if (r < 0 || r >= static_cast<int>(paths_.size()) || path.empty() || path[0] != paths_[r][0] ||
      path.size() > static_cast<std::size_t>(horizon_ + 1))
    return false;
  if (!vertex(r, path[0], 0))
    return false;
  for (int t = 1; t <= horizon_; ++t)
    if (!step(r, position(path, t - 1), position(path, t), t))
      return false;
  // Commit only after every reservation has passed; no partial overwrite.
  paths_[r] = path;
  rebuild();
  return true;
}
std::string to_string(PlanStatus s) {
  switch (s) {
    case PlanStatus::success:
      return "success";
    case PlanStatus::unreachable:
      return "unreachable";
    case PlanStatus::window_exhausted:
      return "window_exhausted";
    case PlanStatus::expansion_budget:
      return "expansion_budget";
    case PlanStatus::deadline:
      return "deadline";
  }
  throw std::logic_error("unknown planner state");
}
PlanResult plan(const Map& map, const Reservations& res, int robot, int start, int goal,
                const std::vector<int>& blocked, PlanBudget budget) {
  auto began = std::chrono::steady_clock::now();
  if (map.distance(start, goal) == unreachable)
    return {PlanStatus::unreachable, {}, 0};
  if (budget.max_expansions < 0 || budget.deadline_us < 0)
    throw std::invalid_argument("negative planning budget");
  auto closed_cell = [&](int c) {
    return std::find(blocked.begin(), blocked.end(), c) != blocked.end();
  };
  if (closed_cell(start) || !res.vertex(robot, start, 0))
    return {PlanStatus::window_exhausted, {}, 0};
  // Queue key is (f, time, cell), giving deterministic equal-cost ordering.
  using Node = std::tuple<int, int, int>;
  std::priority_queue<Node, std::vector<Node>, std::greater<>> open;
  std::vector<int> parent(static_cast<std::size_t>(res.horizon() + 1) * map.size(), -1);
  parent[start] = start;
  open.emplace(map.distance(start, goal), 0, start);
  int expanded = 0;
  while (!open.empty()) {
    if (budget.deadline_us > 0 && std::chrono::duration_cast<std::chrono::microseconds>(
                                      std::chrono::steady_clock::now() - began)
                                          .count() >= budget.deadline_us)
      return {PlanStatus::deadline, {}, expanded};
    if (expanded >= budget.max_expansions)
      return {PlanStatus::expansion_budget, {}, expanded};
    auto [f, t, c] = open.top();
    open.pop();
    (void)f;
    ++expanded;
    int key = t * map.size() + c;
    if (c == goal && res.hold(robot, c, t)) {
      Path path;
      for (int k = key;; k = parent[k]) {
        path.push_back(k % map.size());
        if (k == start)
          break;
      }
      std::reverse(path.begin(), path.end());
      return {PlanStatus::success, path, expanded};
    }
    if (t == res.horizon())
      continue;
    auto neighbors = map.neighbors(c);
    neighbors.push_back(c);
    for (int v : neighbors) {
      int next = (t + 1) * map.size() + v;
      if (parent[next] != -1 || closed_cell(v) || !res.step(robot, c, v, t + 1))
        continue;
      parent[next] = key;
      open.emplace(t + 1 + map.distance(v, goal), t + 1, v);
    }
  }
  return {PlanStatus::window_exhausted, {}, expanded};
}
std::vector<std::string> check_transition(const Map& map, const std::vector<int>& before,
                                          const std::vector<int>& after,
                                          const std::vector<int>& blocked) {
  std::vector<std::string> errors;
  if (before.size() != after.size())
    return {"fleet_size_changed"};
  for (std::size_t i = 0; i < after.size(); ++i) {
    if (!map.free(before[i]) || !map.free(after[i]))
      errors.push_back("invalid_cell");
    else if (std::abs(before[i] / map.width() - after[i] / map.width()) +
                 std::abs(before[i] % map.width() - after[i] % map.width()) >
             1)
      errors.push_back("illegal_motion");
    if (std::find(blocked.begin(), blocked.end(), after[i]) != blocked.end())
      errors.push_back("closed_cell");
    for (std::size_t j = 0; j < i; ++j) {
      if (after[i] == after[j])
        errors.push_back("vertex_conflict");
      if (before[i] == after[j] && before[j] == after[i])
        errors.push_back("edge_conflict");
    }
  }
  return errors;
}
}  // namespace fleet
