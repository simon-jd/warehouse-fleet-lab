#include <algorithm>
#include <fstream>
#include <queue>
#include <stdexcept>

#include "fleet/fleet.hpp"

namespace fleet {
Map::Map(std::vector<std::string> rows)
    : width_(rows.empty() ? 0 : static_cast<int>(rows[0].size())), rows_(std::move(rows)) {
  if (width_ == 0 || rows_.size() > 64 || width_ > 64)
    throw std::invalid_argument("map must be 1..64 by 1..64");
  for (const auto& row : rows_) {
    if (static_cast<int>(row.size()) != width_)
      throw std::invalid_argument("ragged map");
    for (char ch : row)
      if (ch != '.' && ch != '#')
        throw std::invalid_argument("map allows only . and #");
  }
  for (int c = 0; c < size(); ++c)
    if (free(c))
      cells_.push_back(c);
  if (cells_.empty())
    throw std::invalid_argument("empty traversable map");
  // Bounded 64x64 input caps this dense static-distance cache at 64 MiB.
  distances_.assign(static_cast<std::size_t>(size()) * size(), unreachable);
  for (int s : cells_) {
    std::queue<int> q;
    q.push(s);
    distances_[s * size() + s] = 0;
    while (!q.empty()) {
      int u = q.front();
      q.pop();
      for (int v : neighbors(u))
        if (distances_[s * size() + v] == unreachable) {
          distances_[s * size() + v] = distances_[s * size() + u] + 1;
          q.push(v);
        }
    }
  }
}
Map Map::load(const std::string& path) {
  std::ifstream in(path);
  if (!in)
    throw std::runtime_error("cannot open map: " + path);
  std::vector<std::string> rows;
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    rows.push_back(line);
  }
  return Map(std::move(rows));
}
bool Map::free(int c) const { return c >= 0 && c < size() && rows_[c / width_][c % width_] == '.'; }
std::vector<int> Map::neighbors(int c) const {
  std::vector<int> result;
  if (!free(c))
    return result;
  for (int v : {c - width_, c - 1, c + 1, c + width_})
    if (free(v) && std::abs(c / width_ - v / width_) + std::abs(c % width_ - v % width_) == 1)
      result.push_back(v);
  return result;
}
int Map::distance(int a, int b) const {
  return free(a) && free(b) ? distances_[a * size() + b] : unreachable;
}
std::vector<int> Map::route(int a, int b) const {
  if (distance(a, b) == unreachable)
    return {};
  std::vector<int> path{a};
  while (a != b) {
    for (int v : neighbors(a))
      if (distance(v, b) == distance(a, b) - 1) {
        a = v;
        path.push_back(a);
        break;
      }
  }
  return path;
}
int position(const Path& path, int time) {
  if (path.empty() || time < 0)
    throw std::invalid_argument("invalid path/time");
  return path[std::min(time, static_cast<int>(path.size()) - 1)];
}
}  // namespace fleet
