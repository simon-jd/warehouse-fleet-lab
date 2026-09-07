#pragma once
#include <cstdint>
#include <functional>
#include <iosfwd>
#include <string>
#include <vector>

namespace fleet {
constexpr int unreachable = 1000000;
class Map {
 public:
  explicit Map(std::vector<std::string> rows);
  static Map load(const std::string& path);
  int width() const { return width_; }
  int size() const { return static_cast<int>(rows_.size()) * width_; }
  bool free(int cell) const;
  const std::vector<int>& cells() const { return cells_; }
  const std::vector<std::string>& rows() const { return rows_; }
  std::vector<int> neighbors(int cell) const;
  int distance(int from, int to) const;
  std::vector<int> route(int from, int to) const;

 private:
  int width_;
  std::vector<std::string> rows_;
  std::vector<int> cells_, distances_;
};
using Path = std::vector<int>;  // Includes position at t=0; endpoint held thereafter.
int position(const Path& path, int time);
class Reservations {
 public:
  Reservations(int cells, int horizon, const std::vector<Path>& paths);
  bool vertex(int robot, int cell, int time) const;
  bool step(int robot, int from, int to, int time) const;
  bool hold(int robot, int cell, int from) const;
  bool replace(int robot,
               const Path& path);  // Validate before committing; failed replacement is inert.
  int horizon() const { return horizon_; }

 private:
  int cells_, horizon_;
  std::vector<Path> paths_;
  std::vector<int> owner_;
  void rebuild();
};
enum class PlanStatus { success, unreachable, window_exhausted, expansion_budget, deadline };
std::string to_string(PlanStatus value);
struct PlanResult {
  PlanStatus status;
  Path path;
  int expansions = 0;
};
struct PlanBudget {
  int max_expansions = 10000;
  int deadline_us = 0;
};
PlanResult plan(const Map& map, const Reservations& reservations, int robot, int start, int goal,
                const std::vector<int>& blocked, PlanBudget budget);
// Separate execution oracle; never consults reservations or planner state.
std::vector<std::string> check_transition(const Map& map, const std::vector<int>& before,
                                          const std::vector<int>& after,
                                          const std::vector<int>& blocked = {});
enum class TaskState { pending, assigned, in_progress, completed, cancelled };
enum class RobotState { idle, assigned, moving, waiting, blocked, unavailable };
std::string to_string(TaskState value);
std::string to_string(RobotState value);
struct Task {
  int id, pickup, delivery, created;
  TaskState state = TaskState::pending;
  int completed_at = -1;
};
struct Robot {
  int id, cell;
  int task = -1;
  RobotState state = RobotState::idle;
  Path path;
};
enum class Strategy { nearest, distance, congestion };
std::string to_string(Strategy value);
Strategy strategy_from(const std::string& value);
std::vector<int> linear_assignment(const std::vector<std::vector<double>>& costs);
std::vector<std::pair<int, int>> allocate(const Map& map, const std::vector<Robot>& robots,
                                          const std::vector<Task>& tasks, Strategy strategy,
                                          double lambda, int congestion_window,
                                          int candidate_limit);
struct Fault {
  std::string kind;
  int target, start, duration;
};
struct Config {
  int agents = 10, ticks = 200, seed = 100, horizon = 64, arrival_interval = 2, max_pending = 200;
  int congestion_window = 16, no_progress_ticks = 50;
  double lambda = 2.0;
  Strategy strategy = Strategy::nearest;
  PlanBudget budget;
  std::vector<Fault> faults;
};
struct Result {
  int offered = 0, admitted = 0, rejected = 0, completed = 0, unfinished = 0;
  int conflicts = 0, no_progress_events = 0, planner_calls = 0;
  std::vector<int> completion_ticks;
  std::vector<double> planning_us, allocation_us;
  std::vector<int> fault_applied;
  std::vector<int> recovery_ticks;  // -1 means censored at the simulation cutoff.
  std::vector<int> plan_status_counts = std::vector<int>(5, 0);
};
using EventSink = std::function<void(int, const std::string&, int, int, const std::string&)>;
using FrameSink = std::function<void(int, const std::vector<Robot>&)>;
Result simulate(const Map& map, const Config& config, EventSink event = {}, FrameSink frame = {});
std::string result_json(const Config& config, const Result& result);
std::string quote_json(const std::string& input);
void replay(const Map& map, const std::string& trace_path, std::ostream& out);
}  // namespace fleet
