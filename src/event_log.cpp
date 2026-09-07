#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "fleet/fleet.hpp"

namespace fleet {
std::string quote_json(const std::string& input) {
  std::ostringstream out;
  out << '"';
  for (unsigned char c : input) {
    if (c == '"' || c == '\\')
      out << '\\' << c;
    else if (c < 32)
      out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c)
          << std::dec;
    else
      out << c;
  }
  out << '"';
  return out.str();
}
namespace {
template <class T>
double quantile(std::vector<T> samples, double q) {
  if (samples.empty())
    return 0;
  std::sort(samples.begin(), samples.end());
  return samples[static_cast<std::size_t>(std::ceil(q * samples.size())) - 1];
}
}  // namespace
std::string result_json(const Config& c, const Result& r) {
  std::ostringstream o;
  o << std::setprecision(10);
  o << "{\"schema_version\":1,\"mode\":\"local_discrete_simulator\",\"strategy\":"
    << quote_json(to_string(c.strategy)) << ",\"seed\":" << c.seed << ",\"agents\":" << c.agents
    << ",\"ticks\":" << c.ticks << ",\"horizon\":" << c.horizon << ",\"lambda\":" << c.lambda
    << ",\"arrival_interval\":" << c.arrival_interval
    << ",\"congestion_window\":" << c.congestion_window
    << ",\"max_expansions\":" << c.budget.max_expansions
    << ",\"deadline_us\":" << c.budget.deadline_us << ",\"max_pending\":" << c.max_pending
    << ",\"offered\":" << r.offered << ",\"admitted\":" << r.admitted
    << ",\"rejected\":" << r.rejected << ",\"completed\":" << r.completed
    << ",\"unfinished\":" << r.unfinished
    << ",\"throughput_tasks_per_tick\":" << static_cast<double>(r.completed) / c.ticks
    << ",\"conflicts\":" << r.conflicts << ",\"no_progress_events\":" << r.no_progress_events
    << ",\"planner_calls\":" << r.planner_calls
    << ",\"planning_us_p50\":" << quantile(r.planning_us, .5)
    << ",\"planning_us_p95\":" << quantile(r.planning_us, .95)
    << ",\"allocation_us_p50\":" << quantile(r.allocation_us, .5)
    << ",\"allocation_us_p95\":" << quantile(r.allocation_us, .95) << ",\"completion_ticks_p50\":"
    << (r.completion_ticks.empty() ? "null" : std::to_string(quantile(r.completion_ticks, .5)))
    << ",\"completion_ticks_p95\":"
    << (r.completion_ticks.empty() ? "null" : std::to_string(quantile(r.completion_ticks, .95)))
    << ",\"plan_status_counts\":{";
  for (int i = 0; i < 5; ++i) {
    if (i)
      o << ',';
    o << quote_json(to_string(static_cast<PlanStatus>(i))) << ':' << r.plan_status_counts[i];
  }
  o << "},\"recovery_ticks\":[";
  for (std::size_t i = 0; i < r.recovery_ticks.size(); ++i) {
    if (i)
      o << ',';
    if (r.recovery_ticks[i] < 0)
      o << "null";
    else
      o << r.recovery_ticks[i];
  }
  o << "],\"fault_applied\":[";
  for (std::size_t i = 0; i < r.fault_applied.size(); ++i) {
    if (i)
      o << ',';
    o << r.fault_applied[i];
  }
  o << "],\"completion_ticks\":[";
  for (std::size_t i = 0; i < r.completion_ticks.size(); ++i) {
    if (i)
      o << ',';
    o << r.completion_ticks[i];
  }
  o << "],\"planning_us\":[";
  for (std::size_t i = 0; i < r.planning_us.size(); ++i) {
    if (i)
      o << ',';
    o << r.planning_us[i];
  }
  o << "],\"allocation_us\":[";
  for (std::size_t i = 0; i < r.allocation_us.size(); ++i) {
    if (i)
      o << ',';
    o << r.allocation_us[i];
  }
  o << "]}";
  return o.str();
}
void replay(const Map& map, const std::string& path, std::ostream& out) {
  std::ifstream in(path);
  if (!in)
    throw std::runtime_error("cannot open trace");
  std::string line;
  if (!std::getline(in, line) || line != "tick,robot,cell,state,task_index")
    throw std::invalid_argument("invalid trace header");
  std::vector<int> previous, current;
  int frame_tick = -1, last_robot = -1;
  auto finish = [&]() {
    if (current.empty())
      throw std::invalid_argument("empty trace frame");
    auto errors = check_transition(map, previous.empty() ? current : previous, current);
    if (!errors.empty())
      throw std::invalid_argument("invalid trace: " + errors.front());
    out << "tick " << frame_tick << ":";
    for (int c : current)
      out << ' ' << c;
    out << '\n';
    previous = current;
    current.clear();
    last_robot = -1;
  };
  while (std::getline(in, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    int tick, robot, cell, task;
    std::string state, extra;
    if (!(row >> tick >> robot >> cell >> state >> task) || (row >> extra) || tick < 0 ||
        robot < 0 || task < -1)
      throw std::invalid_argument("malformed trace row");
    if (state != "Idle" && state != "Assigned" && state != "Moving" && state != "Waiting" &&
        state != "Blocked" && state != "Unavailable")
      throw std::invalid_argument("unknown robot state");
    if (tick != frame_tick) {
      if (frame_tick >= 0)
        finish();
      if (tick != frame_tick + 1)
        throw std::invalid_argument("noncontiguous tick");
      frame_tick = tick;
    }
    if (robot != last_robot + 1 || robot >= 100)
      throw std::invalid_argument("noncontiguous robot IDs");
    last_robot = robot;
    current.push_back(cell);
  }
  if (frame_tick < 0)
    throw std::invalid_argument("trace contains no frames");
  finish();
}
}  // namespace fleet
