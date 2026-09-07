#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <random>
#include <stdexcept>

#include "fleet/fleet.hpp"

namespace fleet {
std::string to_string(TaskState s) {
  switch (s) {
    case TaskState::pending:
      return "Pending";
    case TaskState::assigned:
      return "Assigned";
    case TaskState::in_progress:
      return "InProgress";
    case TaskState::completed:
      return "Completed";
    case TaskState::cancelled:
      return "Cancelled";
  }
  throw std::logic_error("unknown task state");
}
std::string to_string(RobotState s) {
  switch (s) {
    case RobotState::idle:
      return "Idle";
    case RobotState::assigned:
      return "Assigned";
    case RobotState::moving:
      return "Moving";
    case RobotState::waiting:
      return "Waiting";
    case RobotState::blocked:
      return "Blocked";
    case RobotState::unavailable:
      return "Unavailable";
  }
  throw std::logic_error("unknown robot state");
}
Result simulate(const Map& map, const Config& cfg, EventSink event, FrameSink frame) {
  if (cfg.agents < 1 || cfg.agents > 100 || cfg.agents > static_cast<int>(map.cells().size()) ||
      cfg.ticks < 1 || cfg.ticks > 100000 || cfg.horizon < 1 || cfg.horizon > 256 ||
      cfg.arrival_interval < 1 || cfg.max_pending < 1 || cfg.max_pending > 10000 ||
      cfg.no_progress_ticks < 1 || cfg.congestion_window < 1 || cfg.congestion_window > 256 ||
      !std::isfinite(cfg.lambda) || cfg.lambda < 0 || cfg.lambda > 10000 ||
      cfg.budget.max_expansions < 0 || cfg.budget.max_expansions > 1000000 ||
      cfg.budget.deadline_us < 0)
    throw std::invalid_argument("configuration outside resource limits");
  for (const auto& f : cfg.faults)
    if ((f.kind != "pause" && f.kind != "closure") || f.start < 0 || f.duration < 1 ||
        f.start > cfg.ticks || f.duration > cfg.ticks ||
        (f.kind == "pause" ? f.target < 0 || f.target >= cfg.agents : !map.free(f.target)))
      throw std::invalid_argument("invalid fault");
  if (cfg.faults.size() > 64)
    throw std::invalid_argument("at most 64 faults");
  for (std::size_t i = 0; i < cfg.faults.size(); ++i)
    for (std::size_t j = 0; j < i; ++j) {
      const auto& a = cfg.faults[i];
      const auto& b = cfg.faults[j];
      if (a.kind == b.kind && a.target == b.target && a.start < b.start + b.duration &&
          b.start < a.start + a.duration)
        throw std::invalid_argument("overlapping faults for the same target");
    }
  if (!event)
    event = [](int, const std::string&, int, int, const std::string&) {};
  if (!frame)
    frame = [](int, const std::vector<Robot>&) {};
  std::mt19937 rng(static_cast<std::uint32_t>(cfg.seed));
  auto initial = map.cells();
  // Explicit Fisher-Yates and modulo sampling avoid std::shuffle/distribution library differences.
  for (std::size_t i = initial.size(); i > 1; --i)
    std::swap(initial[i - 1], initial[rng() % i]);
  std::vector<Robot> robots;
  for (int i = 0; i < cfg.agents; ++i)
    robots.push_back({i, initial[i], -1, RobotState::idle, {initial[i]}});
  std::vector<Task> tasks;
  Result result;
  int last_progress = 0;
  bool alerted = false;
  std::vector<int> active(cfg.faults.size(), 0), ended(cfg.faults.size(), -1);
  result.recovery_ticks.assign(cfg.faults.size(), -1);
  result.fault_applied.assign(cfg.faults.size(), 0);
  auto set_robot = [&](int tick, Robot& r, RobotState next) {
    if (r.state != next) {
      event(tick, "robot_state", r.id, r.task < 0 ? -1 : tasks[r.task].id,
            to_string(r.state) + "->" + to_string(next));
      r.state = next;
    }
  };
  auto set_task = [&](int tick, Task& t, TaskState next, int robot) {
    event(tick, "task_state", robot, t.id, to_string(t.state) + "->" + to_string(next));
    t.state = next;
  };
  auto progress = [&](int tick) {
    last_progress = tick;
    alerted = false;
  };
  frame(0, robots);
  for (int tick = 0; tick < cfg.ticks; ++tick) {
    bool invalidate = false;
    for (std::size_t f = 0; f < cfg.faults.size(); ++f) {
      const auto& fault = cfg.faults[f];
      if (tick == fault.start + fault.duration) {
        if (active[f]) {
          active[f] = 0;
          invalidate = true;
          event(tick, "fault_clear", fault.target, -1, fault.kind);
          ended[f] = tick;  // Never-applied closures have no recovery observation.
        }
      }
      if (tick >= fault.start && tick < fault.start + fault.duration && !active[f]) {
        bool occupied = fault.kind == "closure" &&
                        std::any_of(robots.begin(), robots.end(),
                                    [&](const auto& r) { return r.cell == fault.target; });
        if (!occupied) {
          active[f] = 1;
          result.fault_applied[f] = 1;
          invalidate = true;
          event(tick, "fault_start", fault.target, -1, fault.kind);
        } else if (tick == fault.start)
          event(tick, "fault_deferred", fault.target, -1, "closure_cell_occupied");
      }
    }
    std::vector<int> blocked, paused(cfg.agents, 0);
    for (std::size_t i = 0; i < active.size(); ++i)
      if (active[i]) {
        if (cfg.faults[i].kind == "closure")
          blocked.push_back(cfg.faults[i].target);
        else
          paused[cfg.faults[i].target] = 1;
      }
    if (invalidate) {
      // Faults are injected only at a synchronous tick barrier. Explicitly revoke ALL future plans.
      event(tick, "plan_epoch_reset", -1, -1, "synchronous_fault_barrier");
      for (auto& r : robots)
        r.path = {r.cell};
    }
    for (auto& r : robots) {
      if (paused[r.id]) {
        if (r.task >= 0 && tasks[r.task].state == TaskState::assigned) {
          set_task(tick, tasks[r.task], TaskState::pending, r.id);
          r.task = -1;
          event(tick, "task_requeued", r.id, -1, "unpicked_task");
        }
        set_robot(tick, r, RobotState::unavailable);
      } else if (r.state == RobotState::unavailable)
        set_robot(tick, r, r.task >= 0 ? RobotState::assigned : RobotState::idle);
    }
    // Generate offers independently of policy, including rejected offers, for paired workloads.
    if (tick % cfg.arrival_interval == 0) {
      int p = map.cells()[rng() % map.cells().size()], d = map.cells()[rng() % map.cells().size()];
      int id = result.offered++;
      event(tick, "task_offer", -1, id, std::to_string(p) + ":" + std::to_string(d));
      int outstanding = static_cast<int>(tasks.size()) - result.completed;
      if (outstanding < cfg.max_pending) {
        tasks.push_back({id, p, d, tick});
        ++result.admitted;
        event(tick, "task_admitted", -1, id, "Pending");
      } else {
        ++result.rejected;
        event(tick, "task_rejected", -1, id, "queue_capacity");
      }
    }
    auto alloc_start = std::chrono::steady_clock::now();
    auto assignments = allocate(map, robots, tasks, cfg.strategy, cfg.lambda, cfg.congestion_window,
                                2 * cfg.agents);
    result.allocation_us.push_back(
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - alloc_start)
            .count());
    for (auto [r, t] : assignments) {
      robots[r].task = t;
      set_task(tick, tasks[t], TaskState::assigned, r);
      set_robot(tick, robots[r], RobotState::assigned);
    }
    // Advance zero-distance pickup/delivery immediately, and only once per task.
    auto arrive = [&](int time, Robot& r) {
      if (r.task < 0 || paused[r.id] || r.path.size() > 1)
        return;
      auto& t = tasks[r.task];
      if (t.state == TaskState::assigned && r.cell == t.pickup) {
        set_task(time, t, TaskState::in_progress, r.id);
        progress(time);
        r.path = {r.cell};
      }
      if (t.state == TaskState::in_progress && r.cell == t.delivery) {
        set_task(time, t, TaskState::completed, r.id);
        t.completed_at = time;
        ++result.completed;
        result.completion_ticks.push_back(time - t.created);
        r.task = -1;
        r.path = {r.cell};
        set_robot(time, r, RobotState::idle);
        progress(time);
        for (std::size_t f = 0; f < ended.size(); ++f)
          if (ended[f] >= 0 && result.recovery_ticks[f] < 0)
            result.recovery_ticks[f] = time - ended[f];
      }
    };
    for (auto& r : robots)
      arrive(tick, r);
    std::vector<Path> paths;
    for (const auto& r : robots)
      paths.push_back(r.path);
    Reservations reservations(map.size(), cfg.horizon, paths);
    for (auto& r : robots) {
      if (r.task < 0 || paused[r.id] || r.path.size() > 1)
        continue;
      const auto& task = tasks[r.task];
      int goal = task.state == TaskState::assigned ? task.pickup : task.delivery;
      auto began = std::chrono::steady_clock::now();
      auto planned = plan(map, reservations, r.id, r.cell, goal, blocked, cfg.budget);
      result.planning_us.push_back(
          std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - began)
              .count());
      ++result.planner_calls;
      ++result.plan_status_counts[static_cast<int>(planned.status)];
      if (planned.status == PlanStatus::success) {
        if (!reservations.replace(r.id, planned.path))
          throw std::logic_error("planner produced conflicting replacement");
        r.path = std::move(planned.path);
        set_robot(tick, r, RobotState::assigned);
      } else {
        set_robot(tick, r, RobotState::blocked);
        event(tick, "plan_failed", r.id, task.id, to_string(planned.status));
      }
    }
    std::vector<int> before, after;
    for (auto& r : robots) {
      before.push_back(r.cell);
      after.push_back(position(r.path, 1));
    }
    auto errors = check_transition(map, before, after, blocked);
    if (!errors.empty()) {
      result.conflicts += static_cast<int>(errors.size());
      event(tick, "execution_rejected", -1, -1, errors.front());
      throw std::logic_error("independent execution checker rejected transition");
    }
    for (auto& r : robots) {
      int old = r.cell;
      r.cell = after[r.id];
      if (r.path.size() > 1)
        r.path.erase(r.path.begin());
      if (!paused[r.id] && r.task >= 0 && r.state != RobotState::blocked)
        set_robot(tick + 1, r, old == r.cell ? RobotState::waiting : RobotState::moving);
      arrive(tick + 1, r);
    }
    frame(tick + 1, robots);
    if (static_cast<int>(tasks.size()) > result.completed &&
        tick + 1 - last_progress >= cfg.no_progress_ticks && !alerted) {
      ++result.no_progress_events;
      alerted = true;
      event(tick + 1, "no_progress", -1, -1, "not_a_deadlock_proof");
    }
  }
  result.unfinished = result.admitted - result.completed;
  if (result.offered != result.admitted + result.rejected ||
      result.admitted != result.completed + result.unfinished)
    throw std::logic_error("task accounting violation");
  return result;
}
}  // namespace fleet
