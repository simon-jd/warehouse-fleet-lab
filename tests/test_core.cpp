#include <algorithm>
#include <functional>
#include <iostream>
#include <numeric>
#include <random>
#include <stdexcept>

#include "fleet/fleet.hpp"

using namespace fleet;
void require(bool condition, const char* message) {
  if (!condition)
    throw std::runtime_error(message);
}
template <class F>
void rejects(F f) {
  bool caught = false;
  try {
    f();
  } catch (const std::exception&) {
    caught = true;
  }
  require(caught, "expected rejection");
}
int main() {
  int passed = 0;
  auto test = [&](const char* name, const std::function<void()>& fn) {
    try {
      fn();
      std::cout << "PASS " << name << '\n';
      ++passed;
    } catch (const std::exception& e) {
      std::cerr << "FAIL " << name << ": " << e.what() << '\n';
      throw;
    }
  };
  try {
    test("map_boundaries_and_routes", [] {
      Map m({"..#", "..."});
      require(m.distance(0, 5) == 3, "BFS");
      require(m.distance(0, 2) == unreachable, "obstacle");
      require(!m.free(-1) && !m.free(6), "bounds");
      require(m.neighbors(3) == std::vector<int>({0, 4}), "no row wrap");
      require(m.route(0, 5) == Path({0, 1, 4, 5}), "deterministic route");
      rejects([] { Map x({"..", "."}); });
    });
    test("already_at_target", [] {
      Map m({"..."});
      Reservations r(m.size(), 4, {{0}});
      auto p = plan(m, r, 0, 0, 0, {}, {});
      require(p.status == PlanStatus::success && p.path == Path({0}), "zero movement goal");
    });
    test("disconnected_is_distinct", [] {
      Map m({".#."});
      Reservations r(3, 4, {{0}});
      require(plan(m, r, 0, 0, 2, {}, {}).status == PlanStatus::unreachable, "static unreachable");
    });
    test("vertex_contention", [] {
      Map m({"..."});
      Reservations r(3, 4, {{0}, {2, 1}});
      require(plan(m, r, 0, 0, 1, {}, {}).status == PlanStatus::window_exhausted, "reserved goal");
    });
    test("opposite_edge_rejected_explicitly", [] {
      rejects([] { Reservations r(2, 4, {{0, 1}, {1, 0}}); });
      Reservations r(3, 4, {{0}, {2, 1}});
      require(!r.step(0, 1, 2, 1), "reverse-edge reservation");
    });
    test("endpoint_held_and_replacement_atomic", [] {
      Reservations r(3, 4, {{0}, {2, 1}});
      require(!r.vertex(0, 1, 4), "endpoint hold");
      require(!r.replace(0, {0, 1}), "conflicting replacement rejected");
      require(!r.vertex(1, 0, 4), "old reservation retained");
      require(r.replace(1, {2}), "safe replacement");
      require(r.vertex(0, 1, 4), "old future reservations removed");
    });
    test("goal_service_waits_for_reserved_endpoint", [] {
      Map m({"....", "...."});
      Reservations r(m.size(), 8, {{0}, {3, 2, 1, 5, 6, 7}});
      auto p = plan(m, r, 0, 0, 1, {}, {});
      require(p.status == PlanStatus::success, "detour to safe goal hold");
      require(p.path.size() > 2, "cannot stop at early goal visit");
      require(r.replace(0, p.path), "planned final hold valid");
    });
    test("planning_window_and_expansion_budget", [] {
      Map m({"....."});
      Reservations r(5, 2, {{0}});
      require(plan(m, r, 0, 0, 4, {}, {}).status == PlanStatus::window_exhausted, "finite horizon");
      require(plan(m, r, 0, 0, 4, {}, PlanBudget{0, 0}).status == PlanStatus::expansion_budget,
              "zero expansion budget");
    });
    test("wall_clock_deadline", [] {
      Map m(std::vector<std::string>(32, std::string(32, '.')));
      Reservations r(m.size(), 256, {{0}});
      auto p = plan(m, r, 0, 0, 1023, {}, PlanBudget{1000000, 1});
      require(p.status == PlanStatus::deadline, "1us budget includes setup allocation");
    });
    test("dynamic_closure_is_not_static_unreachable", [] {
      Map m({"..."});
      Reservations r(3, 8, {{0}});
      require(plan(m, r, 0, 0, 2, {1}, {}).status == PlanStatus::window_exhausted,
              "temporary blocked cell");
      require(plan(m, r, 0, 0, 2, {}, {}).status == PlanStatus::success, "closure recovery");
    });
    test("independent_collision_oracle", [] {
      Map m({"...", "..."});
      require(!check_transition(m, {0, 2}, {1, 1}).empty(), "vertex detection");
      require(!check_transition(m, {0, 1}, {1, 0}).empty(), "edge detection");
      require(!check_transition(m, {0}, {5}).empty(), "teleport detection");
      require(!check_transition(m, {0}, {1}, {1}).empty(), "closure detection");
      require(check_transition(m, {0, 2}, {3, 1}).empty(), "valid step");
    });
    test("hungarian_matches_bruteforce", [] {
      std::mt19937 rng(22);
      for (int run = 0; run < 100; ++run) {
        int n = 1 + rng() % 4, m = n + 1;
        std::vector<std::vector<double>> a(n, std::vector<double>(m));
        for (auto& row : a)
          for (auto& v : row)
            v = rng() % 20;
        auto chosen = linear_assignment(a);
        double actual = 0;
        for (int i = 0; i < n; ++i)
          actual += a[i][chosen[i]];
        std::vector<int> perm(m);
        std::iota(perm.begin(), perm.end(), 0);
        double best = 1e9;
        do {
          double sum = 0;
          for (int i = 0; i < n; ++i)
            sum += a[i][perm[i]];
          best = std::min(best, sum);
        } while (std::next_permutation(perm.begin(), perm.end()));
        require(actual == best, "optimal rectangular assignment");
      }
    });
    test("empty_queue_ties_and_dummy", [] {
      Map m({"....."});
      std::vector<Robot> robots{{0, 0, -1, RobotState::idle, {0}},
                                {1, 4, -1, RobotState::idle, {4}}};
      require(allocate(m, robots, {}, Strategy::nearest, 0, 4, 4).empty(), "empty queue");
      std::vector<Task> tasks{{7, 2, 2, 0}, {8, 2, 2, 0}};
      auto a = allocate(m, robots, tasks, Strategy::nearest, 0, 4, 4);
      require(a[0].second == 0 && a[1].second == 1, "task ID ties");
      tasks.resize(1);
      require(allocate(m, robots, tasks, Strategy::distance, 0, 4, 4).size() == 1,
              "dummy unassigned");
    });
    test("task_id_order_independent_of_storage", [] {
      Map m({"....."});
      std::vector<Robot> robots{{0, 0, -1, RobotState::idle, {0}}};
      std::vector<Task> tasks{{8, 2, 2, 0}, {7, 2, 2, 0}};
      auto a = allocate(m, robots, tasks, Strategy::nearest, 0, 4, 1);
      require(a.size() == 1 && tasks[a[0].second].id == 7,
              "lower ID selected before candidate cap");
    });
    test("never_applied_closure_is_censored", [] {
      Map m({"."});
      Config c;
      c.agents = 1;
      c.ticks = 12;
      c.faults = {{"closure", 0, 2, 3}};
      auto r = simulate(m, c);
      require(r.fault_applied[0] == 0 && r.recovery_ticks[0] == -1,
              "occupied closure never applied");
    });
    test("overlapping_faults_rejected", [] {
      Map m({"..."});
      Config c;
      c.agents = 1;
      c.ticks = 20;
      c.faults = {{"pause", 0, 2, 5}, {"pause", 0, 4, 3}};
      rejects([&] { simulate(m, c); });
    });
    test("zero_lambda_matches_distance", [] {
      Map m({"....", "...."});
      std::vector<Robot> robots{{0, 0, -1, RobotState::idle, {0}},
                                {1, 7, -1, RobotState::idle, {7}}};
      std::vector<Task> tasks{{0, 2, 4, 0}, {1, 5, 3, 0}};
      require(allocate(m, robots, tasks, Strategy::distance, 0, 4, 4) ==
                  allocate(m, robots, tasks, Strategy::congestion, 0, 4, 4),
              "ablation identity");
    });
    test("one_cell_pickup_delivery", [] {
      Map m({"."});
      Config c;
      c.agents = 1;
      c.ticks = 5;
      c.arrival_interval = 1;
      auto r = simulate(m, c);
      require(r.completed == 5 && r.unfinished == 0, "pickup and delivery processed once");
    });
    test("unavailable_recovery_and_accounting", [] {
      Map m({".....", "....."});
      Config c;
      c.agents = 1;
      c.ticks = 100;
      c.arrival_interval = 3;
      c.faults = {{"pause", 0, 5, 10}};
      int unavailable = 0, clear = 0;
      auto r = simulate(m, c, [&](int, const std::string& k, int, int, const std::string&) {
        unavailable += k == "fault_start";
        clear += k == "fault_clear";
      });
      require(unavailable == 1 && clear == 1, "fault lifecycle");
      require(r.offered == r.admitted + r.rejected && r.admitted == r.completed + r.unfinished,
              "task conservation");
      require(r.recovery_ticks[0] >= 0, "recovery observed");
      require(r.conflicts == 0, "collision free");
    });
    test("bounded_queue_and_no_progress", [] {
      Map m({"....."});
      Config c;
      c.agents = 1;
      c.ticks = 20;
      c.arrival_interval = 1;
      c.max_pending = 2;
      c.budget.max_expansions = 0;
      c.no_progress_ticks = 3;
      auto r = simulate(m, c);
      require(r.rejected > 0 && r.unfinished <= 2, "backpressure");
      require(r.no_progress_events > 0, "no progress event");
    });
    test("seeded_replay_and_randomized_safety", [] {
      Map m({"........", "...##...", "........", "........"});
      for (int seed = 0; seed < 12; ++seed) {
        Config c;
        c.seed = seed;
        c.agents = 6;
        c.ticks = 70;
        c.budget.max_expansions = 1000;
        std::vector<std::vector<int>> a, b;
        auto frames = [](auto& dest) {
          return [&dest](int, const std::vector<Robot>& robots) {
            std::vector<int> p;
            for (auto& r : robots)
              p.push_back(r.cell);
            dest.push_back(p);
          };
        };
        auto r1 = simulate(m, c, {}, frames(a));
        auto r2 = simulate(m, c, {}, frames(b));
        require(a == b && r1.completed == r2.completed, "deterministic simulation");
        for (std::size_t i = 1; i < a.size(); ++i)
          require(check_transition(m, a[i - 1], a[i]).empty(), "independent frame validity");
      }
    });
    test("invalid_config_is_rejected", [] {
      Map m({".."});
      Config c;
      c.agents = 3;
      rejects([&] { simulate(m, c); });
      c.agents = 1;
      c.horizon = 0;
      rejects([&] { simulate(m, c); });
    });
  } catch (...) {
    return 1;
  }
  std::cout << passed << " test cases passed\n";
  return 0;
}
