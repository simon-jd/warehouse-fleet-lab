#include <charconv>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>

#include "fleet/fleet.hpp"

namespace {
int number(const std::string& s) {
  int x;
  auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), x);
  if (ec != std::errc{} || ptr != s.data() + s.size())
    throw std::invalid_argument("invalid integer: " + s);
  return x;
}
void usage() {
  std::cout
      << "fleet simulate --map FILE [--agents 10 --ticks 200 --seed 100 --strategy "
         "nearest|distance|congestion]\n"
         "  [--horizon 64 --lambda 2 --window 16 --max-expansions 10000 --deadline-us 0]\n"
         "  [--arrival-interval 2 --max-pending 200 --no-progress-ticks 50]\n"
         "  [--faults FILE] [--out NEW_DIRECTORY]\n"
         "fleet replay --map FILE --trace FILE\n"
         "Batch evaluation: python3 benchmarks/run.py --binary build/fleet --out NEW_DIRECTORY\n";
}
}  // namespace
int main(int argc, char** argv) {
  try {
    if (argc < 2 || std::string(argv[1]) == "--help") {
      usage();
      return 0;
    }
    std::string command = argv[1];
    if (command != "simulate" && command != "replay")
      throw std::invalid_argument("unknown command");
    std::map<std::string, std::string> opts;
    for (int i = 2; i < argc; i += 2) {
      if (i + 1 >= argc || std::string(argv[i]).rfind("--", 0) != 0 ||
          !opts.emplace(argv[i], argv[i + 1]).second)
        throw std::invalid_argument("options require unique --key value pairs");
    }
    auto take = [&](const std::string& key, const std::string& fallback) {
      auto i = opts.find(key);
      if (i == opts.end())
        return fallback;
      auto v = i->second;
      opts.erase(i);
      return v;
    };
    auto map_path = take("--map", "");
    if (map_path.empty())
      throw std::invalid_argument("--map required");
    auto map = fleet::Map::load(map_path);
    if (command == "replay") {
      auto trace = take("--trace", "");
      if (!opts.empty() || trace.empty())
        throw std::invalid_argument("replay requires --trace and no unknown options");
      fleet::replay(map, trace, std::cout);
      return 0;
    }
    fleet::Config c;
    c.agents = number(take("--agents", "10"));
    c.ticks = number(take("--ticks", "200"));
    c.seed = number(take("--seed", "100"));
    c.horizon = number(take("--horizon", "64"));
    c.arrival_interval = number(take("--arrival-interval", "2"));
    c.max_pending = number(take("--max-pending", "200"));
    c.congestion_window = number(take("--window", "16"));
    c.no_progress_ticks = number(take("--no-progress-ticks", "50"));
    auto lambda = take("--lambda", "2");
    std::size_t used = 0;
    c.lambda = std::stod(lambda, &used);
    if (used != lambda.size())
      throw std::invalid_argument("invalid lambda");
    c.strategy = fleet::strategy_from(take("--strategy", "nearest"));
    c.budget.max_expansions = number(take("--max-expansions", "10000"));
    c.budget.deadline_us = number(take("--deadline-us", "0"));
    auto faults = take("--faults", "");
    auto out = take("--out", "");
    if (!opts.empty())
      throw std::invalid_argument("unknown option: " + opts.begin()->first);
    if (!faults.empty()) {
      std::ifstream f(faults);
      if (!f)
        throw std::invalid_argument("cannot open faults file");
      std::string line;
      while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#')
          continue;
        fleet::Fault fault;
        std::string extra;
        std::istringstream row(line);
        if (!(row >> fault.kind >> fault.target >> fault.start >> fault.duration) || (row >> extra))
          throw std::invalid_argument("fault rows: kind target start duration");
        c.faults.push_back(fault);
      }
    }
    std::ofstream events, trace;
    if (!out.empty()) {
      // Never silently replace a prior experiment. Create-exclusive directory is the run boundary.
      if (!std::filesystem::create_directory(out))
        throw std::invalid_argument("output directory exists; choose a new run path");
      events.exceptions(std::ios::badbit | std::ios::failbit);
      trace.exceptions(std::ios::badbit | std::ios::failbit);
      events.open(out + "/events.jsonl");
      trace.open(out + "/trace.csv");
      trace << "tick,robot,cell,state,task_index\n";
    }
    auto result = fleet::simulate(
        map, c,
        [&](int tick, const std::string& kind, int robot, int task, const std::string& detail) {
          if (events.is_open())
            events << "{\"tick\":" << tick << ",\"kind\":" << fleet::quote_json(kind)
                   << ",\"robot\":" << robot << ",\"task\":" << task
                   << ",\"detail\":" << fleet::quote_json(detail) << "}\n";
        },
        [&](int tick, const std::vector<fleet::Robot>& robots) {
          if (trace.is_open())
            for (const auto& r : robots)
              trace << tick << ',' << r.id << ',' << r.cell << ',' << fleet::to_string(r.state)
                    << ',' << r.task << '\n';
        });
    auto json = fleet::result_json(c, result);
    if (!out.empty()) {
      events.close();
      trace.close();
      std::ofstream summary;
      summary.exceptions(std::ios::badbit | std::ios::failbit);
      summary.open(out + "/summary.json");
      summary << json << '\n';
      summary.close();
      // Completion marker is written last; incomplete runs cannot be benchmark evidence.
      std::ofstream complete;
      complete.exceptions(std::ios::badbit | std::ios::failbit);
      complete.open(out + "/COMPLETE");
      complete << "v1\n";
      complete.close();
    }
    std::cout << json << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "fleet: " << e.what() << '\n';
    return 1;
  }
}
