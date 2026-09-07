// Copyright 2026 JDinSeattle
// SPDX-License-Identifier: Apache-2.0
// Real DDS services deliberately omit/delay replies. No mock future completions.
#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/transition.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager.hpp"
#include "nav2_lifecycle_manager/reconciled_lifecycle_client.hpp"
#include "nav2_util/lifecycle_service_client.hpp"

using namespace std::chrono_literals;
using Change = lifecycle_msgs::srv::ChangeState;
using Get = lifecycle_msgs::srv::GetState;
using Manage = nav2_msgs::srv::ManageLifecycleNodes;
using Trigger = std_srvs::srv::Trigger;
using Client = nav2_lifecycle_manager::ReconciledLifecycleClient;
using Manager = nav2_lifecycle_manager::LifecycleManager;
using Clock = std::chrono::steady_clock;

void require(bool value, const std::string& message) {
  if (!value) {
    throw std::runtime_error(message);
  }
}

class FaultServer : public rclcpp::Node {
 public:
  explicit FaultServer(std::string fault) : Node("fault_server"), mode(std::move(fault)) {
    if (mode == "already_target") {
      state = 2;
    }
    if (mode == "invalid_initial") {
      state = 4;
    }
    change = create_service<Change>(
        "target/change_state",
        [this](std::shared_ptr<rclcpp::Service<Change>> service,
               std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Change::Request> request) {
          ++calls;
          const auto from = state.load();
          const auto id = request->transition.id;
          uint8_t to = 0;
          if (id == 1 && from == 1) {
            to = 2;
          }
          if (id == 3 && from == 2) {
            to = 3;
          }
          if (id == 4 && from == 3) {
            to = 2;
          }
          if (id == 2 && from == 2) {
            to = 1;
          }
          if (id == 5 && from == 1) {
            to = 4;
          }
          if (mode == "drop_before_apply") {
            return;
          }
          if (mode == "late_apply" || mode == "apply_after_deadline") {
            later(mode == "late_apply" ? 400ms : 1700ms, [this, to] { state = to; });
            return;
          }
          auto response = std::make_shared<Change::Response>();
          response->success = to != 0 && mode != "rejected";
          if (response->success && mode != "false_success") {
            state = to;
          }
          std::cout << "FLEET_MUTATION {\"transition\":" << static_cast<int>(id)
                    << ",\"state\":" << static_cast<int>(state.load()) << "}" << std::endl;
          if (mode == "drop_after_apply") {
            return;
          }
          if (mode == "late_reply") {
            later(500ms,
                  [service, header, response] { service->send_response(*header, *response); });
          } else {
            service->send_response(*header, *response);
          }
        });
    get = create_service<Get>(
        "target/get_state",
        [this](std::shared_ptr<rclcpp::Service<Get>> service,
               std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Get::Request>) {
          ++reads;
          if (mode == "get_silent" || (mode == "first_get_lost" && reads == 1)) {
            return;
          }
          Get::Response response;
          response.current_state.id = state;
          service->send_response(*header, response);
        });
    active = create_service<Trigger>(
        "target/is_active",
        [this](std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
          response->success = mode != "manager_inactive";
        });
  }
  void later(std::chrono::milliseconds delay, std::function<void()> action) {
    auto done = std::make_shared<bool>(false);
    const auto at = Clock::now() + delay;
    timers.push_back(create_wall_timer(10ms, [done, at, action] {
      if (!*done && Clock::now() >= at) {
        *done = true;
        action();
      }
    }));
  }
  std::string mode;
  std::atomic<uint8_t> state{1};
  std::atomic<int> calls{0}, reads{0};
  rclcpp::Service<Change>::SharedPtr change;
  rclcpp::Service<Get>::SharedPtr get;
  rclcpp::Service<Trigger>::SharedPtr active;
  std::vector<rclcpp::TimerBase::SharedPtr> timers;
};

template <class Service>
typename Service::Response::SharedPtr invoke(rclcpp::Node::SharedPtr node, std::string name,
                                             typename Service::Request::SharedPtr request) {
  auto client = node->create_client<Service>(name);
  require(client->wait_for_service(5s), "test service unavailable: " + name);
  auto pending = client->async_send_request(request);
  require(
      rclcpp::spin_until_future_complete(node, pending, 6s) == rclcpp::FutureReturnCode::SUCCESS,
      "test service response missing: " + name);
  return pending.get();
}

int main(int argc, char** argv) {
  if (argc < 2) {
    return 2;
  }
  const std::string name = argv[1];
  rclcpp::init(argc, argv);
  auto test_node = std::make_shared<rclcpp::Node>("lifecycle_test_client");
  const bool manager_case = name.rfind("manager_", 0) == 0;
  const bool server_case = name.rfind("server_", 0) == 0;
  const bool baseline_case = name.rfind("baseline_", 0) == 0;
  const auto mode = manager_case    ? name.substr(8)
                    : baseline_case ? name.substr(9)
                    : server_case   ? name.substr(7)
                                    : name;
  auto server = std::make_shared<FaultServer>(mode);
  if (mode == "absent") {
    server->change.reset();
    server->get.reset();
    server->active.reset();
  }
  rclcpp::executors::SingleThreadedExecutor server_executor;
  if (mode != "absent") {
    server_executor.add_node(server);
  }
  std::thread server_thread([&] { server_executor.spin(); });
  rclcpp::executors::SingleThreadedExecutor manager_executor;
  std::shared_ptr<Manager> manager;
  std::thread manager_thread;
  int result = 0;
  double elapsed = 0;
  try {
    if (server_case) {
      while (rclcpp::ok()) {
        std::this_thread::sleep_for(30ms);
      }
    } else if (baseline_case) {
      // The exact unmodified Nav2 1.3.12 call used by the old manager. The
      // external harness kills this isolated probe if a missing reply hangs it.
      nav2_util::LifecycleServiceClient client("target", test_node);
      const auto start = Clock::now();
      require(client.change_state(1), "baseline nominal transition failed");
      elapsed = std::chrono::duration<double>(Clock::now() - start).count();
      require(mode == "nominal", "baseline unexpectedly completed suppressed reply");
    } else if (manager_case) {
      rclcpp::NodeOptions options;
      options.parameter_overrides(
          {rclcpp::Parameter("node_names", std::vector<std::string>{"target"}),
           rclcpp::Parameter("autostart", false), rclcpp::Parameter("bond_timeout", 0.0),
           rclcpp::Parameter("lifecycle_rpc_timeout", 0.15),
           rclcpp::Parameter("lifecycle_transition_timeout", 1.2)});
      manager = std::make_shared<Manager>(options);
      manager_executor.add_node(manager);
      manager_thread = std::thread([&] { manager_executor.spin(); });
      auto request = std::make_shared<Manage::Request>();
      request->command = Manage::Request::STARTUP;
      const auto start = Clock::now();
      auto response = invoke<Manage>(test_node, "/lifecycle_manager/manage_nodes", request);
      elapsed = std::chrono::duration<double>(Clock::now() - start).count();
      const bool expected = mode == "nominal" || mode == "drop_after_apply";
      require(response->success == expected, "manager startup result");
      auto active = invoke<Trigger>(test_node, "/lifecycle_manager/is_active",
                                    std::make_shared<Trigger::Request>());
      require(active->success == expected, "manager falsely reports active or stopped responding");
      require(server->calls == (expected           ? 2
                                : mode == "absent" ? 0
                                                   : 1),
              "manager duplicated mutation");
      require(elapsed < 3.5, "manager exceeded bounded startup envelope");
    } else {
      Client client("target", test_node, {150ms, 1200ms});
      // Discovery warm-up avoids turning behavior-specific tests into discovery tests.
      if (mode != "absent") {
        auto probe = test_node->create_client<Get>("target/get_state");
        require(probe->wait_for_service(5s), "fixture failed to discover");
      }
      const auto start = Clock::now();
      const bool ok = client.change_state(1);
      elapsed = std::chrono::duration<double>(Clock::now() - start).count();
      const bool expected = mode == "nominal" || mode == "drop_after_apply" ||
                            mode == "late_apply" || mode == "late_reply" ||
                            mode == "already_target" || mode == "first_get_lost";
      require(ok == expected, "client outcome: " + mode);
      const int expected_calls = mode == "get_silent" || mode == "absent" ||
                                         mode == "already_target" || mode == "invalid_initial"
                                     ? 0
                                     : 1;
      require(server->calls == expected_calls,
              "state-changing request duplicated or sent before state known");
      require(elapsed < 1.8, "transition exceeded steady-clock budget");
      if (mode == "late_reply") {
        require(client.change_state(3), "late configure reply corrupted activation");
        std::this_thread::sleep_for(650ms);
        require(client.get_state() == 3 && server->calls == 2, "late reply changed final outcome");
      }
      if (mode == "apply_after_deadline") {
        std::this_thread::sleep_for(700ms);
        require(server->state == 2 && server->calls == 1,
                "late mutation fixture / no-replay invariant");
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "FAIL " << name << ": " << e.what() << '\n';
    result = 1;
  }
  manager_executor.cancel();
  if (manager_thread.joinable()) {
    manager_thread.join();
  }
  // Keep manager alive through its registered preshutdown callback.
  rclcpp::shutdown();
  server_executor.cancel();
  server_thread.join();
  std::cout << "FLEET_TEST {\"case\":\"" << name
            << "\",\"passed\":" << (result == 0 ? "true" : "false")
            << ",\"elapsed_seconds\":" << elapsed << ",\"mutation_requests\":" << server->calls
            << ",\"state_reads\":" << server->reads << "}\n";
  return result;
}
