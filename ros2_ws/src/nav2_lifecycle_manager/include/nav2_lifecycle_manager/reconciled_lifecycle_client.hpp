// Copyright 2026 JDinSeattle
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_LIFECYCLE_MANAGER__RECONCILED_LIFECYCLE_CLIENT_HPP_
#define NAV2_LIFECYCLE_MANAGER__RECONCILED_LIFECYCLE_CLIENT_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_lifecycle_manager {
// One serialized lifecycle owner per node. Timeouts never authorize mutation replay.
class ReconciledLifecycleClient {
 public:
  struct Budget {
    std::chrono::milliseconds rpc{1000};
    std::chrono::milliseconds transition{15000};
  };

  ReconciledLifecycleClient(const std::string& name, rclcpp::Node::SharedPtr node, Budget budget);
  ~ReconciledLifecycleClient();
  bool change_state(uint8_t transition);
  uint8_t get_state(std::chrono::seconds timeout = std::chrono::seconds(2));

 private:
  using Clock = std::chrono::steady_clock;
  using Deadline = Clock::time_point;
  using Change = lifecycle_msgs::srv::ChangeState;
  using Get = lifecycle_msgs::srv::GetState;
  template <class Service>
  typename Service::Response::SharedPtr call(typename rclcpp::Client<Service>::SharedPtr client,
                                             typename Service::Request::SharedPtr request,
                                             Deadline deadline);
  std::optional<uint8_t> observe(Deadline deadline);
  bool running(Deadline deadline) const;
  void pause(Deadline deadline) const;
  static std::pair<uint8_t, uint8_t> states(uint8_t transition);

  std::string name_;
  rclcpp::Node::SharedPtr node_;
  Budget budget_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<Change>::SharedPtr change_;
  rclcpp::Client<Get>::SharedPtr get_;
};
}  // namespace nav2_lifecycle_manager
#endif
