// Copyright 2026 JDinSeattle
// SPDX-License-Identifier: Apache-2.0
// Test-only proxy: configure the real Gazebo map server, suppress its first reply.
#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using Change = lifecycle_msgs::srv::ChangeState;
using Clock = std::chrono::steady_clock;

class ReplyProxy : public rclcpp::Node {
 public:
  ReplyProxy() : Node("lifecycle_reply_proxy") {
    forward_ = create_client<Change>("map_server/change_state_actual");
    service_ = create_service<Change>(
        "map_server/change_state",
        [this](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Change::Request> request) {
          if (header_) {
            throw std::runtime_error("Concurrent lifecycle mutation in test proxy");
          }
          header_ = header;
          request_ = request;
          deadline_ = Clock::now() + 5s;
        });
    timer_ = create_wall_timer(10ms, [this] { tick(); });
  }

 private:
  void tick() {
    if (!header_) {
      return;
    }
    if (Clock::now() >= deadline_) {
      if (pending_) {
        forward_->remove_pending_request(*pending_);
      }
      RCLCPP_ERROR(get_logger(), "FLEET_REPLY_PROXY forward_timeout");
      header_.reset();
      pending_.reset();
      return;
    }
    if (pending_ || !forward_->service_is_ready()) {
      return;
    }
    RCLCPP_WARN(get_logger(), "FLEET_REPLY_PROXY forwarded_transition=%u", request_->transition.id);
    pending_ = forward_
                   ->async_send_request(
                       request_,
                       [this](rclcpp::Client<Change>::SharedFuture future) {
                         const auto response = future.get();
                         if (request_->transition.id == 1 && response->success && !dropped_) {
                           dropped_ = true;
                           RCLCPP_WARN(get_logger(),
                                       "FLEET_REPLY_PROXY configured_real_map_server=true "
                                       "response_suppressed=true");
                         } else {
                           service_->send_response(*header_, *response);
                         }
                         header_.reset();
                         pending_.reset();
                       })
                   .request_id;
  }
  bool dropped_{false};
  Clock::time_point deadline_;
  std::optional<int64_t> pending_;
  std::shared_ptr<rmw_request_id_t> header_;
  std::shared_ptr<Change::Request> request_;
  rclcpp::Client<Change>::SharedPtr forward_;
  rclcpp::Service<Change>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReplyProxy>());
  rclcpp::shutdown();
}
