// Copyright 2026 JDinSeattle
// SPDX-License-Identifier: Apache-2.0
#include "nav2_lifecycle_manager/reconciled_lifecycle_client.hpp"

#include <algorithm>
#include <stdexcept>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

namespace nav2_lifecycle_manager {
using namespace std::chrono_literals;
using State = lifecycle_msgs::msg::State;
using Transition = lifecycle_msgs::msg::Transition;

ReconciledLifecycleClient::ReconciledLifecycleClient(const std::string& name,
                                                     rclcpp::Node::SharedPtr node, Budget budget)
    : name_(name), node_(std::move(node)), budget_(budget) {
  if (budget.rpc <= 0ms || budget.transition < budget.rpc) {
    throw std::invalid_argument("lifecycle budgets must be positive; transition >= RPC");
  }
  // Construction must not wait for discovery: otherwise the manager's own service
  // and shutdown callbacks may never become usable when a node is absent.
  group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(group_, node_->get_node_base_interface());
  change_ =
      node_->create_client<Change>(name + "/change_state", rclcpp::SystemDefaultsQoS(), group_);
  get_ = node_->create_client<Get>(name + "/get_state", rclcpp::SystemDefaultsQoS(), group_);
}

ReconciledLifecycleClient::~ReconciledLifecycleClient() { executor_.cancel(); }

bool ReconciledLifecycleClient::running(Deadline deadline) const {
  return rclcpp::ok(node_->get_node_base_interface()->get_context()) && Clock::now() < deadline;
}

void ReconciledLifecycleClient::pause(Deadline deadline) const {
  std::this_thread::sleep_until(std::min(deadline, Clock::now() + 50ms));
}

template <class Service>
typename Service::Response::SharedPtr ReconciledLifecycleClient::call(
    typename rclcpp::Client<Service>::SharedPtr client,
    typename Service::Request::SharedPtr request, Deadline deadline) {
  deadline = std::min(deadline, Clock::now() + budget_.rpc);
  while (running(deadline) && !client->service_is_ready()) {
    const auto left = std::max(Clock::duration::zero(), deadline - Clock::now());
    client->wait_for_service(std::min(left, Clock::duration(50ms)));
  }
  if (!running(deadline)) {
    return nullptr;
  }
  auto pending = client->async_send_request(request);
  try {
    const auto left = std::max(Clock::duration::zero(), deadline - Clock::now());
    if (executor_.spin_until_future_complete(pending, left) == rclcpp::FutureReturnCode::SUCCESS) {
      return pending.get();
    }
  } catch (...) {
    client->remove_pending_request(pending);
    throw;
  }
  // Bound pending-request memory, and discard late replies from this attempt.
  client->remove_pending_request(pending);
  return nullptr;
}

std::optional<uint8_t> ReconciledLifecycleClient::observe(Deadline deadline) {
  auto response = call<Get>(get_, std::make_shared<Get::Request>(), deadline);
  if (!response) {
    return std::nullopt;
  }
  return response->current_state.id;
}

uint8_t ReconciledLifecycleClient::get_state(std::chrono::seconds timeout) {
  const auto state = observe(Clock::now() + timeout);
  if (!state) {
    throw std::runtime_error(name_ + ": bounded get_state failed");
  }
  return *state;
}

std::pair<uint8_t, uint8_t> ReconciledLifecycleClient::states(uint8_t transition) {
  switch (transition) {
    case Transition::TRANSITION_CONFIGURE:
      return {State::PRIMARY_STATE_UNCONFIGURED, State::PRIMARY_STATE_INACTIVE};
    case Transition::TRANSITION_ACTIVATE:
      return {State::PRIMARY_STATE_INACTIVE, State::PRIMARY_STATE_ACTIVE};
    case Transition::TRANSITION_DEACTIVATE:
      return {State::PRIMARY_STATE_ACTIVE, State::PRIMARY_STATE_INACTIVE};
    case Transition::TRANSITION_CLEANUP:
      return {State::PRIMARY_STATE_INACTIVE, State::PRIMARY_STATE_UNCONFIGURED};
    case Transition::TRANSITION_UNCONFIGURED_SHUTDOWN:
      return {State::PRIMARY_STATE_UNCONFIGURED, State::PRIMARY_STATE_FINALIZED};
    case Transition::TRANSITION_INACTIVE_SHUTDOWN:
      return {State::PRIMARY_STATE_INACTIVE, State::PRIMARY_STATE_FINALIZED};
    case Transition::TRANSITION_ACTIVE_SHUTDOWN:
      return {State::PRIMARY_STATE_ACTIVE, State::PRIMARY_STATE_FINALIZED};
    default:
      throw std::invalid_argument("Unsupported managed lifecycle transition");
  }
}

bool ReconciledLifecycleClient::change_state(uint8_t transition) {
  const auto [source, target] = states(transition);
  const auto deadline = Clock::now() + budget_.transition;
  std::optional<uint8_t> initial;
  while (running(deadline)) {
    initial = observe(deadline);
    if (initial && *initial <= State::PRIMARY_STATE_FINALIZED) {
      break;
    }
    pause(deadline);
  }
  if (!initial || !running(deadline)) {
    RCLCPP_ERROR(node_->get_logger(), "LIFECYCLE_BOUND node=%s outcome=initial_state_timeout",
                 name_.c_str());
    return false;
  }
  if (*initial == target) {
    return true;
  }
  if (*initial != source) {
    RCLCPP_ERROR(node_->get_logger(),
                 "LIFECYCLE_BOUND node=%s outcome=unexpected_initial_state state=%u", name_.c_str(),
                 *initial);
    return false;
  }
  auto request = std::make_shared<Change::Request>();
  request->transition.id = transition;
  // At most ONE state-changing request. A source-state read after timeout cannot
  // exclude an old request still queued on the server; therefore never resend it.
  const auto response = call<Change>(change_, request, deadline);
  if (response && !response->success) {
    RCLCPP_ERROR(node_->get_logger(), "LIFECYCLE_BOUND node=%s outcome=rejected", name_.c_str());
    return false;
  }
  if (!response) {
    RCLCPP_WARN(node_->get_logger(),
                "LIFECYCLE_BOUND node=%s outcome=response_timeout reconciliation=read_only",
                name_.c_str());
  }
  while (running(deadline)) {
    const auto current = observe(deadline);
    if (current && *current == target) {
      if (!response) {
        RCLCPP_WARN(node_->get_logger(), "LIFECYCLE_BOUND node=%s outcome=reconciled target=%u",
                    name_.c_str(), target);
      }
      return true;
    }
    // An unrelated primary state is not evidence of this transition succeeding.
    if (current && *current != source && *current <= State::PRIMARY_STATE_FINALIZED) {
      break;
    }
    pause(deadline);
  }
  RCLCPP_ERROR(node_->get_logger(),
               "LIFECYCLE_BOUND node=%s outcome=unresolved no_mutation_retry=true", name_.c_str());
  return false;
}
}  // namespace nav2_lifecycle_manager
