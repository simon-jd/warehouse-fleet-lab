#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>

#include "fleet/fleet.hpp"
#include "rclcpp/rclcpp.hpp"
#include "warehouse_fleet_ros/srv/allocate_tasks.hpp"

class AllocatorNode : public rclcpp::Node {
 public:
  AllocatorNode()
      : Node("fleet_allocator"),
        map_(fleet::Map::load(declare_parameter<std::string>("grid_map", ""))) {
    service_ = create_service<Service>(
        "fleet/allocate", [this](const std::shared_ptr<Service::Request> req,
                                 std::shared_ptr<Service::Response> res) {
          try {
            if (req->robot_ids.empty() || req->robot_ids.size() > 3 || req->task_ids.size() > 32 ||
                req->robot_ids.size() != req->robot_cells.size() ||
                req->task_ids.size() != req->pickups.size() ||
                req->task_ids.size() != req->deliveries.size())
              throw std::invalid_argument("invalid array dimensions (1..3 robots, <=32 tasks)");
            if (!std::isfinite(req->congestion_lambda) || req->congestion_lambda < 0 ||
                req->congestion_lambda > 10000)
              throw std::invalid_argument("invalid lambda");
            auto unique = [](const auto& values) {
              return std::set<int>(values.begin(), values.end()).size() == values.size();
            };
            if (!unique(req->robot_ids) || !unique(req->robot_cells) || !unique(req->task_ids))
              throw std::invalid_argument("duplicate robot, position or task");
            std::vector<fleet::Robot> robots;
            std::vector<fleet::Task> tasks;
            for (std::size_t i = 0; i < req->robot_ids.size(); ++i) {
              if (req->robot_ids[i] < 0 || !map_.free(req->robot_cells[i]))
                throw std::invalid_argument("invalid robot or cell");
              robots.push_back({req->robot_ids[i],
                                req->robot_cells[i],
                                -1,
                                fleet::RobotState::idle,
                                {req->robot_cells[i]}});
            }
            for (std::size_t i = 0; i < req->task_ids.size(); ++i) {
              if (req->task_ids[i] < 0 || !map_.free(req->pickups[i]) ||
                  !map_.free(req->deliveries[i]))
                throw std::invalid_argument("invalid task or endpoint");
              tasks.push_back({req->task_ids[i], req->pickups[i], req->deliveries[i], 0});
            }
            auto assignments =
                fleet::allocate(map_, robots, tasks, fleet::strategy_from(req->strategy),
                                req->congestion_lambda, 16, 32);
            for (auto [r, t] : assignments) {
              warehouse_fleet_ros::msg::Assignment a;
              a.robot_id = robots[r].id;
              a.task_id = tasks[t].id;
              // Reference routes are geometric guides only. Continuous execution uses a
              // feedback-confirmed transit lock.
              auto route = [&](int from, int to) {
                fleet::Reservations reservations(map_.size(), 64, {{from}});
                auto result =
                    fleet::plan(map_, reservations, 0, from, to, {}, fleet::PlanBudget{2000, 0});
                if (result.status != fleet::PlanStatus::success)
                  throw std::runtime_error("reference route search failed");
                return result.path;
              };
              a.pickup_route = route(robots[r].cell, tasks[t].pickup);
              a.delivery_route = route(tasks[t].pickup, tasks[t].delivery);
              a.estimated_grid_steps =
                  static_cast<int>(a.pickup_route.size() + a.delivery_route.size()) - 2;
              res->assignments.push_back(a);
            }
            res->accepted = true;
          } catch (const std::exception& e) {
            res->accepted = false;
            res->error = e.what();
            res->assignments.clear();
          }
        });
  }

 private:
  using Service = warehouse_fleet_ros::srv::AllocateTasks;
  fleet::Map map_;
  rclcpp::Service<Service>::SharedPtr service_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AllocatorNode>());
  rclcpp::shutdown();
}
