#include <memory>
#include <string>
#include <iostream>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/srv/get_waypoint_position.hpp" 
#include "interfaces/msg/waypoint.hpp"   

// Temporal: Used to teleport robot
#include "gazebo_msgs/srv/set_entity_state.hpp"      
#include "gazebo_msgs/msg/entity_state.hpp"         
         

using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("move", 250ms),
    service_called_(false), waypoint_obtained_(false)
  {
    // Create service clients
    get_waypoint_position_client_ = this->create_client<interfaces::srv::GetWaypointPosition>("get_waypoint_position");
    set_entity_state_client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");

    // Wait for services to become available
    while (!get_waypoint_position_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for 'get_waypoint_position' service...");
    }
    while (!set_entity_state_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for 'set_entity_state' service...");
    }
  }

private:
  void do_work() override
  {
      // Process the PDDL arguments
      auto arguments = get_arguments();
      if (arguments.size() < 3) {
        finish(false, 0.0, "Invalid PDDL arguments");
        return;
      }
      waypoint_final_name_ = arguments[2];  // (e.g. move robot1 w0 w1)

      ///////////////////////////////////
      // STEP 1: GET WAYPOINT POSITION
      ///////////////////////////////////

      if (!service_called_) {
        RCLCPP_INFO(this->get_logger(), "Starting Step 1: Sending request to get waypoint position...");

        // Service client: Get Waypoint Position
        auto request = std::make_shared<interfaces::srv::GetWaypointPosition::Request>();
        request->name = waypoint_final_name_;

        // Asynchronously send the request
        auto future = get_waypoint_position_client_->async_send_request(
          request,
          [this](rclcpp::Client<interfaces::srv::GetWaypointPosition>::SharedFuture response) {
            try {
              auto result = response.get();
              if (result->success) {
                RCLCPP_INFO(this->get_logger(), "Waypoint position obtained successfully: %s", result->msg.c_str());
                waypoint_ = result->waypoint;  // This is a waypoint object
                waypoint_obtained_ = true;
                // Proceed to Step 2
                this->step_2();
              } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to get waypoint position: %s", result->msg.c_str());
                finish(false, 0.0, "Failed to get waypoint position");
              }
            } catch (const std::exception &e) {
              RCLCPP_ERROR(this->get_logger(), "Service call threw an exception: %s", e.what());
              finish(false, 0.0, "Failed to get waypoint position");
            }
          });

        service_called_ = true; // Marking Step 1 as completed to avoid repeated calls
      }
  }

  ///////////////////////////////////
  // STEP 2: MOVE THE ROBOT
  ///////////////////////////////////

  void step_2()
  {
    if (!waypoint_obtained_) {
      RCLCPP_ERROR(this->get_logger(), "Waypoint data is not available. Cannot proceed further.");
      finish(false, 0.0, "Missing waypoint data");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Starting Step 2: Moving the robot'");

    // Service client: Gazebo set entity state
    auto state_request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    state_request->state.name = "my_test_robot";
    state_request->state.pose.position.x = waypoint_.x; 
    state_request->state.pose.position.y = waypoint_.y;  
    state_request->state.pose.position.z = waypoint_.z; 

    // Asynchronously send the request
    auto future = set_entity_state_client_->async_send_request(
      state_request,
      [this](rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedFuture response) {
        try {
          auto result = response.get();
          if (result->success) {
            finish(true, 1.0, "Move action completed");
          } else {
            finish(false, 0.0, "Failed to complete the move action");
          }
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Service call threw an exception: %s", e.what());
          finish(false, 0.0, "Error on Move Action service call");
        }
        reset(); // Reset state of variables after completion
      });
  }

  void reset()
  {
    service_called_ = false;
    waypoint_obtained_ = false;
    waypoint_ = interfaces::msg::Waypoint();
    waypoint_final_name_.clear();
  }

  rclcpp::Client<interfaces::srv::GetWaypointPosition>::SharedPtr get_waypoint_position_client_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_entity_state_client_;
  bool service_called_;
  bool waypoint_obtained_;
  interfaces::msg::Waypoint waypoint_;
  std::string waypoint_final_name_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}

