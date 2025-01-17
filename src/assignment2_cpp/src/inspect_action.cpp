#include <memory>
#include <string>
#include <iostream>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/srv/execute_inspect.hpp"
#include "interfaces/srv/update_waypoint.hpp"

using namespace std::chrono_literals;

class InspectAction : public plansys2::ActionExecutorClient
{
public:
  InspectAction()
  : plansys2::ActionExecutorClient("inspect", 250ms),
    service_called_(false), inspection_success_(false)
  {
    // Service creation
    inspection_client_ = this->create_client<interfaces::srv::ExecuteInspect>("inspection_srv");
    update_waypoint_client_ = this->create_client<interfaces::srv::UpdateWaypoint>("update_waypoint");

    // Wait for services to become available
    while (!inspection_client_->wait_for_service(1s) || !update_waypoint_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for 'inspection_srv' or 'update_waypoint' service...");
    }
  }

private:
  void do_work() override
  {
    // Process PDDL arguments
    auto arguments = get_arguments();
    if (arguments.size() < 2) {
      finish(false, 0.0, "Invalid PDDL arguments");
      return;
    }
    waypoint_name_ = arguments[1];  // (e.g. inspect robot1 w0)

    ///////////////////////////////////
    // STEP 1: BEGIN THE INSPECTION PROCEDURE
    ///////////////////////////////////

    if (!service_called_) {
      RCLCPP_INFO(this->get_logger(), "Starting Step 1: Sending start inspection request...");
      
      // Service request
      auto start_request = std::make_shared<interfaces::srv::ExecuteInspect::Request>();
      start_request->name = "start_inspection";

      // Asynchronously send the request
      auto future = inspection_client_->async_send_request(
        start_request,
        [this](rclcpp::Client<interfaces::srv::ExecuteInspect>::SharedFuture response) {
          try {
            auto result = response.get();
            if (result->success) {
              this->step_2(); // Proceed to Step 2
            } else {
              RCLCPP_ERROR(this->get_logger(), "Start inspection failed: %s", result->msg.c_str());
              finish(false, 0.0, "Start inspection failed");
            }
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call threw an exception: %s", e.what());
            finish(false, 0.0, "Start inspection failed");
          }
        });

      service_called_ = true; // Mark Step 1 as completed to avoid repeated calls
    }
  }

  ///////////////////////////////////
  // STEP 2: CHECK THE STATUS OF THE INSPECTION
  ///////////////////////////////////

  void step_2()
  {
    RCLCPP_INFO(this->get_logger(), "Starting Step 2: Loop to check inspection status");

    // Initialize variables for the loop
    inspection_success_ = false;
    inspection_timeout_ = std::chrono::steady_clock::now() + std::chrono::seconds(30);

    // Continously ask the service, if during the inspection, a marker was found
    check_timer_ = this->create_wall_timer(
      3s,  // Wait 3 seconds between each check
      [this]() {
        if (std::chrono::steady_clock::now() >= inspection_timeout_) {
          finish(false, 0.0, "Inspection timed out");
          check_timer_->cancel();  // Stop the timer
          return;
        }

        auto check_request = std::make_shared<interfaces::srv::ExecuteInspect::Request>();
        check_request->name = "check_inspection";

        auto future = inspection_client_->async_send_request(
          check_request,
          [this](rclcpp::Client<interfaces::srv::ExecuteInspect>::SharedFuture response) {
            try {
              auto result = response.get();
              if (result->success) {
                inspection_id_ = result->id;
                inspection_success_ = true;
                check_timer_->cancel();  // Stop the timer
                send_feedback(0.7, "Step 2 completed: Inspection successful");
                this->step_3(); // Proceed to Step 3
              } 
              else {
                RCLCPP_INFO(this->get_logger(), "Inspection not complete yet, retrying...");
              }
            } catch (const std::exception &e) {
              RCLCPP_ERROR(this->get_logger(), "Service call threw an exception: %s", e.what());
            }
          });
      });
  }

  ///////////////////////////////////
  // STEP 3: UPDATE THE WAYPOINT WITH THE OBTAINED MARKER ID
  ///////////////////////////////////

  void step_3()
  {
    RCLCPP_INFO(this->get_logger(), "Starting Step 3: Sending update_waypoint request");

    // Service request
    auto update_request = std::make_shared<interfaces::srv::UpdateWaypoint::Request>();
    update_request->name = waypoint_name_;  
    update_request->id = inspection_id_;    

    // Asynchronously send the request
    auto future = update_waypoint_client_->async_send_request(
      update_request,
      [this](rclcpp::Client<interfaces::srv::UpdateWaypoint>::SharedFuture response) {
        try {
          auto result = response.get();
          if (result->success) {
            RCLCPP_INFO(this->get_logger(), "Waypoint updated successfully: %s", result->msg.c_str());
            finish(true, 1.0, "Inspection process completed");
            reset();  // Reset variables after success
          } else {
            RCLCPP_ERROR(this->get_logger(), "Update waypoint failed: %s", result->msg.c_str());
            finish(false, 0.0, "Update waypoint failed");
          }
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Service call threw an exception: %s", e.what());
          finish(false, 0.0, "Update waypoint failed");
        }
      });
  }

  void reset()
  {
    service_called_ = false;
    inspection_success_ = false;
    inspection_id_ = 0;
    waypoint_name_.clear();
    check_timer_ = nullptr;
    inspection_timeout_ = std::chrono::steady_clock::time_point();
  }

  rclcpp::Client<interfaces::srv::ExecuteInspect>::SharedPtr inspection_client_;
  rclcpp::Client<interfaces::srv::UpdateWaypoint>::SharedPtr update_waypoint_client_;
  bool service_called_;
  bool inspection_success_;
  std::int32_t inspection_id_;
  std::string waypoint_name_;
  std::chrono::steady_clock::time_point inspection_timeout_;
  rclcpp::TimerBase::SharedPtr check_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InspectAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "inspect"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}

