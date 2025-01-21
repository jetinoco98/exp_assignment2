#include <memory>
#include <string>
#include <iostream>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "interfaces/srv/execute_inspect.hpp"
#include "interfaces/srv/update_waypoint.hpp"

using namespace std::chrono_literals;

class InspectAction : public plansys2::ActionExecutorClient {
public:
    InspectAction() : plansys2::ActionExecutorClient("inspect", 250ms), service_called_(false)
    {
        // Service creation
        inspection_client_ = this->create_client<interfaces::srv::ExecuteInspect>("inspection_srv");
        update_waypoint_client_ = this->create_client<interfaces::srv::UpdateWaypoint>("update_waypoint");

        // Wait for services to become available
        while (!inspection_client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for 'inspection_srv' service...");
        }
        while (!update_waypoint_client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for 'update_waypoint' service...");
        }
        RCLCPP_INFO(this->get_logger(), "Services for Inspect Action are ready");
    }

private:
    void do_work() override
    {
        // PROCESS PDDL ARGUMENTS
        auto arguments = get_arguments();
        if (arguments.size() < 2) {
            finish(false, 0.0, "Invalid PDDL arguments");
            return;
        }
        waypoint_name_ = arguments[1];  // (e.g., inspect robot1 w0)

        ///////////////////////////////////////////////////////////
        // STEP 1: BEGIN THE INSPECTION
        ///////////////////////////////////////////////////////////

        if (service_called_) {return;}  // Return to avoid repeated calls

        RCLCPP_INFO(this->get_logger(), "Step 1: Start Inspection request.");

        // Service request
        auto request = std::make_shared<interfaces::srv::ExecuteInspect::Request>();
        request->start = true;

        // Asynchronously send the request
        auto future = inspection_client_->async_send_request(request,
            [this](rclcpp::Client<interfaces::srv::ExecuteInspect>::SharedFuture response) {
                auto result = response.get();
                if (result->success) {
                    RCLCPP_INFO(this->get_logger(), "%s", result->msg.c_str());
                    this->step_2(); // Proceed to Step 2
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Start inspection failed!");
                    finish(false, 0.0, "Start inspection failed");
                }
            });

        service_called_ = true; // Prevent repeated calls
    }

    ///////////////////////////////////////////////////////////
    // STEP 2: CHECK THE STATUS OF THE INSPECTION
    ///////////////////////////////////////////////////////////

    void step_2()
    {
        RCLCPP_INFO(this->get_logger(), "Step 2: Wait for inspection to detect a marker.");

        // Initialize variables for the loop
        inspection_timeout_ = std::chrono::steady_clock::now() + std::chrono::seconds(30);

        // Continously ask the service, if during the inspection, a marker was found
        check_timer_ = this->create_wall_timer(
            2s,  // Wait between each check
            [this]() {
                if (std::chrono::steady_clock::now() >= inspection_timeout_) {
                    finish(false, 0.0, "Inspection timed out");
                    check_timer_->cancel();  // Stop the timer
                    return;
                }

                // Service request
                auto request = std::make_shared<interfaces::srv::ExecuteInspect::Request>();
                request->check = true;

                // Asynchronously send the request
                auto future = inspection_client_->async_send_request(request,
                    [this](rclcpp::Client<interfaces::srv::ExecuteInspect>::SharedFuture response) {
                        auto result = response.get();
                        if (result->success) {
                            inspection_id_ = result->id;
                            check_timer_->cancel();  // Stop the timer
                            RCLCPP_INFO(this->get_logger(), "For waypoint '%s': %s", waypoint_name_.c_str(), result->msg.c_str());
                            this->step_3(); // Proceed to Step 3
                        }
                    });
            });
    }

    ///////////////////////////////////////////////////////////
    // STEP 3: UPDATE THE WAYPOINT WITH THE OBTAINED MARKER ID
    ///////////////////////////////////////////////////////////

    void step_3()
    {
        RCLCPP_INFO(this->get_logger(), "Starting Step 3: Sending update_waypoint request");

        // Service request
        auto request = std::make_shared<interfaces::srv::UpdateWaypoint::Request>();
        request->name = waypoint_name_;
        request->id = inspection_id_;

        // Asynchronously send the request
        auto future = update_waypoint_client_->async_send_request(
            request,
            [this](rclcpp::Client<interfaces::srv::UpdateWaypoint>::SharedFuture response) {
                auto result = response.get();
                if (result->success) {
                    RCLCPP_INFO(this->get_logger(), "Waypoint updated successfully: %s", result->msg.c_str());
                    finish(true, 1.0, "Inspection process completed");
                    reset();  // Reset variables after success
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Update waypoint failed: %s", result->msg.c_str());
                    finish(false, 0.0, "Update waypoint failed");
                }
            });
    }

    void reset()
    {
        // To restrict do_work()
        service_called_ = false;
        // Waypoint data
        inspection_id_ = 0;
        waypoint_name_.clear();
        // Timer data
        check_timer_ = nullptr;
        inspection_timeout_ = std::chrono::steady_clock::time_point();
    }

    // Client variables
    rclcpp::Client<interfaces::srv::ExecuteInspect>::SharedPtr inspection_client_;
    rclcpp::Client<interfaces::srv::UpdateWaypoint>::SharedPtr update_waypoint_client_;

    // Flag variables
    bool service_called_;

    // Data variables
    std::int32_t inspection_id_;
    std::string waypoint_name_;

    // Time variables
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

