#include <memory>
#include <string>
#include <iostream>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "interfaces/msg/waypoint.hpp"  
#include "interfaces/srv/execute_move.hpp"
#include "interfaces/srv/get_waypoint_position.hpp"

using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
    MoveAction() : plansys2::ActionExecutorClient("move", 250ms), service_called_(false)
    {
        // Service creation
        move_client_ = this->create_client<interfaces::srv::ExecuteMove>("execute_move");
        get_waypoint_position_client_ = this->create_client<interfaces::srv::GetWaypointPosition>("get_waypoint_position");

        // Wait for services to become available
        while (!move_client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for 'execute_move' service...");
        }
        while (!get_waypoint_position_client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for 'get_waypoint_position' service...");
        }
        RCLCPP_INFO(this->get_logger(), "Services for Move Action are ready");
    }

private:
    void do_work() override
    {
        // Process PDDL arguments
        auto arguments = get_arguments();
        if (arguments.size() < 3) {
            finish(false, 0.0, "Invalid PDDL arguments");
            return;
        }
        waypoint_name_ = arguments[2];  // (e.g. move robot1 w0 w1)

        ///////////////////////////////////////////////////////////
        // STEP 1: GET THE WAYPOINT POSITION COORDINATES
        ///////////////////////////////////////////////////////////

        if (service_called_) {return;}  // Return to avoid repeated calls

        RCLCPP_INFO(this->get_logger(), "Step 1: Get the waypoint position.");
        
        // Service request
        auto request = std::make_shared<interfaces::srv::GetWaypointPosition::Request>();
        request->name = waypoint_name_;

        // Asynchronously send the request
        auto future = get_waypoint_position_client_->async_send_request(request,
            [this](rclcpp::Client<interfaces::srv::GetWaypointPosition>::SharedFuture response) {
                auto result = response.get();
                if (result->success) {
                    RCLCPP_INFO(this->get_logger(), "Waypoint position obtained successfully: %s", result->msg.c_str());
                    waypoint_ = result->waypoint;
                    this->step_2(); // Proceed to Step 2
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Unable to obtain the waypoint position: %s", result->msg.c_str());
                    finish(false, 0.0, "Get Waypoint Position failed");
                }
            }
        );

        service_called_ = true; // Flag to avoid repeated calls
    }

    ///////////////////////////////////////////////////////////
    // STEP 2: START THE NAVIGATION
    ///////////////////////////////////////////////////////////

    void step_2()
    {
        RCLCPP_INFO(this->get_logger(), "Step 2: Start the Navigation");

        // Service request
        auto request = std::make_shared<interfaces::srv::ExecuteMove::Request>();
        request->start = true;
        request->waypoint = waypoint_;

        // Asynchronously send the request
        auto future = move_client_->async_send_request(request,
            [this](rclcpp::Client<interfaces::srv::ExecuteMove>::SharedFuture response) {
                auto result = response.get();
                if (result->success) {
                    RCLCPP_INFO(this->get_logger(), "Navigation has started successfully");
                    this->step_3(); // Proceed to Step 3
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to start Navigation: %s", result->msg.c_str());
                    finish(false, 0.0, "Failed to start Navigation");
                }
            }
        );
    }


    ///////////////////////////////////////////////////////////
    // STEP 3: WAIT FOR NAVIGATION COMPLETION
    ///////////////////////////////////////////////////////////

    void step_3()
	{
		RCLCPP_INFO(this->get_logger(), "Step 3: Wait for navigation completion");

		// Continously ask the service, if the Navigation is completed
        check_timer_ = this->create_wall_timer(
            1s, // Wait between each check
            [this]() {
                // Service request
                auto request = std::make_shared<interfaces::srv::ExecuteMove::Request>();
                request->check = true;

                // Asynchronously send the request
                auto future = move_client_->async_send_request(
                request,
                [this](rclcpp::Client<interfaces::srv::ExecuteMove>::SharedFuture response) {
                    auto result = response.get();
                    if (result->success) {
                        check_timer_->cancel();  // Stop the timer
                        RCLCPP_INFO(this->get_logger(), "The navigation has completed successfully");
                        finish(true, 1.0, "Navigation successful");
                        reset(); // Reset state of variables after completion
                    }
                });
            }
        );
	}

    void reset()
    {
        // To restrict do_work()
        service_called_ = false;
        // Waypoint data
        waypoint_ = interfaces::msg::Waypoint();
        waypoint_name_.clear();
        // Timers
        check_timer_ = nullptr;
    }

    // Client variables
    rclcpp::Client<interfaces::srv::ExecuteMove>::SharedPtr move_client_;
    rclcpp::Client<interfaces::srv::GetWaypointPosition>::SharedPtr get_waypoint_position_client_;

    // Flag variables
    bool service_called_;

    // Data variables
    interfaces::msg::Waypoint waypoint_;
    std::string waypoint_name_;

    // Time variables
    rclcpp::TimerBase::SharedPtr check_timer_;
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

