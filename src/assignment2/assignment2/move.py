import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from interfaces.srv import ExecuteMove

# Global Variables
DISTANCE_TRESHOLD = 0.3
CANCEL_TIMER_DURATION = 5.0  # Time in seconds before canceling the goal

class MultiGoalNavigator(Node):
    def __init__(self):
        super().__init__('multi_goal_navigator')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Service for starting and checking the move action
        self.srv = self.create_service(ExecuteMove, 'execute_move', self.execute_move_callback)
        self.get_logger().info("Move Manager is ready.")

        # Variables to track current goal state
        self.current_waypoint = None
        self.goal_in_progress = False
        self.goal_reached = False
        self.last_log_time = self.get_clock().now()

        # Variable needed to cancel the current goal
        self.goal_handle = None
        # Timer for canceling the goal
        self.timer = None

    def execute_move_callback(self, request, response):
        if request.start:
            # Start navigation to the provided waypoint
            self.current_waypoint = request.waypoint
            self.get_logger().info(f"Received waypoint: {self.current_waypoint.name} (ID: Unknown)")
            self.send_goal(self.current_waypoint)
            response.success = True
            response.msg = f"Navigation started to waypoint: {self.current_waypoint.name}"

        elif request.check:
            if self.goal_reached:
                response.success = True
                response.msg = f"Goal achieved for waypoint: {self.current_waypoint.name}"
            else:
                response.success = False
                response.msg = f"Goal not yet achieved for waypoint: {self.current_waypoint.name}"

        elif request.cancel:
            self.cancel_goal()
            response.success = True
            response.msg = f"Goal for waypoint '{self.current_waypoint.name}' has been cancelled."

        else:
            response.success = False
            response.msg = "Invalid request."
        return response

    def send_goal(self, waypoint):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'link_chassis'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = waypoint.x
        goal_msg.pose.pose.position.y = waypoint.y
        goal_msg.pose.pose.position.z = 0.0 

        # Default orientation
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

        self.goal_in_progress = True    
        self.goal_reached = False       # Reset goal reached flag

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal was rejected!')
            self.goal_in_progress = False
            return

        self.get_logger().info('Goal accepted!')
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):

        # To avoid infinite looping timer
        # To avoid logging the distance remaining after timer started
        if self.timer is not None:
            return  
        
        distance_to_goal = feedback_msg.feedback.distance_remaining

        # Log distance remaining every 1 second
        current_time = self.get_clock().now()
        if (current_time - self.last_log_time) > Duration(seconds=1):
            self.get_logger().info(f'Distance remaining: {distance_to_goal} meters')
            self.last_log_time = current_time

        # If the robot is close enough to the goal, start the timer
        if distance_to_goal < DISTANCE_TRESHOLD:
            self.get_logger().info(f"Waypoint is close enough!")
            self.get_logger().info(f"Starting cancellation timer of {CANCEL_TIMER_DURATION} seconds...")
            self.timer = self.create_timer(CANCEL_TIMER_DURATION, self.handle_timer_expiry)

    def result_callback(self, future):
        result = future.result()
        self.get_logger().info(f"Result status obtained: [{result.status}]")

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f"Waypoint '{self.current_waypoint.name}' successfully reached!")
            self.goal_reached = True
            self.goal_in_progress = False

        # Cancel the timer and reset variables if the result callback executes
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
            return
        
        if result.status == 6:  # CANCELLED BY NAVIGATOR
            self.get_logger().info(f"Current goal to waypoint was cancelled by navigator.")
            self.get_logger().info(f"Retrying...")
            self.send_goal(self.current_waypoint)
        else:
            self.get_logger().info(f"Retrying...")
            self.send_goal(self.current_waypoint)

    def handle_timer_expiry(self):
        if self.goal_reached:
            return
        self.get_logger().info("Timer expired! Cancelling the goal to save time. The robot is already close enough to the waypoint!")
        self.cancel_goal()

    def cancel_goal(self):
        if self.goal_in_progress and self.goal_handle is not None:
            self.get_logger().info(f"Cancelling the current goal: {self.current_waypoint.name} (ID: {self.current_waypoint.id})")
            self.cancel_goal_future = self.goal_handle.cancel_goal_async()
            self.cancel_goal_future.add_done_callback(self.cancel_goal_response_callback)
        if not self.goal_in_progress:
            self.get_logger().info("No goal is in progress, cannot cancel.")
        if self.goal_handle is None:
            self.get_logger().info("Invalid goal handle, cannot cancel.")

    def cancel_goal_response_callback(self, future):
        cancel_response  = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info(f"Goal successfully cancelled.")
            self.goal_reached = True
        else:
            self.get_logger().info(f"Failed to cancel the goal.")
        self.goal_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    navigator = MultiGoalNavigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Navigator stopped cleanly')
    except Exception as e:
        navigator.get_logger().error(f'Error: {e}')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

