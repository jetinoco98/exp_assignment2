import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState
from interfaces.srv import ExecuteMove 


class TeleportManager(Node):
    '''Preliminary version of Move Action, before the integration of SLAM and NAV2'''

    def __init__(self):
        super().__init__('teleport_manager')
        
        # Create service
        self.srv = self.create_service(ExecuteMove, 'teleport_srv', self.move_callback)
        self.get_logger().info("Teleport Manager is ready.")
        
        # Create a publisher for Gazebo model state
        self.model_state_publisher = self.create_publisher(ModelState, '/gazebo/set_model_state', 10)
        

    def move_callback(self, request, response):
        # ModelState message for teleporting the robot
        model_state = ModelState()
        model_state.model_name = 'my_test_robot'
        model_state.pose.position.x = request.x
        model_state.pose.position.y = request.y
        model_state.pose.position.z = 0.0
        model_state.pose.orientation.x = 0.0
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = 0.0
        model_state.pose.orientation.w = 1.0
        model_state.reference_frame = 'link_chassis'
        
        # Publish the message
        self.model_state_publisher.publish(model_state)
        
        # Respond to the service request
        response.success = True
        response.message = f'''Succesfully teleported robot to waypoint '{request.name}' with 
                            coordinates: ({request.x}, {request.y}).'''
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TeleportManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

