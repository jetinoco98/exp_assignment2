import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from interfaces.srv import ExecuteInspect
from geometry_msgs.msg import Twist


class InspectionManager(Node):
    def __init__(self):
        super().__init__('inspection_manager')

        # Create service: Obtain Waypoint Position
        self.inspection_service = self.create_service(
            ExecuteInspect, 'inspection_srv', self.inspection_callback
        )
        self.get_logger().info("Inspection Manager is ready.")
        
        # Subscription: Aruco marker detection
        self.marker_sub = self.create_subscription(
            ArucoMarkers, '/aruco_markers', self.marker_callback,10
        )
        
        # Publisher: Robot linear/angular speed
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Shared state variables
        self.last_marker_id = None

    def start_rotation(self):
        twist = Twist()
        twist.angular.z = 1.0
        self.cmd_vel_publisher.publish(twist)

    def stop_rotation(self):
        twist = Twist()
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def marker_callback(self, msg):
        '''Obtains only the first marker ID in the message'''
        if msg.marker_ids:
            self.last_marker_id = msg.marker_ids[0]

    def inspection_callback(self, request, response):
        if request.start:
            # Starts the robot rotation and removes any previously found marker
            self.start_rotation()
            response.success = True
            response.msg = "Robot has began rotating to search for markers."

            # Remove any marker id that may have been found in the way to the waypoint.
            if self.last_marker_id:
                self.last_marker_id = None

        if request.check:
            # An instant check for new markers
            if self.last_marker_id not in [None, 0]:
                self.stop_rotation()
                response.id = int(self.last_marker_id)
                response.success = True
                response.msg = f"Marker found with ID:{self.last_marker_id}"
            else:
                self.start_rotation()   
                response.success = False
                response.msg = "No markers detected yet. Waiting..."

        return response
        

def main(args=None):
    rclpy.init(args=args)
    node = InspectionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

