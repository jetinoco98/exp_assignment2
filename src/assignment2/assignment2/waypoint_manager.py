import rclpy
from rclpy.node import Node
from interfaces.srv import GetWaypointPosition, UpdateWaypoint
from interfaces.msg import Waypoint


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        self.waypoints = {
            "w_start": Waypoint(name="w_start", id=0, x=0.0, y=0.0, z=0.0),
            "w0": Waypoint(name="w0", id=0, x=-7.0, y=1.5, z=0.0),
            "w1": Waypoint(name="w1", id=0, x=-3.0, y=-8.0, z=0.0),
            "w2": Waypoint(name="w2", id=0, x=6.0, y=2.0, z=0.0),
            "w3": Waypoint(name="w3", id=0, x=7.0, y=-5.0, z=0.0),
            "w_end": Waypoint(name="w_end", id=0, x=0.0, y=0.0, z=0.0), 
        }

        # Create Service: Waypoint Position -> used by Move Action
        self.get_position_service = self.create_service(
            GetWaypointPosition, 'get_waypoint_position', self.get_waypoint_position_callback
        )

        # Create Service: Update Waypoint ID -> used by Inspection Action
        self.update_id_service = self.create_service(
            UpdateWaypoint, 'update_waypoint', self.update_waypoint_id_callback
        )

        self.get_logger().info("Waypoint Manager is ready.")

    def update_w_end(self):
        """Dynamically update 'w_end' to the waypoint with the lowest ID"""

        waypoints_with_ids = [wp for wp in self.waypoints.values() if wp.id != 0]
        if not waypoints_with_ids:
            return  

        lowest_waypoint = min(waypoints_with_ids, key=lambda wp: wp.id)

        # Proceed only if the ID changes
        if self.waypoints["w_end"].id == lowest_waypoint.id:
            return

        # Update 'w_end' with the values of the waypoint with the lowest ID
        self.waypoints["w_end"].id = lowest_waypoint.id
        self.waypoints["w_end"].x = lowest_waypoint.x
        self.waypoints["w_end"].y = lowest_waypoint.y
        self.waypoints["w_end"].z = lowest_waypoint.z

        # Log the change
        self.get_logger().info(f"Waypoint 'w_end' updated to match waypoint [{lowest_waypoint.name}] with ID:{lowest_waypoint.id}.")


    def get_waypoint_position_callback(self, request, response):
        # Update 'w_end' before processing the request.
        self.update_w_end()
        # Look up waypoint on the dictionary.
        found_waypoint = self.waypoints.get(request.name)

        if found_waypoint:
            response.waypoint = found_waypoint
            response.success = True
            response.msg = f"Waypoint [{request.name}] found."
        else:
            response.success = False
            response.msg = f"Waypoint [{request.name}] NOT found."
        return response

    def update_waypoint_id_callback(self, request, response):
        # Update waypoint ID if it exists
        waypoint = self.waypoints.get(request.name)
        if waypoint:
            waypoint.id = request.id
            self.waypoints[request.name] = waypoint
            response.success = True
            response.msg = f"Waypoint ID for [{request.name}] updated to ID:{request.id}."
            # Update 'w_end' after any ID change
            self.update_w_end()
        else:
            response.success = False
            response.msg = f"Waypoint [{request.name}] NOT found."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

