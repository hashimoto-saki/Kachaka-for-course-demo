import rclpy
from rclpy.node import Node
from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import KachakaCommand
from rclpy.action import ActionClient
 
class WaypointPub(Node):
    def __init__(self):
        super().__init__("waypoint_publisher")
        self._action_client = ActionClient(self, ExecKachakaCommand,
                                           "/kachaka/kachaka_command/execute")
        self._action_client.wait_for_server()
         
    def sent_request(self, pos_x, pos_y, yaw):
        command = KachakaCommand()
        command.command_type = KachakaCommand.MOVE_TO_POSE_COMMAND
        command.move_to_pose_command_x = pos_x
        command.move_to_pose_command_y = pos_y
        command.move_to_pose_command_yaw = yaw
        goal_msg = ExecKachakaCommand.Goal()
        goal_msg.kachaka_command = command
        future = self._action_client.send_goal_async(goal_msg)
        return future

         
def main(args=None):
    rclpy.init(args=args)
    waypoint_pub = WaypointPub()
    
    x, y, yaw = 2.0, 5.0, 0.0
        
    future = waypoint_pub.sent_request(x, y, yaw) 
    rclpy.spin_until_future_complete(waypoint_pub, future)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()