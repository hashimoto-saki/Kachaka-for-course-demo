import rclpy
from rclpy.node import Node
from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import KachakaCommand
from rclpy.action import ActionClient

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self._action_client = ActionClient(self, ExecKachakaCommand, "/kachaka/kachaka_command/execute")
        self._action_client.wait_for_server()

    def send_detection_request(self, object_type):
        command = KachakaCommand()
        command.command_type = KachakaCommand.DETECT_OBJECT_COMMAND
        command.detect_object_command_type = object_type
        goal_msg = ExecKachakaCommand.Goal()
        goal_msg.kachaka_command = command
        future = self._action_client.send_goal_async(goal_msg)
        return future

    def kacha_speaker(self, text):
        command = KachakaCommand()
        command.command_type = KachakaCommand.SPEAK_COMMAND
        command.speak_command_text = text
        sp_msg = ExecKachakaCommand.Goal()
        sp_msg.kachaka_command = command
        self._action_client.send_goal_async(sp_msg)

def main(args=None):
    rclpy.init(args=args)
    detector = ObjectDetector()

    while True:
        object_type = input("検出する物体のタイプを入力してください：")
        detector.kacha_speaker(f"{object_type}を検出します。")
        future = detector.send_detection_request(object_type)
        rclpy.spin_until_future_complete(detector, future)
        
        detector.kacha_speaker(f"{object_type}の検出が完了しました。")
        
        if input("終了しますか？ [y/n] = ") == "y":
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()
