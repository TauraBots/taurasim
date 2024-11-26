import rclpy
import random
from math import sin, cos, pi

from rclpy import Node
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates, ModelState

class VisionProxyNode(Node):
    MODELS_NAMES = [
                "vss_ball",
                "yellow_team/robot_0",
                "yellow_team/robot_1",
                "yellow_team/robot_2",
                "blue_team/robot_0",
                "blue_team/robot_1",
                "blue_team/robot_2",
    ]

    def __init__(self):
        super().__init__('vision_proxy_node')
        self.publishers = {}
        for model in self.MODELS_NAMES:
            self.publishers[model] = self.create_publisher(
                ModelState, 
                '/vision/' + self.clean_model_name(model),
                qos_profile=1
            )
        # Create the subscriber
        self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.callback,
            qos_profile=1
        )

    def clean_model_name(self, model: str) -> str:
        if model.split("_")[0] == "vss":
            model = "_".join(model.split("_")[1:])
        return model

    def apply_noise(self, data):
        std_dev = self.get_parameter('/vision/std_dev').get_parameter_value().double_value
        theta = random.uniform(0, pi)
        radius = random.gauss(0, std_dev)
        data.position.x += radius * cos(theta)
        data.position.y += radius * sin(theta)
        return data

    def callback(self, data: ModelStates) -> None:
        self.get_logger().info(f'{self.get_name()} I heard {data.name}')
        for model, pose in zip(data.name, data.pose):
            if model in self.MODELS_NAMES:
                msg = ModelState(
                    model_name=model,
                    pose=self.apply_noise(pose),
                    twist=data.twist[data.name.index(model)]  # Correct twist index
                )
                self.publishers[model].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    vision_proxy_node = VisionProxyNode()
    rclpy.spin(vision_proxy_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
