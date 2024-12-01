import rclpy
import random
from math import sin, cos, pi

from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates, ModelState
from rclpy.qos import QoSProfile


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
        qos_profile = QoSProfile(depth=10)
    
        self.declare_parameter('/vision/std_dev', 0.1)  # Valor padrão
        # Create publishers using a private attribute to avoid issues with setting attributes directly
        self._publishers = {
            model: self.create_publisher(ModelState, f'/vision/{self.clean_model_name(model)}', qos_profile)
            for model in self.MODELS_NAMES
        }

        # Create subscriber
        self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.callback,
            qos_profile
        )

    def clean_model_name(self, model: str) -> str:
        # Clean up model name by removing the prefix if necessary
        if model.split("_")[0] == "vss":
            model = "_".join(model.split("_")[1:])
        return model

    def apply_noise(self, data):
        # Apply noise to position data
        std_dev = self.get_parameter('/vision/std_dev').get_parameter_value().double_value
        theta = random.uniform(0, pi)
        radius = random.gauss(0, std_dev)

        if hasattr(data, 'position'):
            data.position.x += radius * cos(theta)
            data.position.y += radius * sin(theta)
        else:
            self.get_logger().warning("Data received without position attribute.")
        return data

    def callback(self, data: ModelStates) -> None:
        # Process the model states and apply noise
        
        # O restante do código

        for model, pose in zip(data.name, data.pose):
            if model in self.MODELS_NAMES:
                try:
                    index = data.name.index(model)
                    msg = ModelState(
                        model_name=model,
                        pose=self.apply_noise(pose),
                        twist=data.twist[index]
                    )
                    # Publish the modified state to the appropriate topic
                    self._publishers[model].publish(msg)
                except ValueError:
                    self.get_logger().error(f"Model {model} not found in received data names.")
                except Exception as e:
                    self.get_logger().error(f"Error processing model {model}: {e}")


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    vision_proxy_node = VisionProxyNode()

    try:
        # Spin the node to keep it alive and processing callbacks
        rclpy.spin(vision_proxy_node)
    except KeyboardInterrupt:
        # Gracefully shut down if interrupted
        vision_proxy_node.get_logger().info("Shutting down node...")
    finally:
        # Shutdown ROS 2 client library
        rclpy.shutdown()


if __name__ == "__main__":
    # Entry point for the script
    main()
