import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

class HandShadowingNode(Node):
    def __init__(self):
        super().__init__('hand_shadowing_node')

        # --- Parameters ---
        # Scale factor
        self.declare_parameter('scale_factor', 2.0)
        self.scale_factor = self.get_parameter('scale_factor').value

        # Smoothing factor (alpha for exponential smoothing)
        # alpha in [0.0, 1.0], typical values ~ 0.1 - 0.5
        self.declare_parameter('smoothing_factor', 0.35)
        self.smoothing_factor = self.get_parameter('smoothing_factor').value

        # Create subscription to /hand_joint_angles
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/hand_joint_angles',
            self.joint_angles_callback,
            10
        )

        # Create publisher to /hand_servo_input
        self.publisher = self.create_publisher(Float32MultiArray, '/hand_servo_input', 10)

        # Initialize a list to store smoothed angles (one for each servo output: 16 total)
        # Start them at 0.0. You could also defer initialization until the first message arrives.
        self.smoothed_angles = [0.0]*16

        self.get_logger().info('Hand shadowing node initialized.') 

    def joint_angles_callback(self, msg):
        """
        Callback function for /hand_joint_angles subscription. Expands from 15 to 16 angles,
        then applies scaling, compensation, clamping, and smoothing before publishing.
        """
        if msg.data:
            self.get_logger().debug(f"Received joint angles: {msg.data}")

            # Prepare the output message
            out_msg = Float32MultiArray()
            out_msg.data = [0.0]*16

            # 1. Map from input (15 angles) to output (16 servo angles)
            out_msg.data[0]  = msg.data[1]   # thumb_mcp_abd
            out_msg.data[1]  = msg.data[4]   # index_mcp_abd
            out_msg.data[2]  = msg.data[7]   # middle_mcp_abd
            out_msg.data[3]  = msg.data[10]  # ring_mcp_abd
            out_msg.data[4]  = msg.data[13]  # pinky_mcp_abd

            out_msg.data[5]  = msg.data[3]   # index_mcp_flex
            out_msg.data[6]  = msg.data[6]   # middle_mcp_flex
            out_msg.data[7]  = msg.data[9]   # ring_mcp_flex
            out_msg.data[8]  = msg.data[12]  # pinky_mcp_flex

            out_msg.data[9]  = msg.data[2]   # thumb_pip
            out_msg.data[10] = msg.data[5]   # index_pip
            out_msg.data[11] = msg.data[8]   # middle_pip
            out_msg.data[12] = msg.data[11]  # ring_pip
            out_msg.data[13] = msg.data[14]  # pinky_pip

            # Index 14 unused -> remains 0.0
            out_msg.data[15] = msg.data[0]   # thumb_cmc_flex

            # 2. Apply compensation, scaling, and clamping
            for i in range(16):
                compensated_angle = self.compensate_angle_by_index(out_msg.data[i], i)
                scaled_angle = self.scale_angle_by_index(compensated_angle, i)
                clamped_angle = self.clamp_angle_by_index(scaled_angle, i)

                # 3. Exponential smoothing:
                # smoothed_angle = alpha * (new value) + (1-alpha) * (previous smoothed value)
                alpha = self.smoothing_factor
                smoothed_angle = alpha * clamped_angle + (1 - alpha) * self.smoothed_angles[i]
                self.smoothed_angles[i] = smoothed_angle

                out_msg.data[i] = smoothed_angle

            # 4. Publish the updated (smoothed) message
            self.publisher.publish(out_msg)
        else:
            self.get_logger().debug("Received empty joint angle message")

    def scale_angle_by_index(self, angle, index):
        """
        Scale the input angle by the current scale_factor.
        """
        if index == 0:
            # thumb_mcp_abd
            return angle
        elif 1 <= index <= 4:
            # other mcp_abd
            return angle * 2.0
        elif index == 5:
            # index_mcp_flex
            return angle * -5.0
        elif 6 <= index <= 8:
            # finger mcp_flex
            return angle * -5.0
        elif index == 9:
            # thumb_pip
            return angle * 5.0
        elif 10 <= index <= 13:
            # all pip
            return angle * 1.0 
        elif index == 15:
            # thumb_cmc_flex
            return angle * 10.0
        else:
            return angle * self.scale_factor
    
    def compensate_angle_by_index(self, angle, index):
        """
        Compensate for the angle based on the servo index.
        """
        if index == 0:
            # thumb_mcp_abd
            return angle - 31.0
        elif 1 <= index <= 4:
            # other mcp_abd
            return angle
        elif 5 <= index <= 8:
            # finger mcp_flex
            return angle + 8.0
        elif index == 9:
            # thumb_pip
            return angle + 25.0
        elif 10 <= index <= 13:
            # all pip
            return angle + 90.0
        elif index == 15:
            # thumb_cmc_flex
            return angle + 35.0
        else:
            # Index 14 or any unexpected index
            return angle

    def clamp_angle_by_index(self, angle, index):
        """
        Clamp angle based on the servo index:
          - Index 0–4:   clamp to ±25 degrees
          - Index 5–8:   clamp to ±90 degrees
          - Index 9–13:  clamp to -50 to +90 degrees
          - Index 14:    (unused)
          - Index 15:    clamp to ±90 degrees
        """
        if 0 <= index <= 4:
            # ±25 degrees
            return max(-25.0, min(angle, 25.0))
        elif 5 <= index <= 8:
            # ±90 degrees
            return max(-90.0, min(angle, 90.0))
        elif 9 <= index <= 13:
            # -50 to +90 degrees
            return max(-50.0, min(angle, 90.0))
        elif index == 15:
            # ±90 degrees (thumb_cmc_flex)
            return max(-90.0, min(angle, 90.0))
        else:
            # Index 14 or any unexpected index
            return max(-90.0, min(angle, 90.0))

def main(args=None):
    rclpy.init(args=args)
    hand_shadowing_node = HandShadowingNode()
    rclpy.spin(hand_shadowing_node)
    hand_shadowing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
