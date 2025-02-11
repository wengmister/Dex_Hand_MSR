import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
import threading

class JointAngleGUI(Node):
    def __init__(self):
        super().__init__('joint_angle_gui_node')
        self.hand_subscription = self.create_subscription(
            Float32MultiArray,
            '/hand_joint_angles',
            self.joint_angle_callback,
            10)
        
        # Subscribe to /hand_servo_input (16 data floats)
        self.servo_subscription = self.create_subscription(
            Float32MultiArray,
            '/hand_servo_input',
            self.servo_angle_callback,
            10
        )
        
        # Initialize Tkinter GUI
        self.root = tk.Tk()
        self.root.title("Hand Joint Angles")
        self.labels = []
        self.joint_angles = [0.0] * 15  # Initialize joint angles with zero values
        self.servo_angles = [0.0] * 15  # Initialize servo angles with zero values
        
        # The original ordering of joint names:
        joint_names = [
            "Thumb CMC Flexion",  # 0
            "Thumb MCP Abduction",# 1
            "Thumb PIP",          # 2
            "Index MCP Flexion",  # 3
            "Index MCP Abduction",# 4
            "Index PIP",          # 5
            "Middle MCP Flexion", # 6
            "Middle MCP Abduction",#7
            "Middle PIP",         # 8
            "Ring MCP Flexion",   # 9
            "Ring MCP Abduction", # 10
            "Ring PIP",           # 11
            "Pinky MCP Flexion",  # 12
            "Pinky MCP Abduction",# 13
            "Pinky PIP"           # 14
        ]
        
        for i, name in enumerate(joint_names):
            label = tk.Label(self.root, text=f"{name}: 0.0°", font=("Arial", 14))
            label.grid(row=i, column=0, padx=10, pady=5, sticky='w')
            self.labels.append(label)
        
        self.root.after(100, self.update_gui)

    def joint_angle_callback(self, msg):
        if len(msg.data) == 15:
            self.joint_angles = msg.data
        # else:
            # self.get_logger().warn("Unexpected joint angle data size")

    def servo_angle_callback(self, msg):
        """
        Callback for /hand_servo_input with 16 floats.
        Need to reorder them back to the original 15-joint format:
        
        Incoming servo indices (16 total):
          0:  thumb_mcp_abd
          1:  index_mcp_abd
          2:  middle_mcp_abd
          3:  ring_mcp_abd
          4:  pinky_mcp_abd
          5:  index_mcp_flex
          6:  middle_mcp_flex
          7:  ring_mcp_flex
          8:  pinky_mcp_flex
          9:  thumb_pip
          10: index_pip
          11: middle_pip
          12: ring_pip
          13: pinky_pip
          14: (unused)
          15: thumb_cmc_flex
          
        We want them in original order (15 joints):
          0 -> thumb_cmc_flex
          1 -> thumb_mcp_abd
          2 -> thumb_pip
          3 -> index_mcp_flex
          4 -> index_mcp_abd
          5 -> index_pip
          6 -> middle_mcp_flex
          7 -> middle_mcp_abd
          8 -> middle_pip
          9 -> ring_mcp_flex
          10 -> ring_mcp_abd
          11 -> ring_pip
          12 -> pinky_mcp_flex
          13 -> pinky_mcp_abd
          14 -> pinky_pip
        """
        if len(msg.data) == 16:
            reordered = [0.0] * 15
            reordered[0]  = msg.data[15]  # thumb_cmc_flex
            reordered[1]  = msg.data[0]   # thumb_mcp_abd
            reordered[2]  = msg.data[9]   # thumb_pip
            reordered[3]  = msg.data[5]   # index_mcp_flex
            reordered[4]  = msg.data[1]   # index_mcp_abd
            reordered[5]  = msg.data[10]  # index_pip
            reordered[6]  = msg.data[6]   # middle_mcp_flex
            reordered[7]  = msg.data[2]   # middle_mcp_abd
            reordered[8]  = msg.data[11]  # middle_pip
            reordered[9]  = msg.data[7]   # ring_mcp_flex
            reordered[10] = msg.data[3]   # ring_mcp_abd
            reordered[11] = msg.data[12]  # ring_pip
            reordered[12] = msg.data[8]   # pinky_mcp_flex
            reordered[13] = msg.data[4]   # pinky_mcp_abd
            reordered[14] = msg.data[13]  # pinky_pip
            
            # Update our displayed angles
            self.servo_angles = reordered
        else:
            self.get_logger().debug(
                f"Received {len(msg.data)} floats instead of 16."
            )


    def update_gui(self):
        for i, angle in enumerate(self.joint_angles):
            self.labels[i].config(text=f"{self.labels[i].cget('text').split(':')[0]}: {angle:.2f}°, Servo: {self.servo_angles[i]:.2f}°")
        self.root.after(100, self.update_gui)

    def run(self):
        self.root.mainloop()

def ros_spin(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    gui_node = JointAngleGUI()
    ros_thread = threading.Thread(target=ros_spin, args=(gui_node,), daemon=True)
    ros_thread.start()
    gui_node.run()
    gui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
