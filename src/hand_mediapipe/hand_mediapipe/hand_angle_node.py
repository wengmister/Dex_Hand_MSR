import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2
import mediapipe as mp
import numpy as np
from cv_bridge import CvBridge

def calculate_angle(a, b, c):
    """
    Calculate the angle between vectors formed by three points a, b, and c.
    """
    ba = np.array(a) - np.array(b)
    bc = np.array(c) - np.array(b)
    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.degrees(np.arccos(np.clip(cosine_angle, -1.0, 1.0)))

    return angle

def neutralize_angle(angle):
    """
    Normalize input angle to 0
    """
    return angle - 180

def project_vector_onto_plane(v, normal):
    """
    Project vector v onto a plane defined by the normal vector.
    """
    return v - np.dot(v, normal) * normal / np.linalg.norm(normal) ** 2

def calculate_mcp_felxion(wrist, mcp, pip, palm_normal):
    """
    Calculate MCP flexion by projecting motion onto the plane formed by wrist, MCP, and PIP.
    """
    v1 = np.array(mcp) - np.array(wrist)
    v2 = np.array(pip) - np.array(wrist)
    normal = np.cross(v1, v2)
    normal /= np.linalg.norm(normal)
    
    mcp_pip_vector = np.array(pip) - np.array(mcp)
    projected_vector = project_vector_onto_plane(mcp_pip_vector, normal)
    reference_vector = project_vector_onto_plane(v1, normal)
    
    cosine_angle = np.dot(projected_vector, reference_vector) / (np.linalg.norm(projected_vector) * np.linalg.norm(reference_vector))
    angle = np.degrees(np.arccos(np.clip(cosine_angle, -1.0, 1.0)))
    
    # Ensure flexion is positive
    cross_product = np.cross(reference_vector, projected_vector)
    if np.dot(cross_product, palm_normal) < 0:
        angle = -angle
    
    return angle

def calculate_abduction(wrist, mcp, reference_mcp, pip):
    """
    Calculate MCP adduction/abduction by projecting motion onto the plane formed by wrist, MCP, and reference MCP.
    """
    v1 = np.array(mcp) - np.array(wrist)
    v2 = np.array(reference_mcp) - np.array(wrist)
    normal = np.cross(v1, v2)
    normal /= np.linalg.norm(normal)
    
    mcp_pip_vector = np.array(pip) - np.array(mcp)
    projected_vector = project_vector_onto_plane(mcp_pip_vector, normal)
    reference_vector = project_vector_onto_plane(v1, normal)
    
    cosine_angle = np.dot(projected_vector, reference_vector) / (np.linalg.norm(projected_vector) * np.linalg.norm(reference_vector))
    angle = np.degrees(np.arccos(np.clip(cosine_angle, -1.0, 1.0)))
    
    # Ensure adduction is positive and abduction is negative
    cross_product = np.cross(reference_vector, projected_vector)
    if np.dot(cross_product, normal) < 0:
        angle = -angle
    
    return angle

class HandAngleNode(Node):
    def __init__(self):
        super().__init__('hand_angle_node')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Float32MultiArray, '/hand_joint_angles', 10)
        self.bridge = CvBridge()
        self.mp_hands = mp.solutions.hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.8)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.mp_hands.process(image_rgb)
        joint_angles = Float32MultiArray()

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                landmarks = [(lm.x, lm.y, lm.z) for lm in hand_landmarks.landmark]
                
                # Draw landmarks on the image
                for lm in hand_landmarks.landmark:
                    h, w, _ = image.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
                
                # Draw red lines connecting fingers
                finger_connections = [(0,1), (0,5), (0,9), (0,13), (0,17),  # Palm to fingers
                                      (1, 2), (2, 3), (3, 4),   # Thumb
                                      (5, 6), (6, 7), (7, 8),   # Index finger
                                      (9, 10), (10, 11), (11, 12),  # Middle finger
                                      (13, 14), (14, 15), (15, 16),  # Ring finger
                                      (17, 18), (18, 19), (19, 20)]  # Pinky finger
                
                for connection in finger_connections:
                    x1, y1 = int(landmarks[connection[0]][0] * w), int(landmarks[connection[0]][1] * h)
                    x2, y2 = int(landmarks[connection[1]][0] * w), int(landmarks[connection[1]][1] * h)
                    cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                
                # Compute angles for all five fingers (thumb first)
                wrist = landmarks[0]
                # reference_mcps = [landmarks[5], landmarks[9], landmarks[13], landmarks[17]]
                finger_mcps = [landmarks[1], landmarks[5], landmarks[9], landmarks[13], landmarks[17]]
                finger_pips = [landmarks[2], landmarks[6], landmarks[10], landmarks[14], landmarks[18]]
                finger_dips = [landmarks[3], landmarks[7], landmarks[11], landmarks[15], landmarks[19]]

                angles = []
                for i in range(5):
                    if i == 0:
                        # Thumb: calculate CMC flexion using normal angle instead
                        mcp_flexion = calculate_angle(wrist, finger_mcps[i], finger_pips[i])
                        mcp_flexion = neutralize_angle(mcp_flexion)
                    else:
                        mcp_flexion = calculate_mcp_felxion(wrist, finger_mcps[i], finger_pips[i], np.array([0, 0, 1]))

                    if i == 0:
                        mcp_abduction = calculate_abduction(wrist, finger_mcps[i], landmarks[5], finger_pips[i])
                    else:
                        mcp_abduction = calculate_abduction(wrist, finger_mcps[i], finger_mcps[i-1], finger_pips[i])
                    pip_angle = calculate_angle(finger_mcps[i], finger_pips[i], finger_dips[i])
                    pip_angle = neutralize_angle(pip_angle)
                    angles.extend([mcp_flexion, mcp_abduction, pip_angle])
                
                joint_angles.data = angles
        
        # self.get_logger().info(f'Joint angles: {joint_angles.data}')
                
        self.publisher.publish(joint_angles)
        
        # Display the image with hand landmarks
        cv2.imshow("Hand Tracking", image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HandAngleNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
