import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  
from cv_bridge import CvBridge
import cv2
from mars_rover_tasks.plant_detector import PlantDetector 



class PlantDetectorNode(Node):
    def __init__(self):
        super().__init__('plant_detector_node')
        
        # Initialize the CvBridge
        self.bridge = CvBridge()
        
        path_to_model = "/home/user/ros2_ws/src/basic_ros2_extra_files/plant_detector/best_plant_detector_model.pth"
        self.plant_detector = PlantDetector(model_path=path_to_model)
        
        self.subscription = self.create_subscription(
            Image,
            '/leo/camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(String, '/plant_detector', 10)


    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        prediction = self.plant_detector.predict(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        
        if prediction > 0.5:
            result = f"Plant detected with confidence: {prediction:.2f}"
            self.get_logger().warning(result)
        else:
            result = f"No plant detected. Confidence: {1 - prediction:.2f}"
            self.get_logger().info(result)
        
        msg = String()
        msg.data = result
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    plant_detector_node = PlantDetectorNode()

    rclpy.spin(plant_detector_node)

    plant_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()