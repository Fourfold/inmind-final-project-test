import rclpy
from rclpy.node import Node
from detection_interfaces.action import FindObject
from rclpy.action import ActionServer
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YOLOv8WebcamNode(Node):
    def __init__(self):
        super().__init__('yolov8_webcam_node')

        # Initialize the YOLO model
        model_path = 'best.pt'  # Ensure this path is correct
        self.model = YOLO(model_path)

        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Create an action server
        self._action_server = ActionServer(
            self,
            FindObject,
            'find_object_server',
            execute_callback=self.execute_callback
        )

        # Create a subscription to the TurtleBot3 camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Replace with the correct topic if different
            self.image_callback,
            10
        )

        self.get_logger().info("YOLOv8 Camera Node with Action Server has started.")

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing action')

        input = goal_handle.request.object_type

        result = FindObject.Result()
        try:

            # Run YOLO model on the frame
            results = self.model(self.frame)

            if results[0].boxes:
                # Extract the bounding box center
                box = results[0].boxes[0]
                bbox = box.xyxy[0]  # [x1, y1, x2, y2]
                x_center = (bbox[0] + bbox[2]) / 2
                y_center = (bbox[1] + bbox[3]) / 2

                # Update result with bounding box center
                result.success = True
                result.x = float(x_center)  # Ensure x is float32
                result.y = float(y_center)  # Ensure y is float32
                self.get_logger().info("Can Found")
            else:
                result.success = False
                result.x = 0.0
                result.y = 0.0

            # Annotate the frame with detection results
            annotated_frame = results[0].plot()

            # Optionally display the annotated frame
            cv2.imshow('YOLOv8 Camera Stream', annotated_frame)
            cv2.waitKey(1)  # Display frame until next frame arrives

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

        goal_handle.succeed(result)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8WebcamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
