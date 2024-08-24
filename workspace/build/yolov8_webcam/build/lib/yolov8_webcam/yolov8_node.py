import rclpy
from rclpy.node import Node
from custom_interface_yolo.action import CameraStream
from rclpy.action import ActionServer
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
import threading
from std_msgs.msg import Float32

class YOLOv8WebcamNode(Node):
    def __init__(self):
        super().__init__('yolov8_webcam_node')

        # Initialize the YOLO model
        model_path = os.path.abspath('best.pt')
        self.model = YOLO(model_path)

        # Initialize webcam
        self.cap = cv2.VideoCapture(0)  # 0 is usually the default webcam
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam")

        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Initialize variables to hold the most recent results
        self.latest_result = CameraStream.Result()
        self.lock = threading.Lock()

        # Create an action server
        self._action_server = ActionServer(
            self,
            CameraStream,
            'camera_stream',
            execute_callback=self.execute_callback
        )

        # Start a thread to continuously process frames
        self.process_thread = threading.Thread(target=self.process_frames)
        self.process_thread.start()

        self.get_logger().info("YOLOv8 Webcam Node with Action Server has started.")

    def process_frames(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to capture image")
                continue

            # Run YOLO model on the frame
            results = self.model(frame)

            with self.lock:
                if results[0].boxes:
                    # Extract the bounding box center
                    box = results[0].boxes[0]
                    bbox = box.xyxy[0]  # [x1, y1, x2, y2]
                    x_center = (bbox[0] + bbox[2]) / 2
                    y_center = (bbox[1] + bbox[3]) / 2

                    # Update result with bounding box center
                    self.latest_result.success = True
                    self.latest_result.x = float(x_center)  # Ensure x is float32
                    self.latest_result.y = float(y_center)  # Ensure y is float32
                else:
                    self.latest_result.success = False
                    self.latest_result.x = 0.0
                    self.latest_result.y = 0.0

            # Annotate the frame with detection results
            annotated_frame = results[0].plot()

            # Display the annotated frame
            cv2.imshow('YOLOv8 Webcam Stream', annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing action')

        # Return the latest detection results
        with self.lock:
            result = self.latest_result

        goal_handle.set_succeeded(result)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        self.process_thread.join()
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
