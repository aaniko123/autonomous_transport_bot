import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class HumanDetector(Node):
    def __init__(self):
        super().__init__('human_detector')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.pub = self.create_publisher(Image, '/human_detection/image', 10)

        # Load YOLOv3-tiny
        self.net = cv2.dnn.readNetFromONNX("/home/aditya/yolo_models/yolov3-tiny.onnx")

        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        with open("/home/aditya/yolo_models/coco.names", "r") as f:
            self.classes = f.read().splitlines()

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)

        layer_names = self.net.getUnconnectedOutLayersNames()
        detections = self.net.forward(layer_names)

        boxes = []
        confidences = []
        class_ids = []

        for output in detections:
            for det in output:
                scores = det[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if class_id == 0 and confidence > 0.5:  # class_id 0 = person
                    center_x, center_y, w, h = (det[0:4] * np.array([width, height, width, height])).astype("int")
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, int(w), int(h)])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        for i in indices.flatten():
            x, y, w, h = boxes[i]
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            label = f"{self.classes[class_ids[i]]}: {confidences[i]:.2f}"
            cv2.putText(frame, label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Publish annotated frame
        debug_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(debug_img)

def main(args=None):
    rclpy.init(args=args)
    node = HumanDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
