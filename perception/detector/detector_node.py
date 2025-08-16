#!/usr/bin/env python3
import os
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from typing import Optional

IMAGE_TOPIC = os.environ.get("IMAGE_TOPIC", "/camera/image_raw")
ANNOTATED_TOPIC = os.environ.get("ANNOTATED_TOPIC", "/perception/image_annotated")
DETECTIONS_TOPIC = os.environ.get("DETECTIONS_TOPIC", "/perception/detections")

def find_cascade_path() -> Optional[str]:
    for p in (
        os.environ.get("CASCADE_PATH", "").strip(),
        "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
        "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
        "/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
    ):
        if p and os.path.isfile(p):
            return p
    return None

def set_bbox_center(bb, cx: float, cy: float):
    """
    Be tolerant to message variations:
    - Preferred: bb.center.x/y/theta (Pose2D)
    - Fallback seen in some builds: bb.center.position.x/y and bb.center.theta
    """
    try:
        bb.center.x = float(cx)      # Pose2D (expected in Humble)
        bb.center.y = float(cy)
        bb.center.theta = 0.0
        return
    except Exception:
        pass
    try:
        # Some gens expose a "position" field
        bb.center.position.x = float(cx)
        bb.center.position.y = float(cy)
        bb.center.theta = 0.0
        return
    except Exception:
        pass
    # Last resort: set whatever attributes exist
    if hasattr(bb.center, "x"): setattr(bb.center, "x", float(cx))
    if hasattr(bb.center, "y"): setattr(bb.center, "y", float(cy))
    if hasattr(bb.center, "theta"): setattr(bb.center, "theta", 0.0)

class SimpleDetector(Node):
    def __init__(self):
        super().__init__("simple_detector")
        self.bridge = CvBridge()

        cascade_path = find_cascade_path()
        self.det = cv2.CascadeClassifier(cascade_path) if cascade_path else cv2.CascadeClassifier()
        if not cascade_path or self.det.empty():
            self.get_logger().warn(
                f"Cascade not found/failed to load "
                f"({'none' if not cascade_path else cascade_path}); forwarding frames without boxes."
            )
        else:
            self.get_logger().info(f"Cascade: {cascade_path}")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(Image, IMAGE_TOPIC, self.cb, qos)
        self.pub_img = self.create_publisher(Image, ANNOTATED_TOPIC, qos)
        self.pub_det = self.create_publisher(Detection2DArray, DETECTIONS_TOPIC, qos)
        self.get_logger().info(f"Sub: {IMAGE_TOPIC}  Pub: {ANNOTATED_TOPIC}, {DETECTIONS_TOPIC}")

    def cb(self, msg: Image):
        # decode robustly and normalize to BGR for OpenCV
        try:
            enc = (msg.encoding or "").lower()
            if enc in ("bgr8", "mono8", "rgb8"):
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding=enc)
                if enc == "rgb8":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge decode error: {e}")
            return

        det_array = Detection2DArray()
        det_array.header = msg.header

        if not self.det.empty():
            try:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                boxes = self.det.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=3, minSize=(40, 40))
                for (x, y, w, h) in boxes:
                    d = Detection2D()
                    d.header = msg.header
                    set_bbox_center(d.bbox, x + w/2.0, y + h/2.0)
                    d.bbox.size_x = float(w)
                    d.bbox.size_y = float(h)

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = "face"
                    hyp.hypothesis.score = 0.6
                    d.results.append(hyp)
                    det_array.detections.append(d)

                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            except Exception as e:
                self.get_logger().warn(f"detector error: {e}")

        # always publish annotated frame and detections
        self.pub_det.publish(det_array)
        try:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
        except Exception as e:
            self.get_logger().warn(f"cv_bridge publish error: {e}")

def main():
    rclpy.init()
    node = SimpleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
