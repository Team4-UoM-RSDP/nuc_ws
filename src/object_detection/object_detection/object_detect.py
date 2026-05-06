import argparse
import math
import os

import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

# ROS 2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Float32
from controller_interfaces.srv import DetectObjectsOn, DetectObjectsOff
from controller_interfaces.msg import DetectedObjects
import tf2_ros


SHAPE_WEIGHTS = os.path.join(os.path.dirname(__file__), "weights.pt")

IMG_SIZE = 736
DEVICE = "cpu"


class ObjectDetectionPublisher(Node):
    def __init__(self, args):
        super().__init__("object_detection_publisher")

        self.args = args
        self.frame_id = args.frame_id

        self.start_camera = False
        self.initialise_camera = True
        self.model = None
        self.shape_names = None
        self.pipeline = None
        self.pipeline_started = False
        self.align = None
        self.depth_scale = None

        self.transform_listener_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.transform_listener_buffer, self
        )
        self.parent_name = "/base_link"  #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.child_name = "/camera_color_frame"

        self.color_pub = self.create_publisher(
            String,
            "/object_detection/color",
            1,
        )

        self.position_pub = self.create_publisher(
            msg_type=DetectedObjects,
            topic="/detected_objects",
            qos_profile=1,
        )

        self.side_pub = self.create_publisher(
            Float32,
            "/object_detection/side_m",
            1,
        )

        self.get_logger().info("Publishing color to: /object_detection/color")
        self.get_logger().info("Publishing position to: /object_detection/position")
        self.get_logger().info("Publishing side length to: /object_detection/side_m")
        self.get_logger().info(f"Frame ID: {self.frame_id}")

        self.object_detect_on_server = self.create_service(
            DetectObjectsOn,
            "/turn_object_detection_on",
            self.object_detect_on_callback,
        )
        self.object_detect_off_server = self.create_service(
            DetectObjectsOff,
            "/turn_object_detection_off",
            self.object_detect_off_callback,
        )

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Object detection services ready")

    def publish_detection(self, color, x, y, z, side_m=None):
        color_msg = String()
        color_msg.data = str(color)
        self.color_pub.publish(color_msg)

        pos_msg = DetectedObjects()

        pos_msg.x = float(x)
        pos_msg.y = float(y)
        pos_msg.z = float(z)

        self.position_pub.publish(pos_msg)

        if side_m is not None:
            side_msg = Float32()
            side_msg.data = float(side_m)
            self.side_pub.publish(side_msg)

    def object_detect_on_callback(self, request, response):
        self.start_camera = True
        self.initialise_camera = True
        response.success = True
        self.get_logger().info("Object detection turned on")
        return response

    def object_detect_off_callback(self, request, response):
        self.start_camera = False
        self.initialise_camera = True
        self.shutdown_camera()
        self.model = None
        self.shape_names = None
        response.success = True
        self.get_logger().info("Object detection turned off")
        return response

    def timer_callback(self):
        try:
            self.main_running()
        except Exception as exc:
            self.start_camera = False
            self.initialise_camera = True
            self.shutdown_camera()
            self.get_logger().error(f"Object detection stopped: {exc}")

    def initialise_detection(self):
        if not self.args.weights:
            raise RuntimeError("YOLO weights needed!")

        self.get_logger().info(f"Loading YOLOv8 shape model: {self.args.weights}")
        self.model = YOLO(self.args.weights)
        self.shape_names = self.model.names

        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(
            rs.stream.depth,
            self.args.width,
            self.args.height,
            rs.format.z16,
            self.args.fps,
        )

        config.enable_stream(
            rs.stream.color,
            self.args.width,
            self.args.height,
            rs.format.bgr8,
            self.args.fps,
        )

        profile = self.pipeline.start(config)
        self.pipeline_started = True
        self.align = rs.align(rs.stream.color)

        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        self.initialise_camera = False

        self.get_logger().info(f"Depth scale: {self.depth_scale} m/unit")
        self.get_logger().info("Camera and YOLO initialised")

    def shutdown_camera(self):
        if self.pipeline is not None and self.pipeline_started:
            self.pipeline.stop()
            self.pipeline_started = False
        self.pipeline = None
        self.align = None
        self.depth_scale = None
        cv2.destroyAllWindows()

    def main_running(self):
        if not self.start_camera:
            return

        if self.initialise_camera:
            self.shutdown_camera()
            self.initialise_detection()

        if self.pipeline is None or self.align is None or self.model is None:
            return

        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # align 到 color 之后，使用 color intrinsics
        depth_intrin = color_frame.profile.as_video_stream_profile().intrinsics

        h_img, w_img = color_image.shape[:2]

        results = self.model(
            source=color_image,
            imgsz=self.args.imgsz,
            conf=self.args.conf,
            device=self.args.device,
            verbose=False,
            stream=False,
        )

        boxes = results[0].boxes

        if boxes is None or boxes.xyxy.shape[0] == 0:
            cv2.imshow("RealSense Color+Shape ROS2", color_image)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.start_camera = False
                self.initialise_camera = True
                self.shutdown_camera()
            return

        candidates = []

        for i in range(boxes.xyxy.shape[0]):
            xyxy = boxes.xyxy[i].cpu().numpy().astype(int)
            x1, y1, x2, y2 = xyxy.tolist()

            x1, y1, x2, y2 = clamp_box(
                x1,
                y1,
                x2,
                y2,
                w_img,
                h_img,
            )

            x_center = int((x1 + x2) / 2)
            y_center = int((y1 + y2) / 2)

            x_center, y_center = clamp_xy(
                x_center,
                y_center,
                w_img,
                h_img,
            )

            depth = median_depth_m(
                depth_image,
                x_center,
                y_center,
                self.depth_scale,
                radius=6,
            )

            if depth is None or depth <= 0:
                continue
            try:
                self.tfs = self.transform_listener_buffer.lookup_transform(
                    self.parent_name, self.child_name, rclpy.time.Time()
                )

                candidates.append(
                    (
                        depth,
                        i,
                        x1,
                        y1,
                        x2,
                        y2,
                        x_center,
                        y_center,
                    )
                )

            except tf2_ros.TransformException as e:
                self.get_logger().error(
                    f"Could not get transform from `{self.parent_name}` to `{self.child_name}`: {e}"
                )

        if not candidates:
            cv2.imshow("RealSense Color+Shape ROS2", color_image)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.start_camera = False
                self.initialise_camera = True
                self.shutdown_camera()
            return

        candidates.sort(key=lambda item: item[0])

        best_depth, best_i, x1, y1, x2, y2, x_center, y_center = candidates[0]

        conf = float(boxes.conf[best_i].item())
        cls_id = int(boxes.cls[best_i].item())
        shape_name = get_shape_name(self.shape_names, cls_id)

        roi = color_image[y1:y2, x1:x2]

        pred_color = dominant_color_bgr(roi)
        if pred_color is None:
            pred_color = "unknown"

        refined_bbox, corners, pred_color, _mask = refine_bbox_by_color_edges(
            color_image,
            (x1, y1, x2, y2),
            pred_color,
        )

        rx1, ry1, rx2, ry2 = refined_bbox

        refined_center_x = int((rx1 + rx2) / 2)
        refined_center_y = int((ry1 + ry2) / 2)

        refined_center_x, refined_center_y = clamp_xy(
            refined_center_x,
            refined_center_y,
            w_img,
            h_img,
        )

        side_m, refined_depth, size_method = estimate_side_m(
            depth_image,
            depth_intrin,
            refined_bbox,
            corners,
            self.depth_scale,
        )

        if refined_depth is None:
            refined_depth = best_depth

        X, Y, Z = rs.rs2_deproject_pixel_to_point(
            depth_intrin,
            [float(refined_center_x), float(refined_center_y)],
            float(refined_depth),
        )
        R, translate = get_rotation_translation_matrix(self.tfs)
        Transformed = R @ (np.array([X, Y, Z]).T) + (translate.T)
        self.publish_detection(
            pred_color,
            Transformed[0],
            Transformed[1],
            Transformed[2],
            side_m=side_m,
        )

        cv2.rectangle(
            color_image,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2,
        )

        cv2.rectangle(
            color_image,
            (rx1, ry1),
            (rx2, ry2),
            (0, 220, 255),
            2,
        )

        if corners is not None:
            pts = np.array(corners, dtype=np.int32)
            cv2.polylines(
                color_image,
                [pts],
                True,
                (255, 0, 0),
                2,
            )

        cv2.circle(
            color_image,
            (refined_center_x, refined_center_y),
            5,
            (0, 0, 255),
            -1,
        )

        side_text = f"side={side_m * 100:.1f}cm" if side_m is not None else "side=?"
        label = f"{pred_color} {shape_name} {conf:.2f} {side_text}"

        cv2.putText(
            color_image,
            label,
            (rx1, max(20, ry1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

        depth_text = f"depth={refined_depth:.3f}m"

        cv2.putText(
            color_image,
            depth_text,
            (rx1, min(h_img - 10, ry2 + 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )

        xyz_text = f"X={X:.3f} Y={Y:.3f} Z={Z:.3f}"

        cv2.putText(
            color_image,
            xyz_text,
            (rx1, min(h_img - 10, ry2 + 45)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        if side_m is not None:
            side_value = f"{side_m:.3f}m"
        else:
            side_value = "?"

        self.get_logger().info(
            f"[NEAREST_EDGE] {pred_color} {shape_name} | "
            f"conf={conf:.2f} | "
            f"depth={refined_depth:.3f}m | "
            f"side={side_value} | "
            f"X={X:.3f}, Y={Y:.3f}, Z={Z:.3f} | "
            f"method={size_method}"
        )

        cv2.imshow("RealSense Color+Shape ROS2", color_image)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.start_camera = False
            self.initialise_camera = True
            self.shutdown_camera()

    def destroy_node(self):
        self.shutdown_camera()
        super().destroy_node()


def get_rotation_translation_matrix(tfs):
    x = tfs.transform.translation.x
    y = tfs.transform.translation.y
    z = tfs.transform.translation.z

    # 3. Pull out Rotation (Quaternion)
    qx = tfs.transform.rotation.x
    qy = tfs.transform.rotation.y
    qz = tfs.transform.rotation.z
    qw = tfs.transform.rotation.w

    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    r = math.atan2(sinr_cosp, cosr_cosp)
    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        p = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        p = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    y = math.atan2(siny_cosp, cosy_cosp)

    # Pre-calculate sines and cosines
    cx = np.cos(r)
    sx = np.sin(r)
    cy = np.cos(p)
    sy = np.sin(p)
    cz = np.cos(y)
    sz = np.sin(y)

    # Combined matrix expansion
    R = np.array(
        [
            [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx],
        ]
    )

    return R, [x, y, z]


def median_depth_m(depth_image, x, y, depth_scale, radius=4):
    h, w = depth_image.shape[:2]

    x1 = max(0, x - radius)
    y1 = max(0, y - radius)
    x2 = min(w, x + radius + 1)
    y2 = min(h, y + radius + 1)

    patch = depth_image[y1:y2, x1:x2].astype(np.float32)
    vals = patch[patch > 0]

    if vals.size == 0:
        return None

    return float(np.median(vals) * depth_scale)


def clamp_xy(x, y, width, height):
    x = max(0, min(int(x), width - 1))
    y = max(0, min(int(y), height - 1))
    return x, y


def clamp_box(x1, y1, x2, y2, width, height):
    x1 = max(0, min(int(x1), width - 1))
    y1 = max(0, min(int(y1), height - 1))
    x2 = max(0, min(int(x2), width - 1))
    y2 = max(0, min(int(y2), height - 1))

    if x2 <= x1:
        x2 = min(width - 1, x1 + 1)
    if y2 <= y1:
        y2 = min(height - 1, y1 + 1)

    return x1, y1, x2, y2


def get_shape_name(shape_names, cls_id):
    if isinstance(shape_names, dict):
        return shape_names.get(cls_id, str(cls_id))

    if 0 <= cls_id < len(shape_names):
        return str(shape_names[cls_id])

    return str(cls_id)


def dominant_color_bgr(
    bgr_img,
    sat_thresh=40,
    min_ratio=0.03,
    redfam_min_ratio=0.7,
    pink_v_thresh=110,
    pink_s_thresh=190,
):
    if bgr_img is None or bgr_img.size == 0:
        return None

    hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    valid_mask = s > sat_thresh
    if valid_mask.sum() == 0:
        return None

    h_valid = h[valid_mask]
    s_valid = s[valid_mask]
    v_valid = v[valid_mask]

    color_ranges = {
        "red": [(0, 15), (150, 179)],
        "yellow": [(18, 35)],
        "green": [(40, 85)],
        "blue": [(90, 120)],
        "purple": [(125, 165)],
    }

    def count_range(h_vals, ranges):
        total = 0
        for lo, hi in ranges:
            total += ((h_vals >= lo) & (h_vals <= hi)).sum()
        return total

    total_pixels = h_valid.size

    color_scores = {
        name: count_range(h_valid, ranges) for name, ranges in color_ranges.items()
    }

    best_color, best_count = max(color_scores.items(), key=lambda x: x[1])

    if best_count / total_pixels < min_ratio:
        return None

    if best_color != "red":
        return best_color

    redfam_mask = ((h_valid >= 0) & (h_valid <= 15)) | (
        (h_valid >= 150) & (h_valid <= 179)
    )

    redfam_count = redfam_mask.sum()

    if redfam_count == 0 or redfam_count / total_pixels < redfam_min_ratio:
        return "red"

    s_red = s_valid[redfam_mask].astype(np.float32)
    v_red = v_valid[redfam_mask].astype(np.float32)

    mean_s = float(s_red.mean())
    mean_v = float(v_red.mean())

    if mean_v >= pink_v_thresh and mean_s <= pink_s_thresh:
        return "pink"

    return "red"


# =========================================================
# 边缘检测 / 颜色 mask / bbox 细化
# =========================================================


def hsv_mask_for_color(bgr_img, color):
    hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    valid = (s > 40) & (v > 40)

    if color == "red":
        mask = valid & (((h >= 0) & (h <= 15)) | ((h >= 150) & (h <= 179)))
    elif color == "pink":
        mask = (
            (s > 25) & (v > 80) & (((h >= 0) & (h <= 18)) | ((h >= 145) & (h <= 179)))
        )
    elif color == "yellow":
        mask = valid & (h >= 18) & (h <= 40)
    elif color == "green":
        mask = valid & (h >= 40) & (h <= 85)
    elif color == "blue":
        mask = valid & (h >= 90) & (h <= 125)
    elif color == "purple":
        mask = valid & (h >= 125) & (h <= 165)
    else:
        mask = valid

    mask = mask.astype(np.uint8) * 255

    kernel = np.ones((5, 5), np.uint8)

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel,
        iterations=1,
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel,
        iterations=2,
    )

    return mask


def refine_bbox_by_color_edges(color_image, bbox, pred_color):
    h_img, w_img = color_image.shape[:2]

    x1, y1, x2, y2 = bbox
    x1, y1, x2, y2 = clamp_box(x1, y1, x2, y2, w_img, h_img)

    roi = color_image[y1:y2, x1:x2]

    if roi is None or roi.size == 0:
        return (x1, y1, x2, y2), None, pred_color, None

    if pred_color is None or pred_color == "unknown":
        pred_color = dominant_color_bgr(roi) or "unknown"

    mask = hsv_mask_for_color(roi, pred_color)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    min_area = max(80.0, roi.shape[0] * roi.shape[1] * 0.003)
    contours = [c for c in contours if cv2.contourArea(c) >= min_area]

    if not contours:
        return (x1, y1, x2, y2), None, pred_color, mask

    contour = max(contours, key=cv2.contourArea)

    rx, ry, rw, rh = cv2.boundingRect(contour)
    pad = max(3, int(max(rw, rh) * 0.035))

    refined_bbox = clamp_box(
        x1 + rx - pad,
        y1 + ry - pad,
        x1 + rx + rw + pad,
        y1 + ry + rh + pad,
        w_img,
        h_img,
    )

    rect = cv2.minAreaRect(contour)
    corners = cv2.boxPoints(rect)

    corners[:, 0] += x1
    corners[:, 1] += y1

    return refined_bbox, corners.astype(np.float32), pred_color, mask


# =========================================================
# 用边缘角点估计 cube 尺寸
# =========================================================


def deproject_point(depth_intrin, pixel, depth_m):
    if depth_m is None or depth_m <= 0:
        return None

    point = rs.rs2_deproject_pixel_to_point(
        depth_intrin,
        [float(pixel[0]), float(pixel[1])],
        float(depth_m),
    )

    return np.array(point, dtype=np.float32)


def estimate_side_m(
    depth_image,
    depth_intrin,
    bbox,
    corners,
    depth_scale,
):
    x1, y1, x2, y2 = bbox

    cx = int((x1 + x2) / 2)
    cy = int((y1 + y2) / 2)

    center_depth = median_depth_m(
        depth_image,
        cx,
        cy,
        depth_scale,
        radius=8,
    )

    if center_depth is None:
        return None, None, "no_depth"

    if corners is not None and len(corners) == 4:
        points_2d = [(float(x), float(y)) for x, y in corners]
    else:
        points_2d = [
            (float(x1), float(y1)),
            (float(x2), float(y1)),
            (float(x2), float(y2)),
            (float(x1), float(y2)),
        ]

    points_3d = []

    for px, py in points_2d:
        local_depth = median_depth_m(
            depth_image,
            int(px),
            int(py),
            depth_scale,
            radius=5,
        )

        use_depth = local_depth if local_depth is not None else center_depth

        p = deproject_point(
            depth_intrin,
            (px, py),
            use_depth,
        )

        if p is not None:
            points_3d.append(p)

    if len(points_3d) == 4:
        edges = [
            float(np.linalg.norm(points_3d[(i + 1) % 4] - points_3d[i]))
            for i in range(4)
        ]

        sane_edges = [e for e in edges if 0.01 <= e <= 1.5]

        if sane_edges:
            return float(np.median(sane_edges)), center_depth, "corner_deproject"

    fx = float(getattr(depth_intrin, "fx", 0.0) or 0.0)
    fy = float(getattr(depth_intrin, "fy", 0.0) or 0.0)

    if fx > 0 and fy > 0:
        width_m = abs(x2 - x1) * center_depth / fx
        height_m = abs(y2 - y1) * center_depth / fy
        side_m = float(np.median([width_m, height_m]))

        return side_m, center_depth, "bbox_pinhole"

    return None, center_depth, "no_intrinsics"


def parse_args(args=None):
    parser = argparse.ArgumentParser("RealSense YOLO Color Shape ROS2 Arm Publisher")

    parser.add_argument("--weights", default=SHAPE_WEIGHTS)
    parser.add_argument("--frame-id", default="camera_color_optical_frame")
    parser.add_argument("--imgsz", type=int, default=IMG_SIZE)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--device", default=DEVICE)

    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)

    return parser.parse_known_args(args)[0]


def main(args=None):
    rclpy.init(args=args)
    ros_node = None
    try:
        parsed_args = parse_args(args)
        ros_node = ObjectDetectionPublisher(parsed_args)
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
