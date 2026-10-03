#!/usr/bin/env python3

import os
import time
import math
import threading
import queue
import logging
from collections import deque
from typing import Dict, List, Tuple, Optional, Deque

import cv2
import numpy as np
import mediapipe as mp

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32, String, Bool, Float32
from cv_bridge import CvBridge

# Models
from ultralytics import YOLOWorld, SAM
import supervision as sv

# ==============================================================================
# 1. CONFIGURATION & PARAMS
# ==============================================================================
PEAK_THRESHOLD = 0.002     # Jaw motion threshold
CHEWING_WINDOW = 30         # Motion smoothing window
HTM_START_THRESH = 0.30     # Hand-to-mouth start threshold
HTM_END_THRESH = 0.40       # Hand-to-mouth end threshold
VALIDATION_TIME = 3.0       # Validation timeout

FACIAL_LANDMARKS = list(range(0, 18)) + list(range(61, 88))
REFERENCE_LANDMARKS = [1]
LEFT_WRIST = 15
RIGHT_WRIST = 16
MOUTH_CENTER = 13

YOLO_MODEL_PATH = "yolov8s-world.pt"
SAM_MODEL_PATH = "sam_b.pt"
INTERACTION_IOU_THRESHOLD = 0.15
SETTLING_TIME = 1.5
MIN_BITE_GRAMS = 5.0
VOL_BUFFER_LEN = 15
DETECT_CONF = 0.20
DETECT_IOU = 0.45
DEPTH_INVALID = 0

# 表示順序・全項目の固定リスト
FIXED_FOOD_LABELS = [
    "staple food",
    "main dish",
    "side dish 1",
    "side dish 2",
    "soup"
]

UTENSIL_CLASSES = {"spoon", "fork", "knife", "chopsticks", "spork"}
MY_FOOD_CLASSES = [
    "human hand", "spoon", "fork", "knife", "chopsticks", "spork",
    "bowl of white rice", "bowl of soup", "plate", "bowl", "dish", "food"
]

DENSITY_DB: Dict[str, float] = {
    "default": 1.0,
    "staple food": 0.80,
    "main dish": 1.00,
    "side dish 1": 0.60,
    "side dish 2": 0.60,
    "soup": 1.00,
}

def get_density(label: str) -> float:
    return DENSITY_DB.get(label, DENSITY_DB["default"])

def calculate_iou(box1, box2):
    x1, y1 = max(box1[0], box2[0]), max(box1[1], box2[1])
    x2, y2 = min(box1[2], box2[2]), min(box1[3], box2[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area1 = max(0, box1[2] - box1[0]) * max(0, box1[3] - box1[1])
    area2 = max(0, box2[2] - box2[0]) * max(0, box2[3] - box2[1])
    union = area1 + area2 - inter
    return inter / union if union > 0 else 0.0


# ==============================================================================
# 2. CHEWING & BITE COUNTER LOGIC
# ==============================================================================
class BiteCounter:
    def __init__(self):
        self.state = "IDLE"
        self.total_bites = 0
        self.current_chew_count = 0
        self.last_bite_chews = 0
        self.last_intake_time = 0.0
        self.prev_val = 0.0
        self.prev_prev_val = 0.0

    def update(self, hand_dist: float, is_chewing: bool, current_motion_val: float):
        curr_time = time.time()

        if self.state in ["CHEWING", "VALIDATION"] and is_chewing:
            if (self.prev_prev_val < self.prev_val) and (self.prev_val > current_motion_val) and (self.prev_val > PEAK_THRESHOLD):
                self.current_chew_count += 1

        self.prev_prev_val = self.prev_val
        self.prev_val = current_motion_val

        if self.state == "IDLE":
            if hand_dist < HTM_START_THRESH:
                self.state = "INTAKE"
                self.last_intake_time = curr_time
                self.current_chew_count = 0

        elif self.state == "INTAKE":
            if hand_dist > HTM_END_THRESH:
                self.state = "VALIDATION"
                self.last_intake_time = curr_time
            elif is_chewing:
                self._trigger_bite()

        elif self.state == "VALIDATION":
            if is_chewing:
                self._trigger_bite()
            elif (curr_time - self.last_intake_time) > VALIDATION_TIME:
                self.state = "IDLE"
                self.current_chew_count = 0
            elif hand_dist < HTM_START_THRESH:
                self.state = "INTAKE"

        elif self.state == "CHEWING":
            if not is_chewing and hand_dist > HTM_END_THRESH:
                self.last_bite_chews = self.current_chew_count
                self.state = "IDLE"

        return self.total_bites, self.current_chew_count, self.last_bite_chews, self.state

    def _trigger_bite(self):
        if self.state != "CHEWING":
            self.total_bites += 1
            self.current_chew_count = 0
            self.state = "CHEWING"


def get_distance(p1, p2) -> float:
    return float(np.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2))

def get_relative_motion(prev_coords: np.ndarray, curr_coords: np.ndarray) -> float:
    ref_prev = np.mean(prev_coords[REFERENCE_LANDMARKS], axis=0)
    ref_curr = np.mean(curr_coords[REFERENCE_LANDMARKS], axis=0)
    rel_prev = prev_coords[FACIAL_LANDMARKS] - ref_prev
    rel_curr = curr_coords[FACIAL_LANDMARKS] - ref_curr
    delta = rel_curr - rel_prev
    mag = np.linalg.norm(delta, axis=1)
    return float(np.mean(mag))


# ==============================================================================
# 3. FOOD TRACKING & STATS LOGIC
# ==============================================================================
class FoodItemStats:
    def __init__(self, label: str):
        self.label = label
        self.last_seen = time.time()
        self.vol_buffer: Deque[float] = deque(maxlen=VOL_BUFFER_LEN)
        self.current_avg_vol_ml = 0.0
        self.current_weight_g = 0.0
        
        # 初回操作直前のグラム数を固定保持するフィールド
        self.initial_weight_g: Optional[float] = None
        
        self.is_interacting = False
        self.pre_interaction_vol_ml = 0.0
        self.settling_start_time = 0.0
        self.waiting_for_settle = False

        self.bites_count = 0
        self.total_chews = 0
        self.chews_per_bite: List[int] = []

    def update_volume(self, vol_ml: float, density: float):
        if vol_ml > 0:
            self.vol_buffer.append(vol_ml)
            self.current_avg_vol_ml = sum(self.vol_buffer) / len(self.vol_buffer)
            self.current_weight_g = self.current_avg_vol_ml * density

    def set_baseline_before_interaction(self):
        """手に取られる（操作される）直前の重量を基準として1度だけ永久固定"""
        if self.initial_weight_g is None and self.current_weight_g > 0:
            self.initial_weight_g = self.current_weight_g

    @property
    def eaten_percentage(self) -> float:
        """基準重量に対する現在の摂取率（%）を算出"""
        if self.initial_weight_g is None or self.initial_weight_g <= 0:
            return 0.0
        ratio = 1.0 - (self.current_weight_g / self.initial_weight_g)
        return float(np.clip(ratio * 100.0, 0.0, 100.0))

    def record_bite(self, chews: int):
        self.bites_count += 1
        self.chews_per_bite.append(chews)
        self.total_chews += chews

    @property
    def avg_chews(self) -> float:
        return (self.total_chews / self.bites_count) if self.bites_count > 0 else 0.0


def assign_rule_based_labels(color_image: np.ndarray, dets_food: sv.Detections) -> Dict[int, str]:
    """
    和食の標準的な配置（和定食ルール）に基づく領域・配置固定分類:
    1. 主食: 全料理の中で最も左下に位置するもの（すべて「白米」とみなす）
    2. 汁物: 白米の右側（主菜の右下）に位置するもの
    3. 主菜: 中央付近にあり、皿（バウンディングボックス）が最も大きいもの
    4. 副菜: 主菜の左右に位置するもの（左から順に 副菜1, 副菜2）
    """
    assigned: Dict[int, str] = {}
    if len(dets_food) == 0:
        return assigned

    unassigned = list(range(len(dets_food)))

    centers = []
    areas = []
    class_names = []
    for idx in range(len(dets_food)):
        box = dets_food.xyxy[idx]
        cx = (box[0] + box[2]) / 2.0
        cy = (box[1] + box[3]) / 2.0
        area = (box[2] - box[0]) * (box[3] - box[1])
        det_class = dets_food.data["class_name"][idx] if ("class_name" in dets_food.data) else ""
        centers.append((cx, cy))
        areas.append(area)
        class_names.append(det_class)

    # 1. 主食 (staple food) の判定
    staple_idx = -1
    for idx in unassigned:
        if class_names[idx] == "bowl of white rice":
            staple_idx = idx
            break

    if staple_idx == -1 and unassigned:
        staple_idx = min(unassigned, key=lambda idx: centers[idx][0] - centers[idx][1])

    if staple_idx != -1:
        assigned[staple_idx] = "staple food"
        unassigned.remove(staple_idx)

    # 2. 汁物 (soup) の判定
    soup_idx = -1
    if staple_idx != -1:
        staple_cx, staple_cy = centers[staple_idx]
        candidates = [idx for idx in unassigned if centers[idx][0] > staple_cx]
        if candidates:
            soup_idx = max(candidates, key=lambda idx: centers[idx][1])

    if soup_idx == -1 and unassigned:
        for idx in unassigned:
            if "soup" in class_names[idx]:
                soup_idx = idx
                break

    if soup_idx != -1 and soup_idx in unassigned:
        assigned[soup_idx] = "soup"
        unassigned.remove(soup_idx)

    # 3. 主菜 (main dish) の判定
    if unassigned:
        main_idx = max(unassigned, key=lambda idx: areas[idx])
        assigned[main_idx] = "main dish"
        unassigned.remove(main_idx)

    # 4. 副菜 (side dish 1, side dish 2) の判定
    if unassigned:
        unassigned.sort(key=lambda idx: centers[idx][0])
        side_count = 1
        for idx in unassigned:
            assigned[idx] = f"side dish {side_count}"
            side_count += 1

    return assigned


def estimate_volume_from_mask(mask: np.ndarray, depth_map: np.ndarray, intr: Dict) -> float:
    try:
        ys, xs = np.where(mask > 0)
        if len(xs) == 0:
            return 0.0
        depths = depth_map[ys, xs].astype(np.float32)
        valid = depths > DEPTH_INVALID
        if not np.any(valid):
            return 0.0
        depths = depths[valid]
        table_depth_mm = float(np.percentile(depths, 95))
        avg_d = float(np.mean(depths))
        pixel_area_m2 = max(1e-10, (avg_d/1000.0)/intr["fx"] * (avg_d/1000.0)/intr["fy"])
        heights_m = np.maximum(0.0, (table_depth_mm - depths) / 1000.0)
        return float(np.sum(heights_m) * pixel_area_m2) * 1e6
    except Exception:
        return 0.0


# ==============================================================================
# 4. MAIN INTEGRATED ROS 2 NODE
# ==============================================================================
class IntegratedEatingDashboardNode(Node):
    def __init__(self):
        super().__init__("integrated_eating_dashboard_node")
        self.bridge = CvBridge()

        self.declare_parameter("front_color_topic", "/front_camera/color/image_raw")
        self.declare_parameter("top_color_topic", "/top_camera/color/image_raw")
        self.declare_parameter("top_depth_topic", "/top_camera/depth/image_raw")

        front_color_topic = self.get_parameter("front_color_topic").value
        top_color_topic = self.get_parameter("top_color_topic").value
        top_depth_topic = self.get_parameter("top_depth_topic").value

        self.holistic = mp.solutions.holistic.Holistic(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
            refine_face_landmarks=True
        )
        self.bite_counter = BiteCounter()
        self.prev_landmarks: Optional[np.ndarray] = None
        self.motion_queue: deque = deque(maxlen=CHEWING_WINDOW)

        self.get_logger().info("Loading YOLO & SAM Models...")
        self.detection_model = YOLOWorld(YOLO_MODEL_PATH)
        self.segmentation_model = SAM(SAM_MODEL_PATH)
        self.tracker = sv.ByteTrack()
        self.detection_model.set_classes(MY_FOOD_CLASSES)

        self.box_annotator = sv.BoxAnnotator()
        self.label_annotator = sv.LabelAnnotator(text_position=sv.Position.TOP_LEFT)

        # 常に固定5項目のキーで辞書を初期化
        self.food_db: Dict[str, FoodItemStats] = {
            label: FoodItemStats(label) for label in FIXED_FOOD_LABELS
        }
        self.last_accessed_food: Optional[str] = None
        self.intrinsics = {"fx": 550.0, "fy": 550.0, "cx": 320.0, "cy": 240.0}

        self.seg_job_queue = queue.Queue(maxsize=4)
        self.seg_result_queue = queue.Queue(maxsize=8)
        self.seg_worker_stop = threading.Event()
        threading.Thread(target=self.segmentation_worker, daemon=True).start()

        self.latest_front_img: Optional[np.ndarray] = None
        self.latest_top_color: Optional[np.ndarray] = None
        self.latest_top_depth: Optional[np.ndarray] = None

        self.create_subscription(Image, front_color_topic, self.on_front_color, 10)
        self.create_subscription(Image, top_color_topic, self.on_top_color, 10)
        self.create_subscription(Image, top_depth_topic, self.on_top_depth, 10)

        self.create_timer(0.033, self.process_and_render)

    def on_front_color(self, msg: Image):
        try:
            self.latest_front_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass

    def on_top_color(self, msg: Image):
        try:
            self.latest_top_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass

    def on_top_depth(self, msg: Image):
        try:
            if msg.encoding in ["16UC1"]:
                self.latest_top_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").astype(np.uint16)
            elif msg.encoding in ["32FC1"]:
                depth_f = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").astype(np.float32)
                self.latest_top_depth = (np.clip(depth_f, 0, np.finfo(np.float32).max) * 1000.0).astype(np.uint16)
        except Exception:
            pass

    def segmentation_worker(self):
        while not self.seg_worker_stop.is_set():
            try:
                job = self.seg_job_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            color_img, boxes, labels = job
            try:
                if len(boxes) == 0:
                    self.seg_result_queue.put(([], labels))
                    self.seg_job_queue.task_done()
                    continue
                res = self.segmentation_model.predict(source=color_img, bboxes=boxes, verbose=False)
                masks = []
                out0 = res[0]
                if hasattr(out0, "masks") and out0.masks is not None:
                    data = getattr(out0.masks, "data", None)
                    if data is not None:
                        masks = [(m > 0.5).astype(np.uint8) for m in data.detach().cpu().numpy()]
                self.seg_result_queue.put((masks, labels))
            except Exception:
                self.seg_result_queue.put(([], labels))
            finally:
                self.seg_job_queue.task_done()

    def process_and_render(self):
        if self.latest_front_img is None or self.latest_top_color is None or self.latest_top_depth is None:
            return

        front_img = self.latest_front_img.copy()
        top_color = self.latest_top_color.copy()
        top_depth = self.latest_top_depth.copy()

        # A. PROCESS FRONT CAMERA
        frame_rgb = cv2.cvtColor(front_img, cv2.COLOR_BGR2RGB)
        results = self.holistic.process(frame_rgb)

        is_chewing = False
        closest_hand_dist = 1.0
        avg_motion = 0.0

        if results.face_landmarks:
            coords = np.array([[p.x, p.y] for p in results.face_landmarks.landmark], dtype=np.float32)
            if self.prev_landmarks is not None:
                motion_mag = get_relative_motion(self.prev_landmarks, coords)
                self.motion_queue.append(motion_mag)
                avg_motion = float(np.mean(self.motion_queue))
                is_chewing = avg_motion > PEAK_THRESHOLD
            self.prev_landmarks = coords.copy()

        if results.pose_landmarks and results.face_landmarks:
            pose_lm = results.pose_landmarks.landmark
            mouth_pt = results.face_landmarks.landmark[MOUTH_CENTER]
            left_d = get_distance(pose_lm[LEFT_WRIST], mouth_pt)
            right_d = get_distance(pose_lm[RIGHT_WRIST], mouth_pt)
            closest_hand_dist = min(left_d, right_d)

        prev_state = self.bite_counter.state
        total_bites, current_chews, last_chews, state = self.bite_counter.update(
            closest_hand_dist, is_chewing, avg_motion
        )

        if prev_state == "CHEWING" and state == "IDLE":
            if self.last_accessed_food and self.last_accessed_food in self.food_db:
                self.food_db[self.last_accessed_food].record_bite(last_chews)
                self.get_logger().info(f"Recorded Bite to [{self.last_accessed_food}]: {last_chews} chews")

        # B. PROCESS TOP CAMERA
        try:
            det_res = self.detection_model.predict(top_color, conf=DETECT_CONF, iou=DETECT_IOU, verbose=False)
            detections = sv.Detections.from_ultralytics(det_res[0])
        except Exception:
            detections = sv.Detections.empty()

        class_names = detections.data.get("class_name", [])
        hands_idx = [i for i, n in enumerate(class_names) if n == "human hand"]
        utensils_idx = [i for i, n in enumerate(class_names) if n in UTENSIL_CLASSES]
        food_idx = [i for i, n in enumerate(class_names) if n != "human hand" and n not in UTENSIL_CLASSES]

        dets_hands = detections[np.array(hands_idx)] if hands_idx else sv.Detections.empty()
        dets_utensils = detections[np.array(utensils_idx)] if utensils_idx else sv.Detections.empty()
        dets_food = detections[np.array(food_idx)] if food_idx else sv.Detections.empty()

        if len(dets_food) > 0:
            dets_food = self.tracker.update_with_detections(dets_food)

        rule_labels = assign_rule_based_labels(top_color, dets_food)
        interacting_labels = set()

        def check_interact(actuators):
            if len(dets_food) == 0:
                return
            for a_box in actuators.xyxy:
                for i, f_box in enumerate(dets_food.xyxy):
                    if calculate_iou(a_box, f_box) > INTERACTION_IOU_THRESHOLD:
                        label = rule_labels.get(i, "main dish")
                        interacting_labels.add(label)
                        self.last_accessed_food = label

        if len(dets_hands) > 0: check_interact(dets_hands)
        if len(dets_utensils) > 0: check_interact(dets_utensils)

        boxes_to_seg, labels_to_seg = [], []
        has_tracker_ids = hasattr(dets_food, "tracker_id") and dets_food.tracker_id is not None and len(dets_food.tracker_id) > 0

        if has_tracker_ids:
            for i, tid in enumerate(dets_food.tracker_id):
                label = rule_labels.get(i, "main dish")
                if label not in self.food_db:
                    self.food_db[label] = FoodItemStats(label)

                st = self.food_db[label]
                st.last_seen = time.time()
                is_overlapped = (label in interacting_labels)

                if is_overlapped and not st.is_interacting:
                    # 手やカトラリーが料理に触れる直前のグラム数を基準値として永久固定
                    st.set_baseline_before_interaction()
                    st.is_interacting = True
                    st.waiting_for_settle = False
                    st.pre_interaction_vol_ml = st.current_avg_vol_ml

                elif not is_overlapped and st.is_interacting:
                    st.is_interacting = False
                    st.waiting_for_settle = True
                    st.settling_start_time = time.time()

                if st.waiting_for_settle and (time.time() - st.settling_start_time > SETTLING_TIME):
                    st.waiting_for_settle = False

                if not is_overlapped and not st.waiting_for_settle:
                    boxes_to_seg.append(list(map(int, dets_food.xyxy[i])))
                    labels_to_seg.append(label)

        if boxes_to_seg:
            try:
                self.seg_job_queue.put_nowait((top_color.copy(), np.array(boxes_to_seg), labels_to_seg))
            except queue.Full:
                pass

        try:
            while True:
                masks, labels = self.seg_result_queue.get_nowait()
                for idx, mask in enumerate(masks):
                    if idx < len(labels) and mask is not None:
                        label = labels[idx]
                        if mask.shape != top_depth.shape:
                            mask = cv2.resize(mask.astype(np.uint8), (top_depth.shape[1], top_depth.shape[0]), cv2.INTER_NEAREST)
                        vol_ml = estimate_volume_from_mask(mask, top_depth, self.intrinsics)
                        if label in self.food_db:
                            self.food_db[label].update_volume(vol_ml, get_density(label))
                self.seg_result_queue.task_done()
        except queue.Empty:
            pass

        food_draw_labels = []
        if has_tracker_ids:
            for i, tid in enumerate(dets_food.tracker_id):
                lbl = rule_labels.get(i, "main dish")
                st = self.food_db.get(lbl)
                eaten_pct = st.eaten_percentage if st else 0.0
                food_draw_labels.append(f"{lbl}: Eaten {eaten_pct:.0f}%")

            annotated_top = self.box_annotator.annotate(top_color.copy(), dets_food)
            annotated_top = self.label_annotator.annotate(annotated_top, dets_food, labels=food_draw_labels)
        else:
            annotated_top = top_color.copy()

        # C. RENDER COMBINED DASHBOARD
        self.render_dashboard(front_img, annotated_top, state, current_chews, last_chews, total_bites)

    def render_dashboard(self, front_img, top_img, state, current_chews, last_chews, total_bites):
        target_h = 480
        h_f, w_f = front_img.shape[:2]
        h_t, w_t = top_img.shape[:2]

        front_resized = cv2.resize(front_img, (int(w_f * target_h / h_f), target_h))
        top_resized = cv2.resize(top_img, (int(w_t * target_h / h_t), target_h))

        panel_w = 400
        panel = np.zeros((target_h, panel_w, 3), dtype=np.uint8)
        panel[:] = (30, 30, 30)

        cv2.putText(panel, "EATING DASHBOARD", (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.line(panel, (15, 45), (panel_w - 15, 45), (100, 100, 100), 1)

        state_color = (0, 255, 0) if state == "CHEWING" else ((0, 165, 255) if state == "INTAKE" else (200, 200, 200))
        cv2.putText(panel, f"State: {state}", (15, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.65, state_color, 2)

        disp_chew = current_chews if state != "IDLE" else last_chews
        cv2.putText(panel, f"Current Chew: {disp_chew}", (15, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(panel, f"Total Bites : {total_bites}", (200, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        cv2.line(panel, (15, 120), (panel_w - 15, 120), (100, 100, 100), 1)

        cv2.putText(panel, "FOOD CONSUMPTION TRACKING", (15, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

        y_offset = 175
        # 常に固定の5項目を順序通りループして描画
        for label in FIXED_FOOD_LABELS:
            item = self.food_db.get(label, FoodItemStats(label))
            is_active = (label == self.last_accessed_food)
            bg_color = (60, 60, 20) if is_active else (45, 45, 45)
            text_color = (0, 255, 255) if is_active else (220, 220, 220)

            # パネル枠描画
            cv2.rectangle(panel, (10, y_offset - 15), (panel_w - 10, y_offset + 38), bg_color, -1)
            cv2.rectangle(panel, (10, y_offset - 15), (panel_w - 10, y_offset + 38), (80, 80, 80), 1)

            cv2.putText(panel, f"[{label.upper()}]", (15, y_offset + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)
            
            # Rem: XXg から Eaten: XX% に表示変更
            cv2.putText(panel, f"Eaten: {item.eaten_percentage:.0f}%", (260, y_offset + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 255, 180), 1)

            chew_info = f"Bites: {item.bites_count}  |  Avg Chew: {item.avg_chews:.1f} / bite"
            cv2.putText(panel, chew_info, (20, y_offset + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)

            if item.chews_per_bite:
                recent_history = " ".join([f"[{c}c]" for c in item.chews_per_bite[-4:]])
                cv2.putText(panel, f"History: {recent_history}", (20, y_offset + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (150, 150, 150), 1)

            y_offset += 58
            if y_offset > target_h - 10:
                break

        combined = np.hstack((front_resized, top_resized, panel))
        cv2.imshow("Eating Behavior Dashboard", combined)
        cv2.waitKey(1)

    def destroy_node(self):
        self.seg_worker_stop.set()
        if self.holistic:
            self.holistic.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IntegratedEatingDashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()