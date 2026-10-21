"""Continuously infer on the newest team camera frames using PyTorch CUDA."""

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from .image_processing import HitStreak, image_to_bgr, pink_fraction, red_fraction, resolve_target


@dataclass
class Camera:
    namespace: str
    latest: object = None
    arrived: float = 0.0
    last_received_stamp: object = None
    received: int = 0
    processed: int = 0
    streak: HitStreak = field(default_factory=HitStreak)
    annotated_pub: object = None
    detection_pub: object = None


class Yolo26Detector(Node):
    def __init__(self):
        super().__init__('yolo26_detector')
        defaults = {
            'model_path': str(Path(get_package_share_directory('search_perception')) /
                              'models' / 'yolo26n.pt'),
            'robot_namespaces': ['a200_0000', 'j100_0000', 'j100_0001'],
            'target_class': 'red tennis ball', 'device': 0, 'half_precision': True,
            'confidence_threshold': 0.25, 'image_size': 640, 'processing_rate_hz': 30.0,
            'required_consecutive_hits': 3, 'pink_fraction_threshold': 0.08,
            'red_fraction_threshold': 0.50,
            'max_image_age_s': 2.0, 'max_hit_gap_s': 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        for name in ('processing_rate_hz', 'max_image_age_s', 'max_hit_gap_s'):
            value = self.cfg[name]
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.cfg['required_consecutive_hits'] < 1 or self.cfg['image_size'] < 32:
            raise ValueError('required_consecutive_hits must be >= 1 and image_size >= 32')
        for name in ('confidence_threshold', 'pink_fraction_threshold', 'red_fraction_threshold'):
            if not 0 <= self.cfg[name] <= 1:
                raise ValueError(f'{name} must be in [0, 1]')
        namespaces = [ns.strip('/') for ns in self.cfg['robot_namespaces']]
        if not namespaces or any(not ns for ns in namespaces) or len(set(namespaces)) != len(namespaces):
            raise ValueError('robot_namespaces must contain distinct nonempty names')
        weights = Path(self.cfg['model_path']).expanduser()
        if weights.suffix != '.pt' or not weights.is_file():
            raise RuntimeError(f'Local PyTorch .pt weights are required: {weights}')
        device = self.cfg['device']
        if not torch.cuda.is_available() or not 0 <= device < torch.cuda.device_count():
            raise RuntimeError('CUDA GPU is unavailable. Activate field_robotics and check nvidia-smi; '
                               'this detector requires CUDA-enabled PyTorch and will not use CPU inference.')
        torch.set_num_threads(2)
        self.model = YOLO(str(weights), task='detect')
        self.target = ' '.join(self.cfg['target_class'].lower().split())
        self.target_id, self.color = resolve_target(self.target, self.model.names)
        self.predict_kwargs = dict(device=device, imgsz=self.cfg['image_size'],
                                   conf=self.cfg['confidence_threshold'], verbose=False,
                                   quantize='fp16' if self.cfg['half_precision'] else 'fp32')
        if self.target_id is not None:
            self.predict_kwargs['classes'] = [self.target_id]
        # A real warm-up establishes the inference backend (model.model may remain a CPU copy).
        self.model.predict(np.zeros((240, 424, 3), dtype=np.uint8), **self.predict_kwargs)
        backend = self.model.predictor.model
        parameter = next(backend.model.parameters())
        if backend.device.type != 'cuda' or parameter.device.type != 'cuda':
            raise RuntimeError('YOLO26 inference backend is not on CUDA')
        self.gpu_name = torch.cuda.get_device_name(device)
        self.device = str(backend.device)
        self.dtype = str(parameter.dtype)
        self.found = False
        self.confirmation = None
        self.next_camera = 0
        self.failures = 0
        self.latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.found_pub = self.create_publisher(Bool, '/search/target_found', self.latched)
        self.confirmation_pub = self.create_publisher(String, '/search/target_detection', self.latched)
        self.status_pub = self.create_publisher(String, '/search/yolo26/status', self.latched)
        self.cameras = []
        self.camera_subscriptions = []
        for namespace in namespaces:
            camera = Camera(namespace)
            camera.annotated_pub = self.create_publisher(
                Image, f'/{namespace}/yolo26/annotated_image', QoSProfile(depth=1))
            camera.detection_pub = self.create_publisher(
                Detection2DArray, f'/{namespace}/yolo26/detections', qos_profile_sensor_data)
            self.camera_subscriptions.append(self.create_subscription(
                Image, f'/{namespace}/sensors/camera_0/color/image',
                lambda msg, c=camera: self.receive(c, msg), qos_profile_sensor_data))
            self.cameras.append(camera)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / self.cfg['processing_rate_hz'], self.process,
                                       clock=self.steady_clock)
        self.status_timer = self.create_timer(1.0, self.publish_status, clock=self.steady_clock)
        self.publish_status()
        self.get_logger().info(f'YOLO26 ready: PyTorch {torch.__version__}, {self.device}, '
                               f'{self.gpu_name}, {self.dtype}, {len(self.cameras)} cameras; '
                               f'target={self.target or "detection only"!r}')

    @staticmethod
    def stamp(message):
        return (message.header.frame_id, message.header.stamp.sec, message.header.stamp.nanosec)

    def receive(self, camera, message):
        stamp = self.stamp(message)
        if stamp == camera.last_received_stamp:
            return
        camera.latest = message
        camera.last_received_stamp = stamp
        camera.arrived = time.monotonic()
        camera.received += 1

    def publish_status(self):
        self.found_pub.publish(Bool(data=self.found))
        payload = dict(device=self.device, gpu=self.gpu_name, dtype=self.dtype,
                       backend='PyTorch', target_class=self.target, target_found=self.found,
                       cameras={c.namespace: dict(received=c.received, processed=c.processed,
                                consecutive_hits=c.streak.count) for c in self.cameras})
        self.status_pub.publish(String(data=json.dumps(payload)))

    def process(self):
        # Fixed round robin prevents a busy camera from starving another robot.
        for _ in self.cameras:
            camera = self.cameras[self.next_camera % len(self.cameras)]
            self.next_camera += 1
            if camera.latest is not None:
                break
        else:
            return
        message, camera.latest = camera.latest, None
        now = time.monotonic()
        if now - camera.arrived > self.cfg['max_image_age_s']:
            camera.streak.count = 0
            return
        try:
            frame = image_to_bgr(message)
            result = self.model.predict(frame, **self.predict_kwargs)[0]
            if result.boxes.data.device.type != 'cuda':
                raise RuntimeError('Inference returned CPU detection tensors')
        except Exception as error:
            camera.streak.count = 0
            self.failures += 1
            self.get_logger().error(f'Inference failed for {camera.namespace}: {error}')
            if self.failures >= 3:
                raise RuntimeError('Three consecutive camera/inference failures') from error
            return
        self.failures = 0
        camera.processed += 1
        output = Detection2DArray(header=message.header)
        accepted = []
        # Transfer the small detection table once; image inference stays on the GPU.
        for index, box in enumerate(result.boxes.data.cpu().numpy()):
            x1, y1, x2, y2, confidence, class_id = box[:6]
            class_id = int(class_id)
            detection = Detection2D(header=message.header, id=f'{camera.namespace}_{index}')
            detection.bbox.center.position.x = float((x1 + x2) / 2)
            detection.bbox.center.position.y = float((y1 + y2) / 2)
            detection.bbox.size_x = float(x2 - x1)
            detection.bbox.size_y = float(y2 - y1)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = self.model.names[class_id]
            hypothesis.hypothesis.score = float(confidence)
            detection.results.append(hypothesis)
            output.detections.append(detection)
            if class_id == self.target_id:
                x1, y1 = max(0, int(x1)), max(0, int(y1))
                x2, y2 = min(frame.shape[1], int(x2)), min(frame.shape[0], int(y2))
                dx, dy = int((x2 - x1) * 0.08), int((y2 - y1) * 0.08)
                crop = frame[y1 + dy:y2 - dy, x1 + dx:x2 - dx]
                fraction = red_fraction(crop) if self.color == 'red' else pink_fraction(crop)
                threshold = self.cfg['red_fraction_threshold'] if self.color == 'red' else (
                    self.cfg['pink_fraction_threshold'])
                if self.color is None or fraction >= threshold:
                    accepted.append(dict(bbox=[x1, y1, x2, y2], confidence=float(confidence),
                                         color=self.color, color_fraction=fraction))
        count = camera.streak.update(bool(accepted), self.stamp(message), now,
                                     self.cfg['max_hit_gap_s'])
        if not self.found and accepted and count >= self.cfg['required_consecutive_hits']:
            self.found = True
            details = dict(target_class=self.target, robot_namespace=camera.namespace,
                           camera_frame=message.header.frame_id,
                           stamp=dict(sec=message.header.stamp.sec, nanosec=message.header.stamp.nanosec),
                           consecutive_hits=count, detections=accepted)
            self.confirmation = json.dumps(details)
            self.confirmation_pub.publish(String(data=self.confirmation))
            self.publish_status()
            self.get_logger().warning(f'TARGET FOUND: {self.target} by {camera.namespace}')
        # Show only this camera frame's target status, without object labels or boxes.
        # The retained mission confirmation is separate from current-frame visibility.
        annotated = frame.copy()
        label = f'{self.target.capitalize()} detected' if accepted else 'Target not detected'
        cv2.putText(annotated, label, (5, 18), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (0, 255, 0) if accepted else (0, 255, 255), 1, cv2.LINE_AA)
        image = Image(header=message.header, height=annotated.shape[0], width=annotated.shape[1],
                      encoding='bgr8', is_bigendian=0, step=annotated.shape[1] * 3,
                      data=np.ascontiguousarray(annotated).tobytes())
        camera.annotated_pub.publish(image)
        camera.detection_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Yolo26Detector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
