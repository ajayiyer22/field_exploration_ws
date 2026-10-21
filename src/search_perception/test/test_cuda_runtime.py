"""Exercise the installed ROS detector with real YOLO26 CUDA inference."""

import json
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import numpy as np
import pytest
import rclpy
import torch
import ultralytics
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray
from search_perception.image_processing import image_to_bgr


@pytest.mark.skipif(not torch.cuda.is_available(), reason='CUDA GPU required for real inference test')
@pytest.mark.parametrize('target,expected_class', [('person', 'person'),
                                                  ('red tennis ball', 'sports ball')])
def test_cuda_all_cameras_confirmation_and_retained_event(tmp_path, monkeypatch, target, expected_class):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(200 + os.getpid() % 20 +
                                          (1 if target == 'red tennis ball' else 0)))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    monkeypatch.setenv('YOLO_CONFIG_DIR', str(tmp_path))
    monkeypatch.setenv('MPLCONFIGDIR', str(tmp_path / 'mpl'))
    rclpy.init()
    node = rclpy.create_node('yolo26_runtime_test')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    statuses, found, late_found, images, detections = [], [], [], {}, {}
    subs = [node.create_subscription(String, '/search/yolo26/status',
                                    lambda msg: statuses.append(json.loads(msg.data)), latched),
            node.create_subscription(Bool, '/search/target_found',
                                     lambda msg: found.append(msg.data), latched)]
    namespaces = ['test_a', 'test_b', 'test_c']
    pubs = {}
    for ns in namespaces:
        pubs[ns] = node.create_publisher(Image, f'/{ns}/sensors/camera_0/color/image',
                                        qos_profile_sensor_data)
        subs.append(node.create_subscription(Image, f'/{ns}/yolo26/annotated_image',
                    lambda msg, name=ns: images.__setitem__(name, msg), QoSProfile(depth=2)))
        subs.append(node.create_subscription(Detection2DArray, f'/{ns}/yolo26/detections',
                    lambda msg, name=ns: detections.__setitem__(name, msg), qos_profile_sensor_data))
    if target == 'red tennis ball':
        bus = cv2.imread(str(Path(__file__).parent / 'assets' / 'red_ball_gazebo.png'))
    else:
        bus = cv2.imread(str(Path(ultralytics.__file__).parent / 'assets' / 'bus.jpg'))
        bus = cv2.resize(bus, (424, 560))
    assert bus is not None
    executable = Path(get_package_prefix('search_perception')) / 'lib/search_perception/yolo26_detector'
    weights = Path(get_package_share_directory('search_perception')) / 'models/yolo26n.pt'
    log = (tmp_path / 'detector.log').open('w+')
    process = subprocess.Popen([
        str(executable), '--ros-args', '-p', f'model_path:={weights}',
        '-p', 'robot_namespaces:=[test_a, test_b, test_c]', '-p', f'target_class:={target}',
        # Fixed-stamp fixtures must survive GPU scheduling delays on a busy simulation host.
        '-p', 'max_image_age_s:=30.0', '-p', 'max_hit_gap_s:=30.0'],
        stdout=log, stderr=subprocess.STDOUT)

    def wait_for(predicate, timeout=20, publish=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert process.poll() is None, (tmp_path / 'detector.log').read_text()
            if publish:
                publish()
            rclpy.spin_once(node, timeout_sec=0.1)
            if predicate():
                return
        pytest.fail(f'Detector condition timed out: {statuses[-1:]}; log: '
                    f'{(tmp_path / "detector.log").read_text()}')

    def send(stamp, targets=namespaces):
        for ns in targets:
            message = Image(height=bus.shape[0], width=bus.shape[1], encoding='bgr8',
                            step=bus.shape[1] * 3, data=bus.tobytes())
            message.header.frame_id = ns + '/camera'
            message.header.stamp.sec = stamp
            pubs[ns].publish(message)

    try:
        wait_for(lambda: bool(statuses) and all(pub.get_subscription_count() for pub in pubs.values()))
        assert statuses[-1]['device'] == 'cuda:0'
        assert statuses[-1]['backend'] == 'PyTorch'
        assert statuses[-1]['dtype'] == 'torch.float16'
        wait_for(lambda: all(node.count_publishers(f'/{ns}/yolo26/annotated_image') and
                             node.count_publishers(f'/{ns}/yolo26/detections') for ns in namespaces))
        for stamp in (1, 2):
            wait_for(lambda: all(statuses[-1]['cameras'][ns]['processed'] >= stamp for ns in namespaces),
                     publish=lambda s=stamp: send(s))
            assert True not in found
        wait_for(lambda: set(images) == set(namespaces) and set(detections) == set(namespaces))
        assert all(msg.encoding == 'bgr8' for msg in images.values())
        assert all(any(d.results[0].hypothesis.class_id == expected_class for d in msg.detections)
                   for msg in detections.values())
        assert all(d.results[0].hypothesis.class_id == expected_class
                   for msg in detections.values() for d in msg.detections)
        # The only image alteration is the status line at the top; no class boxes or labels.
        for msg in images.values():
            assert np.array_equal(image_to_bgr(msg)[28:], bus[28:])
        # Repeated copies of frame 2 cannot produce the third confirmation hit.
        stop = time.monotonic() + 1.2
        while time.monotonic() < stop:
            send(2)
            rclpy.spin_once(node, timeout_sec=0.1)
        assert True not in found
        wait_for(lambda: True in found, publish=lambda: send(3, ['test_a']))
        subs.append(node.create_subscription(Bool, '/search/target_found',
                                             lambda msg: late_found.append(msg.data), latched))
        wait_for(lambda: True in late_found)
        print('CUDA runtime PASS:', statuses[-1])
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()
        node.destroy_node()
        rclpy.shutdown()
