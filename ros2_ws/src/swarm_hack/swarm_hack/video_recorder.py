"""video_recorder: save an image topic to an MP4 file (real-time duration).

  ros2 run swarm_hack video_recorder                                   # robot camera
  ros2 run swarm_hack video_recorder --ros-args -p topic:=/objects/debug_image -p name:=yolo
  Ctrl+C finishes the file (do NOT close the window with X - the file would be broken).

topic ending in /compressed -> sensor_msgs/CompressedImage (JPEG), otherwise sensor_msgs/Image.
Frames are written so the video plays at real speed: missing frames are repeated, extra ones dropped.
Output: <out_dir>/<name>_<YYYYmmdd_HHMMSS>.mp4 ; out_dir defaults to the Windows Videos folder.
If ffmpeg is installed, an H.264 copy (*_h264.mp4, plays everywhere) is made when you stop.
"""
import glob
import os
import shutil
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from .object_mapper import image_to_np


def default_out_dir():
    for pattern in ('/mnt/c/Users/*/Videos', '/mnt/c/Users/*/Downloads'):
        hits = [p for p in glob.glob(pattern) if '/Public/' not in p and '/Default' not in p]
        if hits:
            return hits[0]
    return os.path.expanduser('~/videos')


class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')
        dp = self.declare_parameter
        self.topic = dp('topic', '/camera/image_raw/compressed').value
        self.fps = float(dp('fps', 15.0).value)
        self.name = dp('name', 'camera').value
        self.stamp_overlay = dp('timestamp', True).value
        out_dir = dp('out_dir', '').value or default_out_dir()
        os.makedirs(out_dir, exist_ok=True)
        self.path = os.path.join(out_dir, f"{self.name}_{time.strftime('%Y%m%d_%H%M%S')}.mp4")
        self.writer = None
        self.t0 = None
        self.written = 0
        self.frames = 0
        self.last = None
        compressed = self.topic.endswith('/compressed')
        self.create_subscription(CompressedImage if compressed else Image, self.topic,
                                 lambda m: self._on_img(m, compressed), qos_profile_sensor_data)
        self.create_timer(5.0, self._report)
        self.get_logger().info(f'recording {self.topic} -> {self.path} (Ctrl+C to stop)')

    def _decode(self, m, compressed):
        if compressed:
            img = cv2.imdecode(np.frombuffer(m.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                raise ValueError('cannot decode JPEG')
            return img
        return image_to_np(m)

    def _open(self, w, h):
        for cc in ('mp4v',):  # pip OpenCV has no H.264 encoder; ffmpeg converts on stop
            wr = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*cc), self.fps, (w, h))
            if wr.isOpened():
                self.get_logger().info(f'video {w}x{h} @ {self.fps:.0f} fps, codec {cc}')
                return wr
        raise RuntimeError('cannot open video writer')

    def _on_img(self, m, compressed):
        try:
            img = self._decode(m, compressed)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return
        now = time.monotonic()
        if self.writer is None:
            self.writer = self._open(img.shape[1], img.shape[0])
            self.size = (img.shape[1], img.shape[0])
            self.t0 = now
        if (img.shape[1], img.shape[0]) != self.size:
            img = cv2.resize(img, self.size)
        if self.stamp_overlay:
            cv2.putText(img, time.strftime('%H:%M:%S'), (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2, cv2.LINE_AA)
        target = int((now - self.t0) * self.fps) + 1   # frames that should exist by now
        n = target - self.written
        if n <= 0:
            return                                     # camera faster than video fps: drop
        if n > self.fps:                               # gap > 1 s (Wi-Fi hiccup): don't freeze-frame it
            n = 1
        for _ in range(n):                             # small gaps: repeat frame -> real-time speed
            self.writer.write(img)
        self.written = target
        self.frames += n
        self.last = now

    def _report(self):
        if self.writer is None:
            self.get_logger().warn(f'no images on {self.topic} yet', throttle_duration_sec=10.0)
        else:
            self.get_logger().info(f'{self.frames / self.fps:6.1f} s recorded -> {self.path}')

    def finish(self):
        if self.writer is None:
            return
        self.writer.release()
        print(f'\nsaved {self.path}  ({self.frames / self.fps:.1f} s)', flush=True)
        if shutil.which('ffmpeg'):
            h264 = self.path.replace('.mp4', '_h264.mp4')
            r = subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', self.path, '-c:v', 'libx264',
                                '-pix_fmt', 'yuv420p', '-preset', 'veryfast', h264])
            if r.returncode == 0:
                print(f'saved {h264}  (H.264, plays in any player)', flush=True)


def main():
    rclpy.init()
    node = VideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
