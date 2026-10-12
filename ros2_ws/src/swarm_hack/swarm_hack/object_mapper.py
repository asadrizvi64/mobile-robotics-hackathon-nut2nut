"""Object mapper: YOLO on the TurtleBot camera -> 3D object positions in the SLAM map.

Camera input: raw sensor_msgs/Image, or sensor_msgs/CompressedImage when image_topic ends
with "/compressed" (use this over Wi-Fi: ~20x less bandwidth).
If camera_info is missing or uncalibrated (K all zeros), intrinsics are built from hfov_deg.

Depth for each bounding box, best source first:
  1. depth image  (if depth_topic is set; must be registered/aligned to the colour image)
  2. 2D lidar     scan points projected into the image; the points that fall in
                  the box's column band give the range to the object's front face
  3. ground plane ray through the bottom-centre of the box intersected with z=0
                  in the map (works for objects standing on the floor below the lidar plane)

Then: centre_cam = Z * K^-1 [u_c, v_c, 1], transformed to map with TF.

Lidar tracking (360 deg): once YOLO has CONFIRMED an object (min_hits), every scan is checked
for a small isolated lidar cluster near the object's map position. If found, the position is
updated from the lidar (more accurate than the camera), so the object keeps being tracked when
it is too close / beside / behind the robot and out of the camera's ~62 deg view. Walls are
ignored (a cluster must be small and isolated). JSON field "track": camera | lidar | camera+lidar | lost.
Height/width from the box size at that depth. Detections are associated per
class within assoc_radius and smoothed; an object is published after min_hits.

Outputs:
  /objects/json         std_msgs/String  {"objects":[{id,cls,x,y,z,z_top,w,h,conf,hits,method}]}
  /objects/markers      visualization_msgs/MarkerArray
  /objects/debug_image  sensor_msgs/Image (annotated, only when someone subscribes)

Runs in its own process so YOLO inference never stalls the drone setpoint stream.
"""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .geom import tf_to_matrix


def image_to_np(msg):
    """sensor_msgs/Image -> numpy without cv_bridge (avoids the numpy-2 / cv_bridge ABI mess)."""
    h, w, enc = msg.height, msg.width, msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('bgr8', 'rgb8', 'bgra8', 'rgba8'):
        c = 4 if enc.endswith('a8') else 3
        img = buf.reshape(h, msg.step)[:, :w * c].reshape(h, w, c)[:, :, :3]
        return np.ascontiguousarray(img[:, :, ::-1] if enc.startswith('rgb') else img)
    if enc == 'mono8':
        g = buf.reshape(h, msg.step)[:, :w]
        return np.ascontiguousarray(np.repeat(g[:, :, None], 3, axis=2))
    if enc in ('16uc1', 'mono16'):
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(h, msg.step // 2)[:, :w].astype(np.float32) / 1000.0
    if enc == '32fc1':
        return np.frombuffer(msg.data, dtype=np.float32).reshape(h, msg.step // 4)[:, :w]
    raise ValueError(f'unsupported encoding {msg.encoding}')


def compressed_to_np(msg):
    import cv2
    img = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f'cannot decode compressed image ({msg.format})')
    return img


class ObjectMapper(Node):
    def __init__(self):
        super().__init__('object_mapper')
        dp = self.declare_parameter
        self.map_frame = dp('map_frame', 'map').value
        image_topic = dp('image_topic', '/camera/image_raw').value
        info_topic = dp('camera_info_topic', '/camera/camera_info').value
        depth_topic = dp('depth_topic', '').value
        scan_topic = dp('scan_topic', '/scan').value
        self.camera_frame_override = dp('camera_frame', '').value  # if image frame_id is not the optical frame
        model = dp('model', 'yolov8n.pt').value
        self.conf = dp('conf', 0.45).value
        self.device = dp('device', '').value  # '', 'cpu', 'cuda:0'
        rate = dp('rate_hz', 5.0).value
        self.classes = [c for c in dp('classes', ['']).value if c]                 # [] = all
        self.ignore = [c for c in dp('ignore_classes', ['person']).value if c]   # don't map the team
        self.assoc_r = dp('assoc_radius', 0.5).value
        self.min_hits = dp('min_hits', 3).value
        self.max_range = dp('max_range', 4.0).value
        self.forget_s = dp('forget_unconfirmed_s', 5.0).value
        self.hfov = math.radians(dp('hfov_deg', 62.2).value)   # Pi camera v2; v1 = 53.5
        self.compressed = image_topic.endswith('/compressed')
        self.lidar_track = dp('lidar_track', True).value
        self.lidar_gate = dp('lidar_gate', 0.35).value          # m: search radius around the object
        self.lidar_cluster = dp('lidar_cluster', 0.12).value    # m: points this close form one cluster
        self.lidar_max_size = dp('lidar_max_size', 0.30).value  # m: bigger clusters (walls) are ignored
        self.obj_radius = dp('object_radius', 0.04).value       # m: lidar sees the front face; shift to centre
        self.single = dp('single_per_class', True).value          # one bottle in the arena: keep only the newest
        self.replace_after_s = dp('replace_after_s', 2.0).value  # target unseen this long -> new one may take over
        self.drop_unseen_s = dp('drop_unseen_s', 3.0).value      # in camera view, not occluded, no YOLO hit -> drop
        self.view_range = dp('view_range', 2.0).value            # m: only judge "should be visible" this close
        self.last_cam_tick = None

        from ultralytics import YOLO  # imported here so a missing install fails loudly
        self.model = YOLO(model)
        self.get_logger().info(f'YOLO model {model} loaded')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.img = self.K = self.Kinv = self.scan = self.depth = None
        self.create_subscription(CompressedImage if self.compressed else Image, image_topic,
                                 self._on_img, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic, lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        if depth_topic:
            self.create_subscription(Image, depth_topic, lambda m: setattr(self, 'depth', m), qos_profile_sensor_data)

        # /objects/detections = everything the mapper knows (input of target_mission on the robot).
        # target_mission republishes only the targets the robot has REACHED on /objects/json (= the drone).
        self.json_pub = self.create_publisher(String, dp('json_topic', '/objects/detections').value, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/objects/markers', 10)
        self.dbg_pub = self.create_publisher(Image, '/objects/debug_image', 2)

        self.objects = []
        self.next_id = 0
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(0.2, self._track_and_publish)   # lidar tracking + output, 5 Hz, camera or not

    def _on_img(self, m):
        self.img = m

    def _on_info(self, m):
        if self.K is None and m.k[0] > 0 and m.k[4] > 0:
            self._set_K(np.array(m.k, dtype=np.float64).reshape(3, 3), 'camera_info')

    def _set_K(self, K, src):
        self.K, self.Kinv = K, np.linalg.inv(K)
        self.get_logger().info(f'camera intrinsics from {src}: fx={K[0, 0]:.1f} fy={K[1, 1]:.1f} '
                               f'cx={K[0, 2]:.1f} cy={K[1, 2]:.1f}')

    def _K_from_fov(self, w, h):
        f = (w / 2.0) / math.tan(self.hfov / 2.0)
        self._set_K(np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1.0]]),
                    f'hfov={math.degrees(self.hfov):.1f}deg (uncalibrated)')

    def _lookup(self, target, source):
        try:
            return tf_to_matrix(self.tf_buffer.lookup_transform(target, source, Time()))
        except TransformException as e:
            self.get_logger().warn(f'TF {target}<-{source}: {e}', throttle_duration_sec=2.0)
            return None

    # ---------------- depth sources ----------------
    def _lidar_points_cam(self, cam_frame):
        s = self.scan
        if s is None:
            return None
        T = self._lookup(cam_frame, s.header.frame_id)
        if T is None:
            return None
        r = np.asarray(s.ranges, dtype=np.float64)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r > s.range_min) & (r < min(s.range_max, self.max_range))
        pts = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(ok.sum()), np.ones(ok.sum())])
        pc = (T @ pts)[:3].T
        return pc[pc[:, 2] > 0.15]  # in front of the camera

    def _depth_from_image(self, depth, u1, v1, u2, v2):
        h, w = depth.shape
        du, dv = (u2 - u1) * 0.25, (v2 - v1) * 0.25
        patch = depth[int(max(0, v1 + dv)):int(min(h, v2 - dv)), int(max(0, u1 + du)):int(min(w, u2 - du))]
        vals = patch[np.isfinite(patch) & (patch > 0.15) & (patch < self.max_range)]
        if vals.size < 20:
            return None
        return float(np.percentile(vals, 30))  # nearer side of the object, robust to background

    def _depth_from_lidar(self, pc, u1, u2, prior=None):
        """Lidar points in the box's column band. With a prior (floor-contact depth), only points near
        it count: a clear bottle lets the laser through and the band would otherwise hit the wall behind."""
        if pc is None or len(pc) == 0:
            return None
        u = self.K[0, 0] * pc[:, 0] / pc[:, 2] + self.K[0, 2]
        m = 0.15 * (u2 - u1)
        z = pc[(u > u1 + m) & (u < u2 - m), 2]
        if prior is not None:
            z = z[(z > 0.7 * prior - 0.15) & (z < 1.3 * prior + 0.15)]
        if z.size < 2:
            return None
        z0 = np.percentile(z, 20)
        return float(np.median(z[z < z0 + 0.2]))  # closest cluster in the column band

    def _depth_from_ground(self, T_map_cam, uc, v2):
        ray = self.Kinv @ np.array([uc, v2, 1.0])
        o, d = T_map_cam[:3, 3], T_map_cam[:3, :3] @ ray
        if d[2] > -0.05:
            return None  # ray doesn't hit the floor in front of us
        t = -o[2] / d[2]
        return float(t) if 0.2 < t < self.max_range else None  # ray has z=1, so t == optical depth

    # ---------------- main loop ----------------
    def _tick(self):
        msg = self.img
        if msg is None:
            return
        self.img = None
        try:
            bgr = compressed_to_np(msg) if self.compressed else image_to_np(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return
        if self.K is None:
            self._K_from_fov(bgr.shape[1], bgr.shape[0])
        cam_frame = self.camera_frame_override or msg.header.frame_id
        T_map_cam = self._lookup(self.map_frame, cam_frame)
        if T_map_cam is None:
            return

        res = self.model.predict(bgr, conf=self.conf, verbose=False, device=self.device or None)[0]
        pc = self._lidar_points_cam(cam_frame)
        depth = None
        if self.depth is not None:
            try:
                depth = image_to_np(self.depth)
            except ValueError:
                depth = None

        now = time.monotonic()
        matched = set()
        fx, fy = self.K[0, 0], self.K[1, 1]
        boxes = res.boxes
        for (u1, v1, u2, v2), ci, cf in zip(boxes.xyxy.cpu().numpy(), boxes.cls.cpu().numpy(), boxes.conf.cpu().numpy()):
            name = res.names[int(ci)]
            if (self.classes and name not in self.classes) or name in self.ignore:
                continue
            uc, vc = 0.5 * (u1 + u2), 0.5 * (v1 + v2)
            Z, method = None, None
            # floor contact of the box bottom = depth prior (invalid if the box is cut off at the image bottom)
            Zg = self._depth_from_ground(T_map_cam, uc, v2) if v2 < bgr.shape[0] - 4 else None
            if depth is not None:
                Z, method = self._depth_from_image(depth, u1, v1, u2, v2), 'depth'
            if Z is None:
                Z, method = self._depth_from_lidar(pc, u1, u2, Zg), 'lidar'
            if Z is None:
                Z, method = Zg, 'ground'
            if Z is None:
                continue
            w_m, h_m = (u2 - u1) * Z / fx, (v2 - v1) * Z / fy
            Z += min(0.5 * w_m, 0.25)  # front face -> rough centre
            p_cam = Z * (self.Kinv @ np.array([uc, vc, 1.0]))
            p_map = (T_map_cam @ np.append(p_cam, 1.0))[:3]
            matched.add(id(self._associate(name, p_map, w_m, h_m, float(cf), method, now)))

        dt = 0.0 if self.last_cam_tick is None else min(1.0, now - self.last_cam_tick)
        self.last_cam_tick = now
        self._negative_evidence(matched, T_map_cam, bgr.shape[1], bgr.shape[0], dt)
        if self.single:   # only ONE target per class (see below)
            # The OLDEST confirmed track (the incumbent) stays the target while it is still being seen
            # (camera or lidar). A second "bottle" somewhere else is ignored (false positive / 2nd object).
            # If the incumbent has not been seen for replace_after_s (bottle picked up and moved), the
            # newest confirmed detection takes over.
            for cls in {o['cls'] for o in self.objects}:
                conf_ = [o for o in self.objects if o['cls'] == cls and o['hits'] >= self.min_hits]
                if len(conf_) < 2:
                    continue
                inc = min(conf_, key=lambda o: o['id'])
                if now - max(inc['last'], inc.get('lidar_last', 0.0)) < self.replace_after_s:
                    gone = [o for o in conf_ if o is not inc]
                    for x in gone:
                        self.get_logger().info(f"ignoring 2nd {cls} at ({x['p'][0]:.2f}, {x['p'][1]:.2f}): "
                                               f"target is {inc['id']} at ({inc['p'][0]:.2f}, {inc['p'][1]:.2f})",
                                               throttle_duration_sec=2.0)
                else:
                    newest = max(conf_, key=lambda o: o['last'])
                    gone = [o for o in conf_ if o is not newest]
                    for x in gone:
                        self.get_logger().info(f"{cls} {x['id']} replaced by {newest['id']} (not seen for "
                                               f"{now - max(x['last'], x.get('lidar_last', 0.0)):.1f} s)")
                self.objects = [x for x in self.objects if not any(x is g for g in gone)]
        self.objects = [o for o in self.objects
                        if o['hits'] >= self.min_hits or now - o['last'] < self.forget_s]
        if self.dbg_pub.get_subscription_count() > 0:
            self._publish_debug(res.plot(), msg.header)

    def _associate(self, cls, p, w, h, conf, method, now):
        best, best_d = None, self.assoc_r
        for o in self.objects:
            if o['cls'] != cls:
                continue
            d = np.linalg.norm(o['p'][:2] - p[:2])
            if d < best_d:
                best, best_d = o, d
        if best is None:
            self.objects.append({'id': self.next_id, 'cls': cls, 'p': p.copy(), 'w': w, 'h': h,
                                 'conf': conf, 'hits': 1, 'last': now, 'method': method,
                                 'lidar_last': 0.0, 'lidar_hits': 0, 'miss_t': 0.0})
            self.next_id += 1
            return self.objects[-1]
        k = 0.3
        best['p'] = (1 - k) * best['p'] + k * p
        best['w'] = (1 - k) * best['w'] + k * w
        best['h'] = (1 - k) * best['h'] + k * h
        best['conf'] = max(best['conf'], conf)
        best['hits'] += 1
        best['last'] = now
        best['method'] = method
        best['miss_t'] = 0.0
        return best

    def _negative_evidence(self, matched, T_map_cam, W, H, dt):
        """A confirmed object that SHOULD be visible (in the image, close, not hidden behind a lidar
        obstacle) but YOLO keeps not seeing it is wrong (lidar locked onto something else, or the bottle
        was taken away) -> drop it after drop_unseen_s."""
        T_cam_map = np.linalg.inv(T_map_cam)
        pts, sensor = self._scan_points_map() if self.scan is not None else (None, None)
        keep = []
        for o in self.objects:
            if o['hits'] < self.min_hits or id(o) in matched:
                keep.append(o)
                continue
            pc = (T_cam_map @ np.append(o['p'], 1.0))[:3]
            visible = False
            if 0.3 < pc[2] < self.view_range:
                u, v, _ = self.K @ (pc / pc[2])
                visible = 0.08 * W < u < 0.92 * W and 0.0 < v < H
            if visible and pts is not None and len(pts):   # occluded by something closer?
                ray = o['p'][:2] - sensor
                d = np.linalg.norm(ray)
                if d > 1e-3:
                    ray /= d
                    rel = pts - sensor
                    along = rel @ ray
                    perp = np.abs(rel[:, 0] * ray[1] - rel[:, 1] * ray[0])
                    if np.any((along > 0.15) & (along < d - 0.2) & (perp < 0.08)):
                        visible = False
            if visible:
                o['miss_t'] = o.get('miss_t', 0.0) + dt
                if o['miss_t'] > self.drop_unseen_s:
                    self.get_logger().warn(f"{o['cls']} {o['id']} at ({o['p'][0]:.2f}, {o['p'][1]:.2f}) dropped: "
                                           f"in camera view for {o['miss_t']:.1f} s but YOLO doesn't see it")
                    continue
            keep.append(o)
        self.objects = keep

    # ---------------- lidar tracking ----------------
    def _scan_points_map(self):
        s = self.scan
        if s is None:
            return None, None
        T = self._lookup(self.map_frame, s.header.frame_id)
        if T is None:
            return None, None
        r = np.asarray(s.ranges, dtype=np.float64)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r > s.range_min) & (r < min(s.range_max, self.max_range))
        pts = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(ok.sum()), np.ones(ok.sum())])
        return (T @ pts)[:2].T, T[:2, 3]

    def _lidar_cluster(self, pts, sensor_xy, guess_xy):
        """Small isolated cluster near guess_xy -> estimated object centre, else None."""
        d = np.linalg.norm(pts - guess_xy, axis=1)
        near = pts[d < self.lidar_gate]
        if len(near) < 2:
            return None
        seed = near[np.argmin(np.linalg.norm(near - guess_xy, axis=1))]
        cl = near[np.linalg.norm(near - seed, axis=1) < self.lidar_cluster]
        c = cl.mean(axis=0)
        cl = pts[np.linalg.norm(pts - c, axis=1) < self.lidar_cluster]   # regrow from centroid
        if len(cl) < 2:
            return None
        c = cl.mean(axis=0)
        extent = 2 * np.max(np.linalg.norm(cl - c, axis=1))
        if extent > self.lidar_max_size:
            return None
        # isolation: almost nothing in a ring around it (otherwise it is part of a wall / furniture)
        dc = np.linalg.norm(pts - c, axis=1)
        if np.count_nonzero((dc > self.lidar_cluster + 0.03) & (dc < self.lidar_cluster + 0.18)) > 2:
            return None
        ray = c - sensor_xy
        n = np.linalg.norm(ray)
        return c + (ray / n) * self.obj_radius if n > 1e-6 else c

    def _track_and_publish(self):
        now = time.monotonic()
        if self.lidar_track:
            pts, sensor = self._scan_points_map()
            if pts is not None and len(pts):
                for o in self.objects:
                    if o['hits'] < self.min_hits:
                        continue
                    c = self._lidar_cluster(pts, sensor, o['p'][:2])
                    if c is None:
                        continue
                    k = 0.4
                    o['p'][:2] = (1 - k) * o['p'][:2] + k * c
                    o['lidar_last'] = now
                    o['lidar_hits'] += 1
        self._publish(now)

    @staticmethod
    def _track_state(o, now):
        cam = now - o['last'] < 1.0
        lid = now - o.get('lidar_last', 0.0) < 1.0
        return 'camera+lidar' if cam and lid else 'camera' if cam else 'lidar' if lid else 'lost'

    def _publish(self, now):
        out, markers = [], MarkerArray()
        stamp = self.get_clock().now().to_msg()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for o in self.objects:
            if o['hits'] < self.min_hits:
                continue
            x, y, z = (float(v) for v in o['p'])
            out.append({'id': o['id'], 'cls': o['cls'], 'x': round(x, 3), 'y': round(y, 3), 'z': round(z, 3),
                        'z_top': round(z + o['h'] / 2, 3), 'w': round(o['w'], 3), 'h': round(o['h'], 3),
                        'conf': round(o['conf'], 2), 'hits': o['hits'], 'method': o['method'],
                        'track': self._track_state(o, now), 'lidar_hits': o.get('lidar_hits', 0),
                        'age_s': round(now - max(o['last'], o.get('lidar_last', 0.0)), 1)})
            for kind in (Marker.CUBE, Marker.TEXT_VIEW_FACING):
                mk = Marker()
                mk.header.stamp, mk.header.frame_id = stamp, self.map_frame
                mk.ns = 'objects' if kind == Marker.CUBE else 'labels'
                mk.id, mk.type, mk.action = o['id'], kind, Marker.ADD
                mk.pose.position.x, mk.pose.position.y = x, y
                mk.pose.orientation.w = 1.0
                if kind == Marker.CUBE:
                    mk.pose.position.z = z
                    mk.scale.x = mk.scale.y = max(o['w'], 0.05)
                    mk.scale.z = max(o['h'], 0.05)
                    st = self._track_state(o, now)   # green: camera, blue: lidar only, grey: lost
                    mk.color.r, mk.color.g, mk.color.b, mk.color.a = {
                        'camera+lidar': (0.1, 0.9, 0.3, 0.6), 'camera': (0.1, 0.9, 0.3, 0.6),
                        'lidar': (0.2, 0.5, 1.0, 0.6), 'lost': (0.6, 0.6, 0.6, 0.4)}[st]
                else:
                    mk.pose.position.z = z + o['h'] / 2 + 0.15
                    mk.scale.z = 0.12
                    mk.text = f"{o['id']}:{o['cls']} ({self._track_state(o, now)})"
                    mk.color.r = mk.color.g = mk.color.b = mk.color.a = 1.0
                markers.markers.append(mk)
        self.json_pub.publish(String(data=json.dumps({'stamp': time.time(), 'objects': out})))
        self.marker_pub.publish(markers)

    def _publish_debug(self, bgr, header):
        m = Image()
        m.header = header
        m.height, m.width = bgr.shape[:2]
        m.encoding, m.step = 'bgr8', bgr.shape[1] * 3
        m.data = np.ascontiguousarray(bgr).tobytes()
        self.dbg_pub.publish(m)


def main():
    rclpy.init()
    node = ObjectMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
