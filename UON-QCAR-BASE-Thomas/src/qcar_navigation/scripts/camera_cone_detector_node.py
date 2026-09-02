#!/usr/bin/env python3
"""
camera_cone_detector_node.py  (O2 + O3 vision system)
"""

import rospy
import numpy as np
import cv2
import tf
import tf.transformations as tft

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from cv_bridge import CvBridge

from qcar_navigation.msg import ConeDetection, ConeDetectionArray


# ---------------------------------------------------------------------------
# Colour profile
# ---------------------------------------------------------------------------

class ColourProfile:
    def __init__(self, label, colour_id, hsv_ranges, rviz_rgba):
        self.label = label
        self.colour_id = colour_id
        self.hsv_ranges = hsv_ranges
        self.rviz_rgba = rviz_rgba

    def mask(self, hsv_image):
        m = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        for lo, hi in self.hsv_ranges:
            m |= cv2.inRange(hsv_image, lo, hi)
        return m


def _arr(lo, hi):
    return np.array(lo, dtype=np.uint8), np.array(hi, dtype=np.uint8)


def _make_colour_profiles(params):
    blue = ColourProfile(
        label="blue", colour_id=ConeDetection.BLUE,
        hsv_ranges=[_arr(
            [params.get("blue_h_lo", 100), params.get("blue_s_lo", 80),  params.get("blue_v_lo", 50)],
            [params.get("blue_h_hi", 130), 255, 255])],
        rviz_rgba=(0.1, 0.3, 1.0, 0.9))

    yellow = ColourProfile(
        label="yellow", colour_id=ConeDetection.YELLOW,
        hsv_ranges=[_arr(
            [params.get("yellow_h_lo", 20), params.get("yellow_s_lo", 80), params.get("yellow_v_lo", 80)],
            [params.get("yellow_h_hi", 38), 255, 255])],
        rviz_rgba=(1.0, 0.9, 0.0, 0.9))

    orange = ColourProfile(
        label="orange", colour_id=ConeDetection.ORANGE,
        hsv_ranges=[_arr(
            [params.get("orange_h_lo", 5),  params.get("orange_s_lo", 100), params.get("orange_v_lo", 80)],
            [params.get("orange_h_hi", 20), 255, 255])],
        rviz_rgba=(1.0, 0.5, 0.0, 0.9))

    return [blue, yellow, orange]


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class CameraConeConeDetectorNode:

    MIN_BLOB_AREA_PX = 50
    MAX_BLOB_AREA_PX = 80000
    MAX_ASPECT_RATIO = 4.0
    MIN_DEPTH_POINTS = 15
    MAX_RANGE_M      = 8.0
    MORPH_KERNEL_SIZE = 5

    def __init__(self):
        rospy.init_node("camera_cone_detector_node")

        self.rgb_topic    = rospy.get_param("~rgb_topic",          "/front_camera/image_raw")
        self.depth_topic  = rospy.get_param("~depth_topic",        "/depth_camera/depth/image_raw")
        self.info_topic   = rospy.get_param("~camera_info_topic",  "/front_camera/camera_info")
        self.target_frame = rospy.get_param("~target_frame",       "base_footprint")
        self.camera_frame = rospy.get_param("~camera_frame",       "camera_rgb_optical")
        self.debug        = rospy.get_param("~debug",              True)

        self.min_area  = rospy.get_param("~min_blob_area",    self.MIN_BLOB_AREA_PX)
        self.max_area  = rospy.get_param("~max_blob_area",    self.MAX_BLOB_AREA_PX)
        self.max_aspect= rospy.get_param("~max_aspect_ratio", self.MAX_ASPECT_RATIO)
        self.max_range = rospy.get_param("~max_range_m",      self.MAX_RANGE_M)
        morph_k        = rospy.get_param("~morph_kernel",     self.MORPH_KERNEL_SIZE)

        colour_params = {k: rospy.get_param("~" + k, None)
                         for k in ("blue_h_lo","blue_h_hi","blue_s_lo","blue_v_lo",
                                   "yellow_h_lo","yellow_h_hi","yellow_s_lo","yellow_v_lo",
                                   "orange_h_lo","orange_h_hi","orange_s_lo","orange_v_lo")}
        colour_params = {k: v for k, v in colour_params.items() if v is not None}
        self.colour_profiles = _make_colour_profiles(colour_params)

        self.morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (morph_k, morph_k))

        self.K = None
        self.cam_info_recv = False
        self.tf_listener   = tf.TransformListener()
        self.bridge        = CvBridge()

        # Cached TF (updated via timer — avoids per-frame blocking lookup)
        self.T_cam_to_base = None
        rospy.Timer(rospy.Duration(0.05), self._update_tf)

        # Publishers
        self.pub_detections = rospy.Publisher(
            "/cone_detections_camera", ConeDetectionArray, queue_size=1)
        self.pub_pose_array = rospy.Publisher(
            "/cone_detections_colour", PoseArray, queue_size=1)
        
        self.pub_debug_img  = rospy.Publisher(
            "/camera_cone_debug", Image, queue_size=1)
        

        # Subscribers — no synchronizer, just use latest depth
        self.latest_depth       = None
        self.latest_depth_stamp = rospy.Time(0)

        rospy.Subscriber(self.info_topic,   CameraInfo, self._camera_info_cb, queue_size=1)
        rospy.Subscriber(self.depth_topic,  Image,      self._depth_cb,       queue_size=1)
        rospy.Subscriber(self.rgb_topic,    Image,      self._rgb_cb,         queue_size=1)

        rospy.loginfo("[camera_cone_detector] started  rgb=%s  depth=%s  frame=%s",
                      self.rgb_topic, self.depth_topic, self.camera_frame)

    # ── TF (non-blocking, cached) ─────────────────────────────────────────

    def _update_tf(self, _event):
        try:
            # rospy.Time(0) = latest available — never blocks on exact timestamp
            trans, rot = self.tf_listener.lookupTransform(
                self.target_frame, self.camera_frame, rospy.Time(0))
            T = tft.quaternion_matrix(rot)
            T[0:3, 3] = np.array(trans)
            self.T_cam_to_base = T
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            pass   # keep using last known transform

    # ── Camera info ──────────────────────────────────────────────────────

    def _camera_info_cb(self, msg):
        if self.cam_info_recv:
            return
        self.K = np.array(msg.K).reshape(3, 3)
        self.cam_info_recv = True
        rospy.loginfo("[camera_cone_detector] intrinsics: fx=%.1f fy=%.1f",
                      self.K[0, 0], self.K[1, 1])

    # ── Image subscribers ────────────────────────────────────────────────

    def _depth_cb(self, msg):
        self.latest_depth       = msg
        self.latest_depth_stamp = msg.header.stamp

    def _rgb_cb(self, msg):
        if self.latest_depth is None:
            return
        self._process(msg, self.latest_depth)

    # ── Main processing ──────────────────────────────────────────────────

    def _process(self, rgb_msg, depth_msg):
        if not self.cam_info_recv:
            rospy.logwarn_throttle(5.0, "[camera_cone_detector] waiting for CameraInfo")
            return

        if self.T_cam_to_base is None:
            rospy.logwarn_throttle(5.0,
                "[camera_cone_detector] waiting for TF %s -> %s",
                self.camera_frame, self.target_frame)
            return

        try:
            bgr   = self.bridge.imgmsg_to_cv2(rgb_msg,   "bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32)
        except Exception as e:
            rospy.logerr("[camera_cone_detector] CvBridge: %s", e)
            return

        depth[depth == 0.0] = np.nan

        hsv       = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        debug_bgr = bgr.copy() if self.debug else None
        all_dets  = []

        T = self.T_cam_to_base.copy()   # snapshot

        for profile in self.colour_profiles:
            all_dets.extend(
                self._detect_colour(profile, hsv, depth, T,
                                    rgb_msg.header.stamp, debug_bgr))

        stamp = rgb_msg.header.stamp

        # --- publish ConeDetectionArray ---
        arr = ConeDetectionArray()
        arr.header    = Header(stamp=stamp, frame_id=self.target_frame)
        arr.detections = all_dets
        self.pub_detections.publish(arr)

        # --- publish PoseArray (drop-in for fusion node) ---
        pa = PoseArray()
        pa.header = Header(stamp=stamp, frame_id=self.target_frame)
        for d in all_dets:
            p = Pose()
            p.position.x = d.position.x
            p.position.y = d.position.y
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pub_pose_array.publish(pa)

        # --- publish direction markers ---


        # --- publish debug image ---
        if self.debug and debug_bgr is not None:
            try:
                dbg          = self.bridge.cv2_to_imgmsg(debug_bgr, "bgr8")
                dbg.header   = rgb_msg.header
                self.pub_debug_img.publish(dbg)
            except Exception as e:
                rospy.logerr("[camera_cone_detector] debug publish: %s", e)

        rospy.loginfo_throttle(2.0,
            "[camera_cone_detector] %d detections", len(all_dets))

    # ── Colour detection ─────────────────────────────────────────────────

    def _detect_colour(self, profile, hsv, depth, T, stamp, debug_bgr):
        mask = profile.mask(hsv)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self.morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)

        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        detections = []

        for lid in range(1, n):
            x_tl = stats[lid, cv2.CC_STAT_LEFT]
            y_tl = stats[lid, cv2.CC_STAT_TOP]
            w    = stats[lid, cv2.CC_STAT_WIDTH]
            h    = stats[lid, cv2.CC_STAT_HEIGHT]
            area = stats[lid, cv2.CC_STAT_AREA]

            if area < self.min_area or area > self.max_area:
                continue
            if h == 0 or float(w) / float(h) > self.max_aspect:
                continue

            valid_depth = depth[labels == lid]
            valid_depth = valid_depth[np.isfinite(valid_depth) & (valid_depth > 0.05)]
            if len(valid_depth) < self.MIN_DEPTH_POINTS:
                continue

            z = float(np.median(valid_depth))
            if z > self.max_range or z <= 0.0:
                continue

            fx, fy = self.K[0,0], self.K[1,1]
            cx, cy = self.K[0,2], self.K[1,2]
            u_c = x_tl + w / 2.0
            v_c = y_tl + h / 2.0

            xc = (u_c - cx) * z / fx
            yc = (v_c - cy) * z / fy

            # Direction vector
            d3 = np.array([xc, yc, z])
            dn = np.linalg.norm(d3)
            if dn < 1e-6:
                continue
            d_unit = d3 / dn

            # Transform to base_footprint
            pb  = T @ np.array([xc, yc, z, 1.0])
            dbn = T @ np.array([d_unit[0], d_unit[1], d_unit[2], 0.0])
            db  = dbn[:3]
            dbl = np.linalg.norm(db)
            if dbl > 1e-6:
                db /= dbl

            # Covariance
            mad      = float(np.median(np.abs(valid_depth - z)))
            sigma_z  = max(mad * 1.4826, 0.02)
            sigma_lat= max((w / 4.0) * z / fx, 0.02)

            det = ConeDetection()
            det.header   = Header(stamp=stamp, frame_id=self.target_frame)
            det.colour   = profile.colour_id
            det.direction.x = float(db[0])
            det.direction.y = float(db[1])
            det.direction.z = float(db[2])
            det.range        = z
            det.position.x   = float(pb[0])
            det.position.y   = float(pb[1])
            det.position.z   = 0.0
            det.position_covariance = [sigma_z**2, 0.0, 0.0, sigma_lat**2]
            det.bbox_x = int(x_tl); det.bbox_y = int(y_tl)
            det.bbox_w = int(w);    det.bbox_h = int(h)
            det.pixel_count = int(area)
            detections.append(det)

            if debug_bgr is not None:
                BGR = {ConeDetection.BLUE:(200,50,50),
                       ConeDetection.YELLOW:(30,200,200),
                       ConeDetection.ORANGE:(30,120,240)}
                c = BGR.get(profile.colour_id, (255,255,255))
                cv2.rectangle(debug_bgr,(int(x_tl),int(y_tl)),
                              (int(x_tl+w),int(y_tl+h)), c, 2)
                cv2.putText(debug_bgr, f"{profile.label} {z:.1f}m",
                            (int(x_tl), max(int(y_tl)-5,0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, c, 1, cv2.LINE_AA)
                cv2.ellipse(debug_bgr,(int(u_c),int(v_c)),
                            (max(int(w/4),2), max(int(sigma_z*fy/z),2)),
                            0, 0, 360, c, 1)

        return detections

    # ── Direction markers ────────────────────────────────────────────────

    
if __name__ == "__main__":
    try:
        CameraConeConeDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass