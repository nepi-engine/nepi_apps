#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment bstab.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com


import os
import time
import sys
import copy
import threading
from collections import deque

import numpy as np
import cv2
import open3d as o3d

from std_msgs.msg import UInt8, Empty, String, Bool, Float32, Int32
from sensor_msgs.msg import Image
from nepi_app_stereo_depth.msg import StereoDepthAppStatus

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.data_if import DepthMapIF, PointcloudIF


#########################################
# Factory Control Values
#########################################

CALIB_DIR = "/mnt/nepi_storage/user_cfg"
CALIB_FILE = os.path.join(CALIB_DIR, "app_stereo_depth_calibration.npz")

# Calibration capture states
CALIB_IDLE = "idle"
CALIB_CAPTURING = "capturing"
CALIB_COMPUTING = "computing"

# Minimum number of good checkerboard captures required to compute a calibration
MIN_CALIB_CAPTURES = 6

PROCESS_RATE_HZ = 5.0
UPDATE_TOPICS_RATE_HZ = 1.0

#########################################
# Node Class
#########################################

class NepiStereoDepthApp(object):

  node_if = None

  data_products = ["depth_map", "pointcloud"]

  # Factory parameter defaults
  FACTORY_LEFT_TOPIC = "None"
  FACTORY_RIGHT_TOPIC = "None"
  FACTORY_SYNC_TOLERANCE_S = 0.05
  FACTORY_BASELINE_M = 0.06
  FACTORY_CB_COLS = 9          # inner corners across
  FACTORY_CB_ROWS = 6          # inner corners down
  FACTORY_CB_SQUARE_M = 0.025
  FACTORY_CAPTURE_TARGET = 15
  FACTORY_MIN_DISPARITY = 0
  FACTORY_NUM_DISPARITIES = 128  # must be > 0 and divisible by 16
  FACTORY_BLOCK_SIZE = 7         # must be odd, >= 1
  FACTORY_UNIQUENESS_RATIO = 10
  FACTORY_SPECKLE_WINDOW_SIZE = 100
  FACTORY_SPECKLE_RANGE = 2
  FACTORY_MIN_RANGE_M = 0.2
  FACTORY_MAX_RANGE_M = 20.0

  # Runtime state
  available_image_topics = []

  left_topic = "None"
  right_topic = "None"
  left_sub = None
  right_sub = None

  sync_tolerance_s = FACTORY_SYNC_TOLERANCE_S
  baseline_m = FACTORY_BASELINE_M
  cb_cols = FACTORY_CB_COLS
  cb_rows = FACTORY_CB_ROWS
  cb_square_m = FACTORY_CB_SQUARE_M
  capture_target = FACTORY_CAPTURE_TARGET
  min_disparity = FACTORY_MIN_DISPARITY
  num_disparities = FACTORY_NUM_DISPARITIES
  block_size = FACTORY_BLOCK_SIZE
  uniqueness_ratio = FACTORY_UNIQUENESS_RATIO
  speckle_window_size = FACTORY_SPECKLE_WINDOW_SIZE
  speckle_range = FACTORY_SPECKLE_RANGE
  min_range_m = FACTORY_MIN_RANGE_M
  max_range_m = FACTORY_MAX_RANGE_M

  running = False
  last_pair_dt_s = 0.0
  status_message = "Initializing"

  # Calibration runtime state
  calib_state = CALIB_IDLE
  calib_objpoints = []
  calib_imgpoints_left = []
  calib_imgpoints_right = []
  calibrated = False
  last_rms_reproj_error = 0.0
  image_width = 0
  image_height = 0

  # Loaded rectification data
  calib_data = None      # dict of numpy arrays
  map_left_x = None
  map_left_y = None
  map_right_x = None
  map_right_y = None
  Q = None

  # Output interfaces
  depth_map_if = None
  pointcloud_if = None

  DEFAULT_NODE_NAME = "app_stereo_depth"  # Can be overwritten by launch command

  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
    self.class_name = type(self).__name__
    self.base_namespace = nepi_sdk.get_base_namespace()
    self.node_name = nepi_sdk.get_node_name()
    self.node_namespace = nepi_sdk.get_node_namespace()
    self.data_products_list = self.data_products

    ##############################
    # Create Msg Class
    self.msg_if = MsgIF(log_name=self.class_name)
    self.msg_if.pub_info("Starting IF Initialization Processes")

    ##############################
    # Initialize Class Variables
    self.frame_lock = threading.Lock()
    self.proc_lock = threading.Lock()
    self.left_frame = None    # dict: {'img': cv2, 'ts': sec}
    # Keep a short buffer of recent right frames for nearest-timestamp matching
    self.right_frames = deque(maxlen=10)

    ##############################
    ### Setup Node

    # Configs Config Dict ####################
    self.CFGS_DICT = {
        'init_callback': self.initCb,
        'reset_callback': self.resetCb,
        'factory_reset_callback': self.factoryResetCb,
        'init_configs': True,
        'namespace': self.node_namespace
    }

    # Params Config Dict ####################
    self.PARAMS_DICT = {
        'left_topic': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_LEFT_TOPIC
        },
        'right_topic': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_RIGHT_TOPIC
        },
        'sync_tolerance_s': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_SYNC_TOLERANCE_S
        },
        'baseline_m': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_BASELINE_M
        },
        'cb_cols': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_CB_COLS
        },
        'cb_rows': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_CB_ROWS
        },
        'cb_square_m': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_CB_SQUARE_M
        },
        'capture_target': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_CAPTURE_TARGET
        },
        'min_disparity': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_MIN_DISPARITY
        },
        'num_disparities': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_NUM_DISPARITIES
        },
        'block_size': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_BLOCK_SIZE
        },
        'uniqueness_ratio': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_UNIQUENESS_RATIO
        },
        'speckle_window_size': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_SPECKLE_WINDOW_SIZE
        },
        'speckle_range': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_SPECKLE_RANGE
        },
        'min_range_m': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_MIN_RANGE_M
        },
        'max_range_m': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_MAX_RANGE_M
        }
    }

    # Publishers Config Dict ####################
    self.PUBS_DICT = {
        'status_pub': {
            'namespace': self.node_namespace,
            'topic': 'status',
            'msg': StereoDepthAppStatus,
            'qsize': 1,
            'latch': True
        }
    }

    # Subscribers Config Dict ####################
    self.SUBS_DICT = {
        'set_left_topic': {
            'namespace': self.node_namespace,
            'topic': 'set_left_topic',
            'msg': String,
            'qsize': 10,
            'callback': self.setLeftTopicCb,
            'callback_args': ()
        },
        'set_right_topic': {
            'namespace': self.node_namespace,
            'topic': 'set_right_topic',
            'msg': String,
            'qsize': 10,
            'callback': self.setRightTopicCb,
            'callback_args': ()
        },
        'set_sync_tolerance': {
            'namespace': self.node_namespace,
            'topic': 'set_sync_tolerance',
            'msg': Float32,
            'qsize': 10,
            'callback': self.setSyncToleranceCb,
            'callback_args': ()
        },
        'set_baseline': {
            'namespace': self.node_namespace,
            'topic': 'set_baseline',
            'msg': Float32,
            'qsize': 10,
            'callback': self.setBaselineCb,
            'callback_args': ()
        },
        'set_checkerboard_cols': {
            'namespace': self.node_namespace,
            'topic': 'set_checkerboard_cols',
            'msg': Int32,
            'qsize': 10,
            'callback': self.setCheckerboardColsCb,
            'callback_args': ()
        },
        'set_checkerboard_rows': {
            'namespace': self.node_namespace,
            'topic': 'set_checkerboard_rows',
            'msg': Int32,
            'qsize': 10,
            'callback': self.setCheckerboardRowsCb,
            'callback_args': ()
        },
        'set_square_size': {
            'namespace': self.node_namespace,
            'topic': 'set_square_size',
            'msg': Float32,
            'qsize': 10,
            'callback': self.setSquareSizeCb,
            'callback_args': ()
        },
        'set_capture_target': {
            'namespace': self.node_namespace,
            'topic': 'set_capture_target',
            'msg': Int32,
            'qsize': 10,
            'callback': self.setCaptureTargetCb,
            'callback_args': ()
        },
        'set_num_disparities': {
            'namespace': self.node_namespace,
            'topic': 'set_num_disparities',
            'msg': Int32,
            'qsize': 10,
            'callback': self.setNumDisparitiesCb,
            'callback_args': ()
        },
        'set_block_size': {
            'namespace': self.node_namespace,
            'topic': 'set_block_size',
            'msg': Int32,
            'qsize': 10,
            'callback': self.setBlockSizeCb,
            'callback_args': ()
        },
        'set_min_range': {
            'namespace': self.node_namespace,
            'topic': 'set_min_range',
            'msg': Float32,
            'qsize': 10,
            'callback': self.setMinRangeCb,
            'callback_args': ()
        },
        'set_max_range': {
            'namespace': self.node_namespace,
            'topic': 'set_max_range',
            'msg': Float32,
            'qsize': 10,
            'callback': self.setMaxRangeCb,
            'callback_args': ()
        },
        'start_calibration': {
            'namespace': self.node_namespace,
            'topic': 'start_calibration',
            'msg': Empty,
            'qsize': 10,
            'callback': self.startCalibrationCb,
            'callback_args': ()
        },
        'capture_calibration': {
            'namespace': self.node_namespace,
            'topic': 'capture_calibration',
            'msg': Empty,
            'qsize': 10,
            'callback': self.captureCalibrationCb,
            'callback_args': ()
        },
        'compute_calibration': {
            'namespace': self.node_namespace,
            'topic': 'compute_calibration',
            'msg': Empty,
            'qsize': 10,
            'callback': self.computeCalibrationCb,
            'callback_args': ()
        },
        'cancel_calibration': {
            'namespace': self.node_namespace,
            'topic': 'cancel_calibration',
            'msg': Empty,
            'qsize': 10,
            'callback': self.cancelCalibrationCb,
            'callback_args': ()
        },
        'start_pub': {
            'namespace': self.node_namespace,
            'topic': 'start_pub',
            'msg': Empty,
            'qsize': 10,
            'callback': self.startPubCb,
            'callback_args': ()
        },
        'stop_pub': {
            'namespace': self.node_namespace,
            'topic': 'stop_pub',
            'msg': Empty,
            'qsize': 10,
            'callback': self.stopPubCb,
            'callback_args': ()
        }
    }

    # Create Node Class ####################
    self.node_if = NodeClassIF(
        configs_dict=self.CFGS_DICT,
        params_dict=self.PARAMS_DICT,
        pubs_dict=self.PUBS_DICT,
        subs_dict=self.SUBS_DICT
    )

    ready = self.node_if.wait_for_ready()

    ##############################
    # Setup Output Interfaces (auto-create colorized image + rendered pointcloud image + save data)
    self.depth_map_if = DepthMapIF(
        namespace=self.node_namespace,
        data_product='depth_map',
        data_source_description='stereo_depth',
        data_ref_description='sensor',
        pub_image=True,
        log_name_list=[self.node_name]
    )
    self.pointcloud_if = PointcloudIF(
        namespace=self.node_namespace,
        data_product='pointcloud',
        data_source_description='stereo_depth',
        data_ref_description='sensor',
        pub_image=True,
        log_name_list=[self.node_name]
    )

    ##############################
    # Load params and any saved calibration
    self.initCb(do_updates=True)
    self._loadCalibration()

    time.sleep(1)
    nepi_sdk.start_timer_process(1.0, self.updateTopicsCb, oneshot=True)
    nepi_sdk.start_timer_process(float(1) / PROCESS_RATE_HZ, self.processCb, oneshot=True)
    nepi_sdk.start_timer_process(1.0, self.statusPublishCb)
    time.sleep(1)

    self.msg_if.pub_info("Initialization Complete")

    nepi_sdk.on_shutdown(self.cleanup_actions)
    nepi_sdk.spin()

  ###################
  ## Camera Selection Callbacks

  def setLeftTopicCb(self, msg):
    self.msg_if.pub_info("Set left topic: " + str(msg.data))
    self.left_topic = msg.data
    self._subscribeLeft(self.left_topic)
    if self.node_if is not None:
      self.node_if.set_param('left_topic', self.left_topic)
      self.node_if.save_config()
    self.publish_status()

  def setRightTopicCb(self, msg):
    self.msg_if.pub_info("Set right topic: " + str(msg.data))
    self.right_topic = msg.data
    self._subscribeRight(self.right_topic)
    if self.node_if is not None:
      self.node_if.set_param('right_topic', self.right_topic)
      self.node_if.save_config()
    self.publish_status()

  def setSyncToleranceCb(self, msg):
    val = float(msg.data)
    if val > 0.0:
      self.sync_tolerance_s = val
      self._saveParam('sync_tolerance_s', val)
    self.publish_status()

  ###################
  ## Calibration Parameter Callbacks

  def setBaselineCb(self, msg):
    val = float(msg.data)
    if val > 0.0:
      self.baseline_m = val
      self._saveParam('baseline_m', val)
    self.publish_status()

  def setCheckerboardColsCb(self, msg):
    val = int(msg.data)
    if val > 1:
      self.cb_cols = val
      self._saveParam('cb_cols', val)
    self.publish_status()

  def setCheckerboardRowsCb(self, msg):
    val = int(msg.data)
    if val > 1:
      self.cb_rows = val
      self._saveParam('cb_rows', val)
    self.publish_status()

  def setSquareSizeCb(self, msg):
    val = float(msg.data)
    if val > 0.0:
      self.cb_square_m = val
      self._saveParam('cb_square_m', val)
    self.publish_status()

  def setCaptureTargetCb(self, msg):
    val = int(msg.data)
    if val >= MIN_CALIB_CAPTURES:
      self.capture_target = val
      self._saveParam('capture_target', val)
    self.publish_status()

  ###################
  ## Disparity / Range Parameter Callbacks

  def setNumDisparitiesCb(self, msg):
    val = int(msg.data)
    # StereoSGBM requires num_disparities > 0 and divisible by 16
    val = max(16, int(round(val / 16.0)) * 16)
    self.num_disparities = val
    self._saveParam('num_disparities', val)
    self.publish_status()

  def setBlockSizeCb(self, msg):
    val = int(msg.data)
    # StereoSGBM block size must be odd and >= 1
    if val < 1:
      val = 1
    if val % 2 == 0:
      val += 1
    self.block_size = val
    self._saveParam('block_size', val)
    self.publish_status()

  def setMinRangeCb(self, msg):
    val = float(msg.data)
    if val >= 0.0 and val < self.max_range_m:
      self.min_range_m = val
      self._saveParam('min_range_m', val)
    self.publish_status()

  def setMaxRangeCb(self, msg):
    val = float(msg.data)
    if val > self.min_range_m:
      self.max_range_m = val
      self._saveParam('max_range_m', val)
    self.publish_status()

  ###################
  ## Calibration Mode Command Callbacks

  def startCalibrationCb(self, msg):
    with self.proc_lock:
      self.calib_state = CALIB_CAPTURING
      self.calib_objpoints = []
      self.calib_imgpoints_left = []
      self.calib_imgpoints_right = []
    self.status_message = "Calibration capture started"
    self.msg_if.pub_info(self.status_message)
    self.publish_status()

  def captureCalibrationCb(self, msg):
    self._captureCalibrationFrame()
    self.publish_status()

  def computeCalibrationCb(self, msg):
    self._computeCalibration()
    self.publish_status()

  def cancelCalibrationCb(self, msg):
    with self.proc_lock:
      self.calib_state = CALIB_IDLE
      self.calib_objpoints = []
      self.calib_imgpoints_left = []
      self.calib_imgpoints_right = []
    self.status_message = "Calibration cancelled"
    self.msg_if.pub_info(self.status_message)
    self.publish_status()

  ###################
  ## Run Mode Command Callbacks

  def startPubCb(self, msg):
    if self.calibrated == False:
      self.status_message = "Cannot run: no valid calibration loaded"
      self.msg_if.pub_warn(self.status_message)
    else:
      self.running = True
      self.status_message = "Running stereo depth pipeline"
      self.msg_if.pub_info(self.status_message)
    self.publish_status()

  def stopPubCb(self, msg):
    self.running = False
    self.status_message = "Stopped"
    self.msg_if.pub_info(self.status_message)
    self.publish_status()

  ###################
  ## Image Subscriber Callbacks

  def leftImageCb(self, msg):
    cv2_img = self._rosImgToCv2(msg)
    if cv2_img is None:
      return
    ts = nepi_sdk.sec_from_msg_stamp(msg.header.stamp)
    with self.frame_lock:
      self.left_frame = {'img': cv2_img, 'ts': ts}

  def rightImageCb(self, msg):
    cv2_img = self._rosImgToCv2(msg)
    if cv2_img is None:
      return
    ts = nepi_sdk.sec_from_msg_stamp(msg.header.stamp)
    with self.frame_lock:
      self.right_frames.append({'img': cv2_img, 'ts': ts})

  ###################
  ## Timer Callbacks

  def updateTopicsCb(self, timer):
    topics = nepi_sdk.find_topics_by_msg('Image')
    self.available_image_topics = list(topics)
    nepi_sdk.start_timer_process(1.0, self.updateTopicsCb, oneshot=True)

  def processCb(self, timer):
    try:
      self._processPair()
    except Exception as e:
      self.msg_if.pub_warn("Process error: " + str(e), throttle_s=5.0)
    nepi_sdk.start_timer_process(float(1) / PROCESS_RATE_HZ, self.processCb, oneshot=True)

  def statusPublishCb(self, timer):
    self.publish_status()

  #######################
  ### Frame Pairing Helpers

  def _getMatchedPair(self):
    # Returns (left_img, right_img, timestamp) for the nearest-timestamp match
    # within sync tolerance, or None if no valid pair is available.
    with self.frame_lock:
      if self.left_frame is None or len(self.right_frames) == 0:
        return None
      left = self.left_frame
      right_list = list(self.right_frames)
    best = None
    best_dt = None
    for right in right_list:
      dt = abs(left['ts'] - right['ts'])
      if best_dt is None or dt < best_dt:
        best_dt = dt
        best = right
    if best is None:
      return None
    self.last_pair_dt_s = best_dt
    if best_dt > self.sync_tolerance_s:
      return None
    return left['img'], best['img'], left['ts']

  #######################
  ### Calibration Helpers

  def _captureCalibrationFrame(self):
    if self.calib_state != CALIB_CAPTURING:
      self.status_message = "Not in calibration capture mode"
      self.msg_if.pub_warn(self.status_message)
      return
    pair = self._getMatchedPair()
    if pair is None:
      self.status_message = "No synchronized frame pair available to capture"
      self.msg_if.pub_warn(self.status_message)
      return
    left_img, right_img, _ = pair
    gray_left = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

    self.image_width = gray_left.shape[1]
    self.image_height = gray_left.shape[0]

    pattern = (self.cb_cols, self.cb_rows)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    found_l, corners_l = cv2.findChessboardCorners(gray_left, pattern, flags)
    found_r, corners_r = cv2.findChessboardCorners(gray_right, pattern, flags)

    if not (found_l and found_r):
      self.status_message = "Checkerboard not found in both frames"
      self.msg_if.pub_warn(self.status_message)
      return

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners_l = cv2.cornerSubPix(gray_left, corners_l, (11, 11), (-1, -1), criteria)
    corners_r = cv2.cornerSubPix(gray_right, corners_r, (11, 11), (-1, -1), criteria)

    # Build the object-point grid in metric units
    objp = np.zeros((self.cb_cols * self.cb_rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:self.cb_cols, 0:self.cb_rows].T.reshape(-1, 2)
    objp *= self.cb_square_m

    with self.proc_lock:
      self.calib_objpoints.append(objp)
      self.calib_imgpoints_left.append(corners_l)
      self.calib_imgpoints_right.append(corners_r)
      count = len(self.calib_objpoints)

    self.status_message = "Captured calibration frame " + str(count)
    self.msg_if.pub_info(self.status_message)

  def _computeCalibration(self):
    with self.proc_lock:
      n = len(self.calib_objpoints)
      objpoints = list(self.calib_objpoints)
      imgpoints_left = list(self.calib_imgpoints_left)
      imgpoints_right = list(self.calib_imgpoints_right)
      self.calib_state = CALIB_COMPUTING

    if n < MIN_CALIB_CAPTURES:
      self.status_message = "Need at least " + str(MIN_CALIB_CAPTURES) + " captures, have " + str(n)
      self.msg_if.pub_warn(self.status_message)
      with self.proc_lock:
        self.calib_state = CALIB_CAPTURING
      return

    if self.image_width == 0 or self.image_height == 0:
      self.status_message = "Unknown image size for calibration"
      self.msg_if.pub_warn(self.status_message)
      with self.proc_lock:
        self.calib_state = CALIB_CAPTURING
      return

    image_size = (self.image_width, self.image_height)

    try:
      # Per-lens intrinsic calibration
      rms_l, mtxL, distL, _, _ = cv2.calibrateCamera(
          objpoints, imgpoints_left, image_size, None, None)
      rms_r, mtxR, distR, _, _ = cv2.calibrateCamera(
          objpoints, imgpoints_right, image_size, None, None)

      # Stereo calibration (fix intrinsics from the per-lens step)
      criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
      stereo_flags = cv2.CALIB_FIX_INTRINSIC
      rms_stereo, mtxL, distL, mtxR, distR, R, T, E, F = cv2.stereoCalibrate(
          objpoints, imgpoints_left, imgpoints_right,
          mtxL, distL, mtxR, distR, image_size,
          criteria=criteria, flags=stereo_flags)

      # Rectification -> R1,R2,P1,P2,Q and validity ROIs
      R1, R2, P1, P2, Q, roi_l, roi_r = cv2.stereoRectify(
          mtxL, distL, mtxR, distR, image_size, R, T,
          flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)

    except Exception as e:
      self.status_message = "Calibration computation failed: " + str(e)
      self.msg_if.pub_warn(self.status_message)
      with self.proc_lock:
        self.calib_state = CALIB_CAPTURING
      return

    computed_baseline = float(np.linalg.norm(T))

    calib_data = {
        'mtxL': mtxL, 'distL': distL,
        'mtxR': mtxR, 'distR': distR,
        'R': R, 'T': T,
        'R1': R1, 'R2': R2, 'P1': P1, 'P2': P2, 'Q': Q,
        'image_size': np.array(image_size, dtype=np.int32),
        'rms_stereo': np.array([rms_stereo], dtype=np.float64),
        'computed_baseline_m': np.array([computed_baseline], dtype=np.float64),
        'measured_baseline_m': np.array([self.baseline_m], dtype=np.float64)
    }

    self._saveCalibration(calib_data)
    self._applyCalibration(calib_data)

    self.last_rms_reproj_error = float(rms_stereo)
    with self.proc_lock:
      self.calib_state = CALIB_IDLE

    self.status_message = ("Calibration complete. RMS=" + "{:.3f}".format(rms_stereo) +
                           " computed baseline=" + "{:.3f}m".format(computed_baseline) +
                           " (measured=" + "{:.3f}m)".format(self.baseline_m))
    self.msg_if.pub_info(self.status_message)

  def _applyCalibration(self, calib_data):
    image_size = tuple(int(v) for v in calib_data['image_size'])
    map_lx, map_ly = cv2.initUndistortRectifyMap(
        calib_data['mtxL'], calib_data['distL'], calib_data['R1'],
        calib_data['P1'], image_size, cv2.CV_32FC1)
    map_rx, map_ry = cv2.initUndistortRectifyMap(
        calib_data['mtxR'], calib_data['distR'], calib_data['R2'],
        calib_data['P2'], image_size, cv2.CV_32FC1)
    self.map_left_x = map_lx
    self.map_left_y = map_ly
    self.map_right_x = map_rx
    self.map_right_y = map_ry
    self.Q = calib_data['Q']
    self.image_width = image_size[0]
    self.image_height = image_size[1]
    self.calib_data = calib_data
    self.calibrated = True

  def _saveCalibration(self, calib_data):
    try:
      if not os.path.isdir(CALIB_DIR):
        os.makedirs(CALIB_DIR)
      np.savez(CALIB_FILE, **calib_data)
      self.msg_if.pub_info("Saved calibration to " + CALIB_FILE)
    except Exception as e:
      self.msg_if.pub_warn("Failed to save calibration: " + str(e))

  def _loadCalibration(self):
    if not os.path.isfile(CALIB_FILE):
      self.msg_if.pub_info("No saved calibration found at " + CALIB_FILE)
      return
    try:
      loaded = np.load(CALIB_FILE)
      calib_data = {k: loaded[k] for k in loaded.files}
      self._applyCalibration(calib_data)
      if 'rms_stereo' in calib_data:
        self.last_rms_reproj_error = float(calib_data['rms_stereo'][0])
      self.status_message = "Loaded saved calibration"
      self.msg_if.pub_info(self.status_message)
    except Exception as e:
      self.msg_if.pub_warn("Failed to load calibration: " + str(e))

  #######################
  ### Run-mode Pipeline

  def _processPair(self):
    if self.running == False or self.calibrated == False:
      return
    pair = self._getMatchedPair()
    if pair is None:
      return
    left_img, right_img, timestamp = pair

    # Rectify both frames with the stored maps
    rect_left = cv2.remap(left_img, self.map_left_x, self.map_left_y, cv2.INTER_LINEAR)
    rect_right = cv2.remap(right_img, self.map_right_x, self.map_right_y, cv2.INTER_LINEAR)

    gray_left = cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)

    # Disparity with StereoSGBM
    matcher = cv2.StereoSGBM_create(
        minDisparity=self.min_disparity,
        numDisparities=self.num_disparities,
        blockSize=self.block_size,
        P1=8 * self.block_size * self.block_size,
        P2=32 * self.block_size * self.block_size,
        uniquenessRatio=self.uniqueness_ratio,
        speckleWindowSize=self.speckle_window_size,
        speckleRange=self.speckle_range,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

    # StereoSGBM returns fixed-point disparity scaled by 16
    disparity = matcher.compute(gray_left, gray_right).astype(np.float32) / 16.0

    # Reproject to 3D using the Q matrix (points in the rectified left-camera frame, meters)
    points_3d = cv2.reprojectImageTo3D(disparity, self.Q)

    # Valid pixels: disparity above the minimum and finite
    valid = (disparity > float(self.min_disparity)) & np.isfinite(points_3d[:, :, 2])
    depth = points_3d[:, :, 2]
    valid = valid & (depth >= self.min_range_m) & (depth <= self.max_range_m)

    # Depth map: float32 meters, invalid pixels set to NaN
    depth_map = np.where(valid, depth, np.nan).astype(np.float32)

    self.depth_map_if.publish_np_depth_map(
        depth_map,
        encoding='32FC1',
        min_range_m=self.min_range_m,
        max_range_m=self.max_range_m,
        timestamp=timestamp)

    # Point cloud from valid 3D points, colored with the rectified left image
    self._publishPointcloud(points_3d, rect_left, valid, timestamp)

  def _publishPointcloud(self, points_3d, rect_left_bgr, valid, timestamp):
    xyz = points_3d[valid].reshape(-1, 3)
    if xyz.shape[0] == 0:
      return
    o3d_pc = o3d.geometry.PointCloud()
    o3d_pc.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    # OpenCV frames are BGR; convert to RGB float [0,1] for Open3D
    rgb = rect_left_bgr[valid].reshape(-1, 3)[:, ::-1].astype(np.float64) / 255.0
    o3d_pc.colors = o3d.utility.Vector3dVector(rgb)
    self.pointcloud_if.publish_o3d_pc(
        o3d_pc,
        timestamp=timestamp,
        min_range_m=self.min_range_m,
        max_range_m=self.max_range_m,
        frame_id='sensor')

  #######################
  ### Subscription Helpers

  def _subscribeLeft(self, topic):
    if self.left_sub is not None:
      try:
        self.left_sub.unregister()
      except Exception:
        pass
      self.left_sub = None
    with self.frame_lock:
      self.left_frame = None
    if topic is not None and topic != "None" and topic != "":
      self.left_sub = nepi_sdk.create_subscriber(topic, Image, self.leftImageCb, queue_size=1)

  def _subscribeRight(self, topic):
    if self.right_sub is not None:
      try:
        self.right_sub.unregister()
      except Exception:
        pass
      self.right_sub = None
    with self.frame_lock:
      self.right_frames.clear()
    if topic is not None and topic != "None" and topic != "":
      self.right_sub = nepi_sdk.create_subscriber(topic, Image, self.rightImageCb, queue_size=1)

  def _rosImgToCv2(self, msg):
    try:
      cv2_img = nepi_img.rosimg_to_cv2img(msg, encoding='bgr8')
      return cv2_img
    except Exception as e:
      self.msg_if.pub_warn("Failed to convert image: " + str(e), throttle_s=5.0)
      return None

  def _saveParam(self, name, value):
    if self.node_if is not None:
      self.node_if.set_param(name, value)
      self.node_if.save_config()

  def _leftConnected(self):
    return self.left_topic != "None" and self.left_topic in self.available_image_topics

  def _rightConnected(self):
    return self.right_topic != "None" and self.right_topic in self.available_image_topics

  #######################
  ### Config Functions

  def initCb(self, do_updates=False):
    if self.node_if is not None:
      self.left_topic = self.node_if.get_param('left_topic')
      self.right_topic = self.node_if.get_param('right_topic')
      self.sync_tolerance_s = self.node_if.get_param('sync_tolerance_s')
      self.baseline_m = self.node_if.get_param('baseline_m')
      self.cb_cols = self.node_if.get_param('cb_cols')
      self.cb_rows = self.node_if.get_param('cb_rows')
      self.cb_square_m = self.node_if.get_param('cb_square_m')
      self.capture_target = self.node_if.get_param('capture_target')
      self.min_disparity = self.node_if.get_param('min_disparity')
      self.num_disparities = self.node_if.get_param('num_disparities')
      self.block_size = self.node_if.get_param('block_size')
      self.uniqueness_ratio = self.node_if.get_param('uniqueness_ratio')
      self.speckle_window_size = self.node_if.get_param('speckle_window_size')
      self.speckle_range = self.node_if.get_param('speckle_range')
      self.min_range_m = self.node_if.get_param('min_range_m')
      self.max_range_m = self.node_if.get_param('max_range_m')
    if do_updates == True:
      self._subscribeLeft(self.left_topic)
      self._subscribeRight(self.right_topic)
    self.publish_status()

  def resetCb(self, do_updates=True):
    self.msg_if.pub_warn("Reseting")
    self.initCb(do_updates=do_updates)

  def factoryResetCb(self, do_updates=True):
    self.msg_if.pub_warn("Factory Reseting")
    self.initCb(do_updates=do_updates)

  ###################
  ## Status Publisher

  def publish_status(self):
    """Publish the latched Stereo Depth app status message with all current state."""
    status_msg = StereoDepthAppStatus()
    status_msg.available_image_topics = self.available_image_topics
    status_msg.selected_left_topic = self.left_topic
    status_msg.selected_right_topic = self.right_topic
    status_msg.left_connected = self._leftConnected()
    status_msg.right_connected = self._rightConnected()
    status_msg.sync_tolerance_s = self.sync_tolerance_s
    status_msg.last_pair_dt_s = self.last_pair_dt_s
    status_msg.calibrated = self.calibrated
    status_msg.calibration_file = CALIB_FILE
    status_msg.baseline_m = self.baseline_m
    status_msg.image_width = self.image_width
    status_msg.image_height = self.image_height
    status_msg.last_rms_reproj_error = self.last_rms_reproj_error
    status_msg.calib_state = self.calib_state
    status_msg.capture_count = len(self.calib_objpoints)
    status_msg.capture_target = self.capture_target
    status_msg.checkerboard_cols = self.cb_cols
    status_msg.checkerboard_rows = self.cb_rows
    status_msg.checkerboard_square_m = self.cb_square_m
    status_msg.min_disparity = self.min_disparity
    status_msg.num_disparities = self.num_disparities
    status_msg.block_size = self.block_size
    status_msg.uniqueness_ratio = self.uniqueness_ratio
    status_msg.speckle_window_size = self.speckle_window_size
    status_msg.speckle_range = self.speckle_range
    status_msg.min_range_m = self.min_range_m
    status_msg.max_range_m = self.max_range_m
    status_msg.running = self.running
    status_msg.status_message = self.status_message
    if self.node_if is not None:
      self.node_if.publish_pub('status_pub', status_msg)

  #######################
  # Utility Functions

  def cleanup_actions(self):
    self.msg_if.pub_info("STEREO_DEPTH_APP: Shutting down: Executing script cleanup actions")
    self._subscribeLeft("None")
    self._subscribeRight("None")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiStereoDepthApp()
