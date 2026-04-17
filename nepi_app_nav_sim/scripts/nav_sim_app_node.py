#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
# Nav Sim App: runs NMEA and/or HNav TCP simulator servers that the
# corresponding NPX driver nodes (npx_nmea_udp_node, npx_hnav_tcu_node)
# can connect to.  All position/orientation parameters are adjustable via
# ROS topics and the NEPI RUI.

import copy
import datetime
import math
import socket
import struct
import threading
import time

from std_msgs.msg import Bool, Empty, Float32, Int32

from nepi_app_nav_sim.msg import NepiAppNavSimStatus

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_nav

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.data_if import NavPoseIF


#########################################
# Factory Defaults

FACTORY_LATITUDE     = 47.6205
FACTORY_LONGITUDE    = -122.3493
FACTORY_ALTITUDE_M   = 10.0
FACTORY_DEPTH_M      = 0.0
FACTORY_HEADING_DEG  = 0.0
FACTORY_ROLL_DEG     = 0.0
FACTORY_PITCH_DEG    = 0.0
FACTORY_SPEED_MS     = 0.0
FACTORY_NMEA_ENABLED = False
FACTORY_HNAV_ENABLED = False
FACTORY_NMEA_PORT    = 50000
FACTORY_HNAV_PORT    = 16718

STATUS_RATE_HZ = 1.0

#########################################
# HNav packet helpers (from npx_hnav_tcu_discovery.py)
#########################################

_PACKET_SIZE     = 67
_START_BYTE1     = 0xAA
_START_BYTE2     = 0xBF
_PROTO_VER       = 0x00
_DATA_SIZE_LO    = 0x37
_DATA_SIZE_HI    = 0x00
_LAT_SCALE       = 90.0  / 2147483648.0
_LON_SCALE       = 180.0 / 2147483648.0
_DEPTH_SCALE     = 0.001
_ALT_SCALE       = 0.01
_ORIENT_SCALE    = 0.01
_VEL_SCALE       = 0.001
_SOUND_VEL_SCALE = 0.1
_TEMP_SCALE      = 0.01
_DEFAULT_STATUS  = 0x0002   # hybrid mode; all validity bits clear (= valid)
_DATA_FMT        = '<BQiiiHhhHhhhhhhHhfHHH'


def _reflect(value, bit_num):
    out, bit = 0, 1
    for i in range(bit_num - 1, -1, -1):
        if value & (1 << i):
            out |= bit
        bit <<= 1
    return out


def _crc16(data_bytes):
    crc = 0xFFFF
    for byte in data_bytes:
        current = _reflect(byte, 8)
        j = 0x80
        while j:
            bit = crc & 0x8000
            crc = (crc << 1) & 0xFFFF
            if current & j:
                bit ^= 0x8000
            if bit:
                crc ^= 0x1021
            j >>= 1
    crc = _reflect(crc, 16)
    crc ^= 0xFFFF
    return crc & 0xFFFF


def _build_hnav_packet(lat_deg, lon_deg, depth_m, alt_m,
                       roll_deg, pitch_deg, heading_deg,
                       vel_fwd_ms=0.0, sound_vel_ms=1500.0,
                       temp_c=12.0, pos_qual_m=1.0,
                       status=_DEFAULT_STATUS):
    utc_us = int(time.time() * 1e6)

    def c16s(v): return max(-32768, min(32767, v))
    def c16u(v): return max(0,      min(65535, v))

    data = struct.pack(
        _DATA_FMT,
        0,
        utc_us & 0xFFFFFFFFFFFFFFFF,
        int(lat_deg  / _LAT_SCALE),
        int(lon_deg  / _LON_SCALE),
        int(depth_m  / _DEPTH_SCALE),
        c16u(max(0, int(alt_m    / _ALT_SCALE))),
        c16s(int(roll_deg  / _ORIENT_SCALE)),
        c16s(int(pitch_deg / _ORIENT_SCALE)),
        c16u(int((heading_deg % 360.0) / _ORIENT_SCALE)),
        c16s(int(vel_fwd_ms / _VEL_SCALE)),
        0, 0, 0, 0, 0,
        c16u(int(sound_vel_ms / _SOUND_VEL_SCALE)),
        c16s(int(temp_c      / _TEMP_SCALE)),
        float(pos_qual_m),
        10, 10,
        c16u(status),
    )
    header = bytes([_START_BYTE1, _START_BYTE2, _PROTO_VER,
                    0x00, 0x00, _DATA_SIZE_LO, _DATA_SIZE_HI,
                    0x00, 0x00, 0x00])
    body = header + data
    return body + struct.pack('<H', _crc16(body))


#########################################
# NMEA sentence helpers (from npx_nmea_udp_discovery.py)
#########################################

def _dd_to_nmea(lat, lon):
    alat, alon = abs(lat), abs(lon)
    lat_deg, lon_deg = int(alat), int(alon)
    lat_min = (alat - lat_deg) * 60.0
    lon_min = (alon - lon_deg) * 60.0
    ns = 'N' if lat >= 0 else 'S'
    ew = 'E' if lon >= 0 else 'W'
    return f"{lat_deg:02d}{lat_min:07.4f}", ns, f"{lon_deg:03d}{lon_min:07.4f}", ew


def _nmea_checksum(s):
    c = 0
    for ch in s:
        c ^= ord(ch)
    return f"{c:02X}"


def _make_GGA(lat, lon, alt_m):
    now = datetime.datetime.utcnow().strftime("%H%M%S")
    ls, ns, lons, ew = _dd_to_nmea(lat, lon)
    parts = ["GPGGA", now, ls, ns, lons, ew, "1", "08", "1.0",
             f"{alt_m:.1f}", "M", "0.0", "M", "", ""]
    core = ",".join(parts)
    return f"${core}*{_nmea_checksum(core)}"


def _make_RMC(lat, lon, sog_kts, cog_deg):
    now = datetime.datetime.utcnow()
    ts, ds = now.strftime("%H%M%S"), now.strftime("%d%m%y")
    ls, ns, lons, ew = _dd_to_nmea(lat, lon)
    parts = ["GPRMC", ts, "A", ls, ns, lons, ew,
             f"{sog_kts:.1f}", f"{cog_deg:.1f}", ds, "", ""]
    core = ",".join(parts)
    return f"${core}*{_nmea_checksum(core)}"


def _make_VTG(cog_deg, sog_kts):
    parts = ["GPVTG", f"{cog_deg:.1f}", "T", "", "M",
             f"{sog_kts:.1f}", "N", f"{sog_kts*1.852:.1f}", "K"]
    core = ",".join(parts)
    return f"${core}*{_nmea_checksum(core)}"


def _make_HDG(heading_deg):
    parts = ["HCHDG", f"{heading_deg:.1f}", "", "", "", ""]
    core = ",".join(parts)
    return f"${core}*{_nmea_checksum(core)}"


#########################################
# Node Class
#########################################

class NepiNavSimApp(object):

    DEFAULT_NODE_NAME = "app_nav_sim"

    nmea_sim_enabled = FACTORY_NMEA_ENABLED
    hnav_sim_enabled = FACTORY_HNAV_ENABLED
    latitude         = FACTORY_LATITUDE
    longitude        = FACTORY_LONGITUDE
    altitude_m       = FACTORY_ALTITUDE_M
    depth_m          = FACTORY_DEPTH_M
    heading_deg      = FACTORY_HEADING_DEG
    roll_deg         = FACTORY_ROLL_DEG
    pitch_deg        = FACTORY_PITCH_DEG
    speed_ms         = FACTORY_SPEED_MS
    nmea_port        = FACTORY_NMEA_PORT
    hnav_port        = FACTORY_HNAV_PORT

    nmea_connected = False
    hnav_connected = False

    node_if = None

    _lock             = threading.Lock()
    _nmea_stop        = None
    _hnav_stop        = None
    _nmea_navpose_if  = None
    _hnav_navpose_if  = None

    def __init__(self):
        nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
        self.class_name     = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name      = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        self.CFGS_DICT = {
            'init_callback':          self.initCb,
            'reset_callback':         self.resetCb,
            'factory_reset_callback': self.factoryResetCb,
            'init_configs':           True,
            'namespace':              self.node_namespace,
        }

        self.PARAMS_DICT = {
            'nmea_sim_enabled': {'namespace': self.node_namespace, 'factory_val': self.nmea_sim_enabled},
            'hnav_sim_enabled': {'namespace': self.node_namespace, 'factory_val': self.hnav_sim_enabled},
            'latitude':         {'namespace': self.node_namespace, 'factory_val': self.latitude},
            'longitude':        {'namespace': self.node_namespace, 'factory_val': self.longitude},
            'altitude_m':       {'namespace': self.node_namespace, 'factory_val': self.altitude_m},
            'depth_m':          {'namespace': self.node_namespace, 'factory_val': self.depth_m},
            'heading_deg':      {'namespace': self.node_namespace, 'factory_val': self.heading_deg},
            'roll_deg':         {'namespace': self.node_namespace, 'factory_val': self.roll_deg},
            'pitch_deg':        {'namespace': self.node_namespace, 'factory_val': self.pitch_deg},
            'speed_ms':         {'namespace': self.node_namespace, 'factory_val': self.speed_ms},
            'nmea_port':        {'namespace': self.node_namespace, 'factory_val': self.nmea_port},
            'hnav_port':        {'namespace': self.node_namespace, 'factory_val': self.hnav_port},
        }

        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.node_namespace,
                'topic':     'status',
                'msg':       NepiAppNavSimStatus,
                'qsize':     1,
                'latch':     True,
            }
        }

        self.SUBS_DICT = {
            'set_nmea_enabled': {'namespace': self.node_namespace, 'topic': 'set_nmea_enabled', 'msg': Bool,    'qsize': 10, 'callback': self.setNmeaEnabledCb,  'callback_args': ()},
            'set_hnav_enabled': {'namespace': self.node_namespace, 'topic': 'set_hnav_enabled', 'msg': Bool,    'qsize': 10, 'callback': self.setHnavEnabledCb,  'callback_args': ()},
            'set_latitude':     {'namespace': self.node_namespace, 'topic': 'set_latitude',     'msg': Float32, 'qsize': 10, 'callback': self.setLatitudeCb,      'callback_args': ()},
            'set_longitude':    {'namespace': self.node_namespace, 'topic': 'set_longitude',    'msg': Float32, 'qsize': 10, 'callback': self.setLongitudeCb,     'callback_args': ()},
            'set_altitude':     {'namespace': self.node_namespace, 'topic': 'set_altitude',     'msg': Float32, 'qsize': 10, 'callback': self.setAltitudeCb,      'callback_args': ()},
            'set_depth':        {'namespace': self.node_namespace, 'topic': 'set_depth',        'msg': Float32, 'qsize': 10, 'callback': self.setDepthCb,         'callback_args': ()},
            'set_heading':      {'namespace': self.node_namespace, 'topic': 'set_heading',      'msg': Float32, 'qsize': 10, 'callback': self.setHeadingCb,       'callback_args': ()},
            'set_roll':         {'namespace': self.node_namespace, 'topic': 'set_roll',         'msg': Float32, 'qsize': 10, 'callback': self.setRollCb,          'callback_args': ()},
            'set_pitch':        {'namespace': self.node_namespace, 'topic': 'set_pitch',        'msg': Float32, 'qsize': 10, 'callback': self.setPitchCb,         'callback_args': ()},
            'set_speed':        {'namespace': self.node_namespace, 'topic': 'set_speed',        'msg': Float32, 'qsize': 10, 'callback': self.setSpeedCb,         'callback_args': ()},
        }

        self.node_if = NodeClassIF(
            configs_dict=self.CFGS_DICT,
            params_dict=self.PARAMS_DICT,
            pubs_dict=self.PUBS_DICT,
            subs_dict=self.SUBS_DICT,
        )
        self.node_if.wait_for_ready()

        self.initCb(do_updates=True)

        time.sleep(1)
        nepi_sdk.start_timer_process(1.0 / STATUS_RATE_HZ, self.statusPublishCb)

        time.sleep(1)
        self.msg_if.pub_info("Initialization Complete")

        nepi_sdk.on_shutdown(self.cleanup_actions)
        nepi_sdk.spin()


    ###################
    ## Control Callbacks

    def setNmeaEnabledCb(self, msg):
        with self._lock:
            self.nmea_sim_enabled = msg.data
        self._apply_nmea_state()
        self._save_param('nmea_sim_enabled', self.nmea_sim_enabled)

    def setHnavEnabledCb(self, msg):
        with self._lock:
            self.hnav_sim_enabled = msg.data
        self._apply_hnav_state()
        self._save_param('hnav_sim_enabled', self.hnav_sim_enabled)

    def setLatitudeCb(self, msg):
        with self._lock:
            self.latitude = float(msg.data)
        self._save_param('latitude', self.latitude)
        self.publish_status()

    def setLongitudeCb(self, msg):
        with self._lock:
            self.longitude = float(msg.data)
        self._save_param('longitude', self.longitude)
        self.publish_status()

    def setAltitudeCb(self, msg):
        with self._lock:
            self.altitude_m = float(msg.data)
        self._save_param('altitude_m', self.altitude_m)
        self.publish_status()

    def setDepthCb(self, msg):
        with self._lock:
            self.depth_m = float(msg.data)
        self._save_param('depth_m', self.depth_m)
        self.publish_status()

    def setHeadingCb(self, msg):
        with self._lock:
            self.heading_deg = float(msg.data) % 360.0
        self._save_param('heading_deg', self.heading_deg)
        self.publish_status()

    def setRollCb(self, msg):
        with self._lock:
            self.roll_deg = float(msg.data)
        self._save_param('roll_deg', self.roll_deg)
        self.publish_status()

    def setPitchCb(self, msg):
        with self._lock:
            self.pitch_deg = float(msg.data)
        self._save_param('pitch_deg', self.pitch_deg)
        self.publish_status()

    def setSpeedCb(self, msg):
        with self._lock:
            self.speed_ms = max(0.0, float(msg.data))
        self._save_param('speed_ms', self.speed_ms)
        self.publish_status()


    #######################
    ### Config Functions

    def initCb(self, do_updates=False):
        if self.node_if is not None:
            with self._lock:
                self.nmea_sim_enabled = self.node_if.get_param('nmea_sim_enabled')
                self.hnav_sim_enabled = self.node_if.get_param('hnav_sim_enabled')
                self.latitude         = float(self.node_if.get_param('latitude'))
                self.longitude        = float(self.node_if.get_param('longitude'))
                self.altitude_m       = float(self.node_if.get_param('altitude_m'))
                self.depth_m          = float(self.node_if.get_param('depth_m'))
                self.heading_deg      = float(self.node_if.get_param('heading_deg'))
                self.roll_deg         = float(self.node_if.get_param('roll_deg'))
                self.pitch_deg        = float(self.node_if.get_param('pitch_deg'))
                self.speed_ms         = float(self.node_if.get_param('speed_ms'))
                self.nmea_port        = int(self.node_if.get_param('nmea_port'))
                self.hnav_port        = int(self.node_if.get_param('hnav_port'))
        if do_updates:
            self._apply_nmea_state()
            self._apply_hnav_state()
        self.publish_status()

    def resetCb(self, do_updates=True):
        self.msg_if.pub_warn("Resetting")
        self.initCb(do_updates=do_updates)

    def factoryResetCb(self, do_updates=True):
        self.msg_if.pub_warn("Factory Resetting")
        self.initCb(do_updates=do_updates)


    ###################
    ## Status Publisher

    def statusPublishCb(self, timer):
        self.publish_status()
        self._publishNavpose()

    def publish_status(self):
        msg = NepiAppNavSimStatus()
        with self._lock:
            msg.nmea_sim_enabled = self.nmea_sim_enabled
            msg.hnav_sim_enabled = self.hnav_sim_enabled
            msg.nmea_connected   = self.nmea_connected
            msg.hnav_connected   = self.hnav_connected
            msg.latitude         = self.latitude
            msg.longitude        = self.longitude
            msg.altitude_m       = self.altitude_m
            msg.depth_m          = self.depth_m
            msg.heading_deg      = self.heading_deg
            msg.roll_deg         = self.roll_deg
            msg.pitch_deg        = self.pitch_deg
            msg.speed_ms         = self.speed_ms
            msg.nmea_port        = self.nmea_port
            msg.hnav_port        = self.hnav_port
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', msg)


    #######################
    ## TCP Server Management

    def _apply_nmea_state(self):
        with self._lock:
            enabled = self.nmea_sim_enabled
            port    = self.nmea_port
        if enabled and self._nmea_stop is None:
            self._nmea_navpose_if = NavPoseIF(
                namespace=self.node_namespace + '/nmea',
                data_source_description='nav_sim_nmea',
                data_ref_description='WGS84',
                pub_navpose=True,
                pub_location=True,
                pub_heading=True,
                pub_altitude=True,
                msg_if=self.msg_if,
            )
            self._start_nmea_server(port)
        elif not enabled and self._nmea_stop is not None:
            self._stop_server('nmea')
            self._nmea_navpose_if = None
        self.publish_status()

    def _apply_hnav_state(self):
        with self._lock:
            enabled = self.hnav_sim_enabled
            port    = self.hnav_port
        if enabled and self._hnav_stop is None:
            self._hnav_navpose_if = NavPoseIF(
                namespace=self.node_namespace + '/hnav',
                data_source_description='nav_sim_hnav',
                data_ref_description='WGS84',
                pub_navpose=True,
                pub_location=True,
                pub_heading=True,
                pub_orientation=True,
                pub_altitude=True,
                pub_depth=True,
                msg_if=self.msg_if,
            )
            self._start_hnav_server(port)
        elif not enabled and self._hnav_stop is not None:
            self._stop_server('hnav')
            self._hnav_navpose_if = None
        self.publish_status()

    def _start_nmea_server(self, port):
        stop_evt = threading.Event()
        self._nmea_stop = stop_evt
        t = threading.Thread(
            target=self._nmea_server_loop,
            args=('127.0.0.1', port, stop_evt),
            daemon=True,
        )
        t.start()
        self.msg_if.pub_info(f"NMEA simulator started on port {port}")

    def _start_hnav_server(self, port):
        stop_evt = threading.Event()
        self._hnav_stop = stop_evt
        t = threading.Thread(
            target=self._hnav_server_loop,
            args=('127.0.0.1', port, stop_evt),
            daemon=True,
        )
        t.start()
        self.msg_if.pub_info(f"HNav simulator started on port {port}")

    def _stop_server(self, which):
        if which == 'nmea' and self._nmea_stop is not None:
            self._nmea_stop.set()
            self._nmea_stop = None
            with self._lock:
                self.nmea_connected = False
            self.msg_if.pub_info("NMEA simulator stopped")
        elif which == 'hnav' and self._hnav_stop is not None:
            self._hnav_stop.set()
            self._hnav_stop = None
            with self._lock:
                self.hnav_connected = False
            self.msg_if.pub_info("HNav simulator stopped")


    #######################
    ## NMEA TCP Server Loop

    def _nmea_server_loop(self, host, port, stop_evt):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind((host, port))
            srv.listen(1)
            srv.settimeout(0.5)
            while not stop_evt.is_set():
                try:
                    conn, _ = srv.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                with self._lock:
                    self.nmea_connected = True
                self._serve_nmea_client(conn, stop_evt)
                with self._lock:
                    self.nmea_connected = False
        except Exception as e:
            self.msg_if.pub_warn(f"NMEA server error: {e}")
        finally:
            try:
                srv.close()
            except Exception:
                pass

    def _serve_nmea_client(self, conn, stop_evt):
        try:
            while not stop_evt.is_set():
                with self._lock:
                    lat     = self.latitude
                    lon     = self.longitude
                    alt     = self.altitude_m
                    heading = self.heading_deg
                    speed   = self.speed_ms
                sog_kts = speed * 1.944   # m/s → knots
                lines = [
                    _make_GGA(lat, lon, alt),
                    _make_RMC(lat, lon, sog_kts, heading),
                    _make_VTG(heading, sog_kts),
                    _make_HDG(heading),
                ]
                payload = ("\r\n".join(lines) + "\r\n").encode("ascii")
                conn.sendall(payload)
                # Dead-reckoning update
                if speed > 0:
                    dt = 1.0 / 5   # 5 Hz NMEA rate
                    self._deadReckon(dt)
                time.sleep(1.0 / 5)
        except Exception:
            pass
        finally:
            try:
                conn.close()
            except Exception:
                pass


    #######################
    ## HNav TCP Server Loop

    def _hnav_server_loop(self, host, port, stop_evt):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind((host, port))
            srv.listen(1)
            srv.settimeout(0.5)
            while not stop_evt.is_set():
                try:
                    conn, _ = srv.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                with self._lock:
                    self.hnav_connected = True
                self._serve_hnav_client(conn, stop_evt)
                with self._lock:
                    self.hnav_connected = False
        except Exception as e:
            self.msg_if.pub_warn(f"HNav server error: {e}")
        finally:
            try:
                srv.close()
            except Exception:
                pass

    def _serve_hnav_client(self, conn, stop_evt):
        rate_hz = 10
        period  = 1.0 / rate_hz
        try:
            while not stop_evt.is_set():
                with self._lock:
                    lat     = self.latitude
                    lon     = self.longitude
                    alt     = self.altitude_m
                    depth   = self.depth_m
                    heading = self.heading_deg
                    roll    = self.roll_deg
                    pitch   = self.pitch_deg
                    speed   = self.speed_ms
                packet = _build_hnav_packet(
                    lat_deg=lat, lon_deg=lon,
                    depth_m=depth, alt_m=alt,
                    roll_deg=roll, pitch_deg=pitch,
                    heading_deg=heading,
                    vel_fwd_ms=speed,
                )
                conn.sendall(packet)
                if speed > 0:
                    self._deadReckon(period)
                time.sleep(period)
        except Exception:
            pass
        finally:
            try:
                conn.close()
            except Exception:
                pass


    #######################
    ## Dead-Reckoning

    def _deadReckon(self, dt_sec):
        """Advance lat/lon in the current heading direction by speed * dt."""
        with self._lock:
            heading_rad = math.radians(self.heading_deg)
            speed       = self.speed_ms
            lat         = self.latitude
            lon         = self.longitude
        dist_m = speed * dt_sec
        dlat   = (dist_m / 111320.0) * math.cos(heading_rad)
        dlon   = (dist_m / (111320.0 * max(math.cos(math.radians(lat)), 1e-6))) \
                 * math.sin(heading_rad)
        with self._lock:
            self.latitude  += dlat
            self.longitude += dlon


    #######################
    ## NavPose Publishing

    def _publishNavpose(self):
        if self._nmea_navpose_if is not None:
            self._nmea_navpose_if.publish_navpose(self._buildNmeaNavposeDict())
        if self._hnav_navpose_if is not None:
            self._hnav_navpose_if.publish_navpose(self._buildHnavNavposeDict())

    def _buildNmeaNavposeDict(self):
        with self._lock:
            lat     = self.latitude
            lon     = self.longitude
            alt     = self.altitude_m
            heading = self.heading_deg
        t = nepi_sdk.get_time()
        d = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        d['has_location']  = True;  d['latitude']    = lat;     d['longitude']   = lon;     d['time_location']  = t
        d['has_heading']   = True;  d['heading_deg'] = heading;                             d['time_heading']   = t
        d['has_altitude']  = True;  d['altitude_m']  = alt;                                 d['time_altitude']  = t
        return d

    def _buildHnavNavposeDict(self):
        with self._lock:
            lat     = self.latitude
            lon     = self.longitude
            alt     = self.altitude_m
            depth   = self.depth_m
            heading = self.heading_deg
            roll    = self.roll_deg
            pitch   = self.pitch_deg
        t = nepi_sdk.get_time()
        d = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        d['has_location']    = True;  d['latitude']    = lat;     d['longitude']   = lon;     d['time_location']    = t
        d['has_heading']     = True;  d['heading_deg'] = heading;                             d['time_heading']     = t
        d['has_orientation'] = True;  d['roll_deg']    = roll;    d['pitch_deg']   = pitch;   d['yaw_deg'] = heading; d['time_orientation'] = t
        d['has_altitude']    = True;  d['altitude_m']  = alt;                                 d['time_altitude']    = t
        d['has_depth']       = True;  d['depth_m']     = depth;                               d['time_depth']       = t
        return d


    #######################
    ## Utility

    def _save_param(self, key, val):
        if self.node_if is not None:
            self.node_if.set_param(key, val)
            self.node_if.save_config()

    def cleanup_actions(self):
        self._stop_server('nmea')
        self._stop_server('hnav')
        self.msg_if.pub_info("NAV_SIM_APP: Shutting down")


#########################################
# Main
#########################################
if __name__ == '__main__':
    NepiNavSimApp()
