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
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

import copy
import datetime
import math
import os
import random
import socket
import struct
import threading
import time
import yaml

import numpy as np

from std_msgs.msg import Bool, Empty, Float32, Header, String
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

from mavros_msgs.msg import GPSINPUT

from nepi_interfaces.msg import UpdateString

from nepi_app_nav_sim.msg import (
    NepiAppNmeaSimStatus,
    NepiAppHNavSimStatus,
    NepiAppGpsSimStatus,
    NepiAppNavSimMasterStatus,
)

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_controls

from nepi_api.messages_if import MsgIF
from nepi_api.data_if import NavPoseIF
from nepi_api.system_if import ControlsIF


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

_NMEA_MOVE_FIELDS = (
    'nmea_latitude', 'nmea_longitude', 'nmea_altitude_m',
    'nmea_heading_deg', 'nmea_speed_ms',
)

_HNAV_MOVE_FIELDS = (
    'hnav_latitude', 'hnav_longitude', 'hnav_altitude_m', 'hnav_depth_m',
    'hnav_heading_deg', 'hnav_roll_deg', 'hnav_pitch_deg', 'hnav_speed_ms',
)

_SIN_FIELDS = ('hnav_heading_deg', 'hnav_roll_deg', 'hnav_pitch_deg')


#########################################
# GPS sim factory defaults and MAVLink constants
#
# Ported from nepi_app_fake_gps, which this app absorbed. The GPS kind does not
# serve a TCP stream like the other two; it publishes a simulated NavSatFix and
# Odometry and injects a MAVLink GPS_INPUT into a selected mavros node.
#
###################################################
### For Ardupilot Mavlink Support
### These Ardupilot Parameters Must Be Configured First to allow MAVLINK GPS Override:
# GPS_TYPE = 14
# GPS_DELAY_MS = 1
# EK3_POS_I_GATE = 300
# EK3_POSNE_M_NSE = 5
# EK3_SRC_OPTIONS = 0
# EK3_SRC1_POSXY = 3
# EK3_SRC1_POSZ = 3
# EK3_SRC1_VELXY = 3
# EK3_SRC1_VELZ = 3
# EK3_SRC1_YAW = 1
# BARO_OPTION = 1  (required for proper barometer reading on Pixhawk)
###################################################

FACTORY_GPS_ENABLED       = False
FACTORY_GPS_LATITUDE      = 46.6540828
FACTORY_GPS_LONGITUDE     = -122.3187578
FACTORY_GPS_ALTITUDE_M    = 0.0
FACTORY_SELECTED_MAVROS   = 'None'
FACTORY_SAT_COUNT         = 20

# Fake GPS publish rate (Hz) and move interpolation tuning
GPS_PUB_RATE_HZ            = 50      # factory default publish rate
MIN_GPS_PUB_RATE_HZ        = 1.0
MAX_GPS_PUB_RATE_HZ        = 100.0
MOVE_UPDATE_TIME_SEC_PER_M = 1.0
MAX_MOVE_TIME_S            = 20.0

# A goto axis value of this sentinel means "hold the current value"
HOLD_SENTINEL = -999.0

# Control bounds. The rate bounds are the ones declared above. The rest are not
# declared anywhere else, so they are the widest range still meaningful for a
# simulated vehicle: lat/lon are the WGS84 domain, the altitude range spans
# subsea to high-altitude fixed-wing, the satellite count feeds a uint8
# GPS_INPUT field and 30 is above any real constellation, and the ENU offset is
# an operator sanity range (the node bounds move DURATION with MAX_MOVE_TIME_S,
# never move distance).
MIN_LATITUDE_DEG   = -90.0
MAX_LATITUDE_DEG   = 90.0
MIN_LONGITUDE_DEG  = -180.0
MAX_LONGITUDE_DEG  = 180.0
MIN_ALTITUDE_M     = -500.0
MAX_ALTITUDE_M     = 20000.0
MIN_SAT_COUNT      = 0
MAX_SAT_COUNT      = 30
MAX_ENU_OFFSET_M   = 10000.0

# nepi_controls clamps 'round' to 6 decimal places, so a latitude control holds
# about 0.11 m of resolution. That is well inside the fidelity of the rest of
# this simulation, but it is why a factory latitude's seventh decimal does not
# survive a round trip through the controls dict.
GEO_ROUND_PLACES = 6

# mavros liveness topic used for target-node discovery
MAVROS_STATE_MSG       = 'State'
MAVROS_STATE_SUFFIX    = '/state'
# GPS_INPUT carries a yaw field (HilGPS does not), letting the simulated GPS
# supply the heading the EKF would otherwise get from a compass. Injected via
# the mavros gps_input plugin; consumed by the ArduPilot MAV GPS backend
# (GPS_TYPE=14).
MAVROS_GPS_INPUT_TOPIC = 'gps_input/gps_input'

# Reject finite-difference velocity spikes from a position reset/teleport (m/s).
MAX_FAKE_GPS_SPEED_MPS = 100.0
# GPS week/epoch for deriving GPS_INPUT time-of-week from system UTC. A real GPS
# always reports valid, advancing GPS time; without it AP_GPS marks the receiver
# unhealthy ("GPS: Fail") and the EKF will not declare a healthy position.
GPS_EPOCH_UNIX_S = 315964800   # 1980-01-06 00:00:00 UTC, in Unix seconds
GPS_LEAP_SECONDS = 18          # current GPS-UTC offset (leap seconds)
SECONDS_PER_WEEK = 604800

# Discovery of mavros nodes is PROCESS-GLOBAL: find_topics_by_msg walks the
# whole master topic list, and every GPS instance would get the same answer. The
# master class runs one timer at this rate and pushes the result down.
MAVROS_DISCOVER_RATE_HZ = 1.0


#########################################
# Control sets
#
# Each simulated field renders as ONE row -- the value box, an Auto toggle, and
# (only while Auto is on) Step and Rate Hz boxes. Control.display_group is what
# puts them on one line: every control of a row carries the field name as its
# group, and Nepi_IF_Controls lays a group out horizontally in declaration
# order. The FIRST control of a group supplies the row label on the left, which
# is why the value control carries the full label and the rest carry the short
# inline captions.
#
# Control names ARE the instance attribute names. _controlsUpdatedCb setattrs
# straight back onto the instance, so there is no separate name table to drift
# out of sync with the status message.

_ROW_VALUE_WIDTH = 100
_ROW_SMALL_WIDTH = 70

# Section suffixes of the per-instance control sets.
#
# A ControlsIF is ALWAYS a direct child of the NODE namespace: its __init__
# builds create_namespace(node_namespace, controls_name) and there is no
# namespace argument, while get_clean_name() rewrites '/' to '_' so the name
# cannot carry a path. A set therefore CANNOT be mounted under an instance
# namespace, and the name is the only thing that can distinguish one set from
# another.
#
# So the full controls name is built per instance as
#     <instance kind>_instances_<instance name>_<section suffix>
# e.g. nmea_instances_nmea_0_position -- mirroring the instance's own ROS path
# so it stays unique across both instance kinds and every instance of each, and
# so the RUI can derive exactly the same string from the names it already shows.
# Getting this wrong is silent: six sets collapse onto three namespaces, the
# last one constructed wins, and the page renders its headings with nothing
# underneath.
#
# One ControlsIF per SECTION rather than one per instance, because a control
# set renders as a single flat list and the page groups its rows under
# Position / Orientation / Dead-Reckoning headings.
CONTROLS_SUFFIX_POSITION      = 'position'
CONTROLS_SUFFIX_ORIENTATION   = 'orientation'
CONTROLS_SUFFIX_DEADRECKONING = 'dead_reckoning'

# The GPS kind has no Auto/Step rows, so it reuses only the position suffix and
# adds two of its own: 'move' for the goto buffers and their command Buttons,
# 'output' for the injection settings (target mavros node, publish rate,
# reported satellite count).
#
# Checked against every sub-interface leaf this node mounts before being
# settled on, per the namespace rule: the only sub-IF in this node is NavPoseIF,
# and each instance mounts it at <instance_ns>/navpose -- under the INSTANCE
# namespace, not the node namespace -- so it cannot collide with a control set,
# which is always a direct child of the node namespace. The full derived set
# names are gps_instances_<name>_{position,move,output}, which also cannot
# collide with the <node>/gps_instances/<name>/... instance subtree, with each
# other, or with an NMEA or HNav set.
CONTROLS_SUFFIX_MOVE   = 'move'
CONTROLS_SUFFIX_OUTPUT = 'output'


def instanceControlsName(instance_ns, base_ns, suffix):
    """Build the flat controls name for one instance's section control set."""
    tail = instance_ns.replace(base_ns, '')
    tail = tail.strip('/').replace('/', '_')
    return tail + '_' + suffix

# (section title, controls-name suffix, ((field, label, factory value), ...))
_NMEA_SECTIONS = (
    ('Position', CONTROLS_SUFFIX_POSITION, (
        ('nmea_latitude',    'Latitude (°)',  FACTORY_LATITUDE),
        ('nmea_longitude',   'Longitude (°)', FACTORY_LONGITUDE),
        ('nmea_altitude_m',  'Altitude (m)',  FACTORY_ALTITUDE_M),
    )),
    ('Orientation', CONTROLS_SUFFIX_ORIENTATION, (
        ('nmea_heading_deg', 'Heading (°)',   FACTORY_HEADING_DEG),
    )),
    ('Dead-Reckoning', CONTROLS_SUFFIX_DEADRECKONING, (
        ('nmea_speed_ms',    'Speed (m/s)',   FACTORY_SPEED_MS),
    )),
)

_HNAV_SECTIONS = (
    ('Position', CONTROLS_SUFFIX_POSITION, (
        ('hnav_latitude',    'Latitude (°)',  FACTORY_LATITUDE),
        ('hnav_longitude',   'Longitude (°)', FACTORY_LONGITUDE),
        ('hnav_altitude_m',  'Altitude (m)',  FACTORY_ALTITUDE_M),
        ('hnav_depth_m',     'Depth (m)',     FACTORY_DEPTH_M),
    )),
    ('Orientation', CONTROLS_SUFFIX_ORIENTATION, (
        ('hnav_heading_deg', 'Heading (°)',   FACTORY_HEADING_DEG),
        ('hnav_roll_deg',    'Roll (°)',      FACTORY_ROLL_DEG),
        ('hnav_pitch_deg',   'Pitch (°)',     FACTORY_PITCH_DEG),
    )),
    ('Dead-Reckoning', CONTROLS_SUFFIX_DEADRECKONING, (
        ('hnav_speed_ms',    'Speed (m/s)',   FACTORY_SPEED_MS),
    )),
)


def build_row_controls(field, label, value_default, with_sin = False):
    """Build the control entries for one simulated field's row.

    Args:
        field: instance attribute name, also the control name and the row's
            display_group.
        label: row label shown at the left of the line.
        value_default: factory value for the value control.
        with_sin: True for the three HNav orientation fields that also carry
            sinusoidal and wave motion.

    Returns:
        dict: control-name -> init dict, in row order.
    """
    controls = {}
    controls[field] = {
        'type': 'Float', 'default': value_default, 'description': label,
        'display_name': label, 'display_group': field,
        'display_width': _ROW_VALUE_WIDTH,
    }
    controls['enable_move_' + field] = {
        'type': 'Toggle', 'default': False,
        'description': 'Step ' + label + ' automatically',
        'display_name': 'Auto', 'display_group': field,
    }
    # Step and Rate stay hidden until Auto is on. syncRowVisibility below is
    # what flips them, and it runs on every controls update and after every
    # config restore so the row can never be left showing a stale shape.
    controls['move_step_' + field] = {
        'type': 'Float', 'default': 0.0,
        'description': label + ' change per step',
        'display_name': 'Step', 'display_group': field,
        'display_width': _ROW_SMALL_WIDTH, 'display_hidden': True,
    }
    controls['move_rate_hz_' + field] = {
        'type': 'Float', 'default': 1.0,
        'description': label + ' steps per second',
        'display_name': 'Rate Hz', 'display_group': field,
        'display_width': _ROW_SMALL_WIDTH, 'display_hidden': True,
    }
    if with_sin == True:
        controls['enable_sin_' + field] = {
            'type': 'Toggle', 'default': False,
            'description': 'Oscillate ' + label + ' sinusoidally',
            'display_name': 'Sin', 'display_group': field,
            'display_hidden': True,
        }
        controls['sin_amplitude_' + field] = {
            'type': 'Float', 'default': 0.0,
            'description': label + ' oscillation amplitude',
            'display_name': 'Amp', 'display_group': field,
            'display_width': _ROW_SMALL_WIDTH, 'display_hidden': True,
        }
        controls['sin_period_s_' + field] = {
            'type': 'Float', 'default': 1.0,
            'description': label + ' oscillation period in seconds',
            'display_name': 'Period', 'display_group': field,
            'display_width': _ROW_SMALL_WIDTH, 'display_hidden': True,
        }
        controls['enable_wave_' + field] = {
            'type': 'Toggle', 'default': False,
            'description': 'Sum several sine components for ' + label,
            'display_name': 'Wave', 'display_group': field,
            'display_hidden': True,
        }
        controls['sin_spread_' + field] = {
            'type': 'Float', 'default': 0.0,
            'description': label + ' wave component spread',
            'display_name': 'Spread', 'display_group': field,
            'display_width': _ROW_SMALL_WIDTH, 'display_hidden': True,
        }
    return controls


def build_section_controls(section_fields, sin_fields = ()):
    """Build one section's init dict from its field specs, in row order."""
    controls = {}
    for field, label, value_default in section_fields:
        controls.update(build_row_controls(field, label, value_default,
                                           with_sin = (field in sin_fields)))
    return controls


#########################################
# GPS sim control sets
#
# Unlike the NMEA and HNav kinds, whose controls are one generated row per
# simulated field, the GPS kind's controls came over from nepi_app_fake_gps as
# a hand-written set. They are declared here as one flat dict and sliced into
# sections by _GPS_SECTIONS, so key order is still display order within each
# section.
#
# Control names ARE the instance attribute names, matching the convention the
# other two kinds use.

_GPS_ENU_AXIS_LABELS = ['East (m)', 'North (m)', 'Up (m)']
_GPS_GEO_AXIS_LABELS = ['Latitude', 'Longitude', 'Altitude (m)']

# Button controls, keyed here so the updated callback can tell a command press
# from a value edit without restating the names inline.
_GPS_BUTTON_CONTROLS = ('use_current_location', 'set_location',
                        'goto_location', 'goto_position', 'stop')

_GPS_CONTROLS = {

    # ---- Position: the start location, which is also the teleport target ----
    #
    # The node seeds the simulated position from these on init and reset;
    # 'set_location' teleports there on demand.
    # Every GPS control below carries its own display_group, which is what makes
    # it render as one compact line -- row label in the left gutter, widget
    # inline -- instead of the stacked caption-over-widget block. That is the
    # same layout build_row_controls gives the NMEA and HNav fields, and it is
    # deliberate that each group holds exactly ONE control: Nepi_IF_Controls
    # dispatches on the group NAME being non-empty, not on the group holding
    # more than one control, so a group of one still takes the row path.
    #
    # Grouping also suppresses the control's read-only Min/Max block, because
    # renderBounds is a block element that would break the line. The bounds
    # below are still enforced -- nepi_controls clamps against them on every
    # write -- they are simply no longer drawn. The controls that cannot be
    # grouped (the two Floats axis rows and the slider) get the same result
    # from the show_bounds={false} the page passes when it mounts these sets.
    'start_latitude': {
        'type': 'Float', 'default': FACTORY_GPS_LATITUDE,
        'bounds': [MIN_LATITUDE_DEG, MAX_LATITUDE_DEG],
        'round': GEO_ROUND_PLACES, 'display_round': GEO_ROUND_PLACES,
        'display_name': 'Start Latitude',
        'display_group': 'start_latitude', 'display_width': _ROW_VALUE_WIDTH,
        'description': 'WGS84 latitude the simulated position starts and resets at'},

    'start_longitude': {
        'type': 'Float', 'default': FACTORY_GPS_LONGITUDE,
        'bounds': [MIN_LONGITUDE_DEG, MAX_LONGITUDE_DEG],
        'round': GEO_ROUND_PLACES, 'display_round': GEO_ROUND_PLACES,
        'display_name': 'Start Longitude',
        'display_group': 'start_longitude', 'display_width': _ROW_VALUE_WIDTH,
        'description': 'WGS84 longitude the simulated position starts and resets at'},

    'start_altitude_m': {
        'type': 'Float', 'default': FACTORY_GPS_ALTITUDE_M,
        'bounds': [MIN_ALTITUDE_M, MAX_ALTITUDE_M],
        'round': 2, 'display_round': 2,
        'display_name': 'Start Altitude (m)',
        'display_group': 'start_altitude_m', 'display_width': _ROW_VALUE_WIDTH,
        'description': 'WGS84 altitude the simulated position starts and resets at'},

    # Buttons group too. A grouped Button draws its caption once, in the row
    # gutter, instead of once as a header Label and again on the button face.
    'use_current_location': {
        'type': 'Button',
        'display_name': 'Use Current Location',
        'display_group': 'use_current_location',
        'description': 'Copy the live simulated position into the start and goto controls'},

    'set_location': {
        'type': 'Button',
        'display_name': 'Set Location (teleport)',
        'display_group': 'set_location',
        'description': 'Jump the simulated position to the start location with no interpolated move'},

    # ---- Move: the goto buffers and the commands that consume them ----
    # One Floats control, same shape as goto_position_m below, and for the same
    # display reason: display_row lays the three boxes on one line with each
    # axis caption above its own box. Three separately grouped controls cannot
    # produce that -- a row group prints ONE label, in the left gutter, and
    # every later control's caption inline AFTER its widget.
    #
    # The cost is the one a multi-value control always carries, and it is why
    # these were three controls before: Control.msg holds a SINGLE
    # min_bound/max_bound pair, so the three axes cannot each keep their own,
    # and the axes genuinely disagree -- a pair wide enough for altitude lets
    # latitude reach 20000, a pair that fits longitude caps altitude at 180 m.
    # No pair is right, so bounds are left unset (the -999 no-limit sentinel)
    # and the per-axis limits are enforced in _setGotoLocationGeoCb instead,
    # against the same constants. Same clamping, one step later, which is the
    # pattern _setSpeedCb already uses.
    'goto_location_geo': {
        'type': 'Floats',
        'default': [FACTORY_GPS_LATITUDE, FACTORY_GPS_LONGITUDE, FACTORY_GPS_ALTITUDE_M],
        'round': GEO_ROUND_PLACES, 'display_round': GEO_ROUND_PLACES,
        'display_row': True,
        'display_labels': _GPS_GEO_AXIS_LABELS,
        'display_name': 'Goto Location (WGS84)',
        'description': 'WGS84 latitude, longitude and altitude to simulate a move to'},

    'goto_location': {
        'type': 'Button',
        'display_name': 'Goto Location',
        'display_group': 'goto_location',
        'description': 'Simulate a move to the goto location'},

    # One Floats control rather than three Floats: a control carries a single
    # bound pair, and all three ENU axes share the same metre bound -- so this
    # one keeps its bounds. The three geopoint axes above do not share a bound,
    # which is why goto_location_geo declares none and clamps in its setter.
    #
    # NOT grouped, unlike every other control here. A grouped widget renders
    # with hide_label, which is what stops a row printing its caption twice --
    # but this control's per-component captions ARE its display_labels, and
    # hide_label suppresses exactly those, so grouping it would erase the
    # East/North/Up labels and leave three anonymous boxes. It keeps display_row
    # instead, which already lays the three axes out on one line, and the page's
    # show_bounds={false} takes care of its Min/Max block.
    'goto_position_m': {
        'type': 'Floats', 'default': [0.0, 0.0, 0.0],
        'bounds': [-MAX_ENU_OFFSET_M, MAX_ENU_OFFSET_M],
        'round': 2, 'display_round': 2, 'display_row': True,
        'display_labels': _GPS_ENU_AXIS_LABELS,
        'display_name': 'Goto Position (ENU meters)',
        'description': 'Relative move offset from the current simulated position'},

    'goto_position': {
        'type': 'Button',
        'display_name': 'Goto Position',
        'display_group': 'goto_position',
        'description': 'Simulate a move by the goto position offset'},

    'stop': {
        'type': 'Button',
        'display_name': 'Stop',
        'display_group': 'stop',
        'description': 'Stop the active simulated move and hold the current position'},

    # ---- Output: where the simulated fix goes and how it is described ----
    #
    # Selection, not Menu: a Menu value is the INDEX into its option list, and
    # this option list is rebuilt every discovery cycle, so an index would point
    # at a different mavros node as soon as one appeared or went away.
    'mavros_node': {
        'type': 'Selection', 'default': FACTORY_SELECTED_MAVROS,
        'options': [FACTORY_SELECTED_MAVROS],
        'display_name': 'Target Mavros Node',
        'description': 'Mavros node namespace to inject GPS_INPUT into'},

    'gps_pub_rate_hz': {
        'type': 'FloatSlider', 'default': float(GPS_PUB_RATE_HZ),
        'bounds': [MIN_GPS_PUB_RATE_HZ, MAX_GPS_PUB_RATE_HZ],
        'round': 2, 'display_round': 1,
        'display_name': 'GPS Publish Rate (Hz)',
        'description': 'Rate the simulated fix is published and injected at'},

    'satellites_visible': {
        'type': 'Int', 'default': FACTORY_SAT_COUNT,
        'bounds': [MIN_SAT_COUNT, MAX_SAT_COUNT],
        'display_name': 'Satellites Visible',
        'display_group': 'satellites_visible', 'display_width': _ROW_SMALL_WIDTH,
        'description': 'Satellite count reported in the injected GPS_INPUT message'},
}

# (section title, controls-name suffix, (control name, ...)) -- the same
# (title, suffix, payload) shape as _NMEA_SECTIONS and _HNAV_SECTIONS, but the
# payload names controls in _GPS_CONTROLS rather than describing generated rows.
_GPS_SECTIONS = (
    ('Position', CONTROLS_SUFFIX_POSITION, (
        'start_latitude', 'start_longitude', 'start_altitude_m',
        'use_current_location', 'set_location',
    )),
    ('Move', CONTROLS_SUFFIX_MOVE, (
        'goto_location_geo', 'goto_location',
        'goto_position_m', 'goto_position', 'stop',
    )),
    ('Output', CONTROLS_SUFFIX_OUTPUT, (
        'mavros_node', 'gps_pub_rate_hz', 'satellites_visible',
    )),
)

# Value controls only -- the attributes applyGpsControls reads back.
_GPS_VALUE_CONTROLS = tuple(name for name in _GPS_CONTROLS.keys()
                            if name not in _GPS_BUTTON_CONTROLS)


def build_gps_section_controls(control_names):
    """Build one GPS section's init dict, in declaration order."""
    controls = {}
    for name in control_names:
        controls[name] = copy.deepcopy(_GPS_CONTROLS[name])
    return controls


def checkGpsControls(msg_if):
    # create_controls_dict drops a malformed control and logs a warning rather
    # than raising, so a typo above costs one widget and nothing else says so.
    # Run it once at import-time cost and name what went missing.
    try:
        controls_dict = nepi_controls.create_controls_dict(_GPS_CONTROLS)
    except Exception as e:
        if msg_if is not None:
            msg_if.pub_warn('Nav Sim: could not validate GPS controls: ' + str(e))
        return
    missing = [name for name in _GPS_CONTROLS.keys()
               if name not in controls_dict.keys()]
    if len(missing) > 0 and msg_if is not None:
        msg_if.pub_warn('Nav Sim: GPS controls dropped at registration: ' + str(missing))


class ControlValue:
    """Stand-in for the std_msgs value the instance setter callbacks expect.

    The setters kept their original signatures through this migration, so the
    control routes hand them an object with a .data attribute exactly as the
    ROS subscribers did. Nothing about their per-field clamping moved.
    """

    def __init__(self, data):
        self.data = data


def setupInstanceControlSection(inst, title, suffix, fields, sin_fields = (),
                                init_dict = None):
    # The set name carries the instance identity because the namespace cannot --
    # see the CONTROLS_SUFFIX_* comment above. Keyed in _controls_ifs by the
    # SUFFIX, which is stable, rather than by the full name.
    #
    # node_if is left None so each IF builds and owns its own NodeClassIF, the
    # same choice NavPoseIF already makes in this node -- sharing one would
    # merge registries and a generic key would orphan a sibling's publisher.
    # pub_status and save_params are passed explicitly to match how fake_gps
    # mounts its set, even though both already default True.
    #
    # init_dict is given directly by the GPS kind, whose controls are a
    # hand-written set rather than generated rows. Left None the NMEA and HNav
    # kinds generate theirs from their field specs exactly as before.
    controls_name = instanceControlsName(inst._ns, inst._base_ns, suffix)
    if init_dict is None:
        init_dict = build_section_controls(fields, sin_fields = sin_fields)
    try:
        controls_if = ControlsIF(
            controls_name = controls_name,
            controls_display_name = title,
            controls_description = title + ' controls for ' + inst.name,
            controls_init_dict = init_dict,
            controls_updated_callback = inst.controlsUpdatedCb,
            pub_status = True,
            save_params = True,
            msg_if = inst._msg_if,
        )
        controls_if.wait_for_controls_ready(timeout = 10)
        inst._controls_ifs[suffix] = controls_if
    except Exception as e:
        # Same degrade-to-None contract NavPoseIF already has in this node: the
        # sim still runs and still publishes, it just loses that panel.
        if inst._msg_if is not None:
            inst._msg_if.pub_warn('Nav Sim: controls unavailable for ' +
                                  inst.name + ' ' + title + ': ' + str(e))
        inst._controls_ifs[suffix] = None


def findControlsIf(inst, control_name):
    # Which of the instance's section IFs owns this control. Looked up rather
    # than captured in the callback because ControlsIF takes its updated
    # callback at construction, before the IF it would have to close over
    # exists.
    for controls_if in inst._controls_ifs.values():
        if controls_if is None:
            continue
        try:
            if control_name in controls_if.get_controls_dict():
                return controls_if
        except Exception:
            continue
    return None


def setControlHidden(inst, control_name, hidden):
    controls_if = findControlsIf(inst, control_name)
    if controls_if is None:
        return
    try:
        controls_if.set_control_hidden(control_name, hidden)
    except Exception:
        pass


def applyControlUpdate(inst, control_name):
    # Route a changed control back through the instance's original setter, so
    # the sin/wave side effects (start time, base value, regenerated wave
    # components) happen exactly where they always did.
    route = inst._routes.get(control_name, None)
    if route is None:
        return
    controls_if = findControlsIf(inst, control_name)
    if controls_if is None:
        return
    try:
        value = controls_if.get_control_value(control_name)
    except Exception:
        return
    if value is None:
        return
    route(ControlValue(value))


def pushControlValuesByName(inst, names):
    # Push live instance state INTO the control sets, for a flat list of control
    # names. Needed after a config restore, where apply_dict writes the
    # attributes directly and the RUI would otherwise keep showing the values
    # the controls were built with.
    for name in names:
        if hasattr(inst, name) == False:
            continue
        controls_if = findControlsIf(inst, name)
        if controls_if is None:
            continue
        try:
            controls_if.set_control_value(name, getattr(inst, name))
        except Exception:
            continue


def pushInstanceControlValues(inst, sections):
    # The generated-row kinds' name list, derived from their field specs.
    names = []
    for title, suffix, fields in sections:
        for field, label, value_default in fields:
            names += [field, 'enable_move_' + field,
                      'move_step_' + field, 'move_rate_hz_' + field]
            if field in _SIN_FIELDS:
                names += ['enable_sin_' + field, 'sin_amplitude_' + field,
                          'sin_period_s_' + field, 'enable_wave_' + field,
                          'sin_spread_' + field]
    pushControlValuesByName(inst, names)


def cleanupInstanceControls(inst):
    for title in list(inst._controls_ifs.keys()):
        controls_if = inst._controls_ifs.get(title, None)
        if controls_if is None:
            continue
        try:
            controls_if.unregister()
        except Exception:
            pass
        inst._controls_ifs[title] = None


#########################################
# HNav packet helpers

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
_DEFAULT_STATUS  = 0x0002
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
# NMEA sentence helpers

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
# NMEA sim instance

class NmeaSimInstance:
    """Self-contained NMEA nav sim: own ROS subs/pub, TCP server, move thread."""

    def __init__(self, name, base_ns, msg_if):
        self.name    = name
        self._ns     = base_ns + '/nmea_instances/' + name
        self._base_ns = base_ns
        self._msg_if = msg_if
        self._lock   = threading.Lock()

        self.nmea_sim_enabled = FACTORY_NMEA_ENABLED
        self.nmea_latitude    = FACTORY_LATITUDE
        self.nmea_longitude   = FACTORY_LONGITUDE
        self.nmea_altitude_m  = FACTORY_ALTITUDE_M
        self.nmea_heading_deg = FACTORY_HEADING_DEG
        self.nmea_speed_ms    = FACTORY_SPEED_MS
        self.nmea_port        = FACTORY_NMEA_PORT
        self.nmea_connected   = False

        for f in _NMEA_MOVE_FIELDS:
            setattr(self, 'enable_move_' + f,   False)
            setattr(self, 'move_step_' + f,      0.0)
            setattr(self, 'move_rate_hz_' + f,   1.0)

        self._stop_evt       = None
        self._navpose_if     = None
        self._move_last_tick = {f: 0.0 for f in _NMEA_MOVE_FIELDS}
        self._alive          = True

        self._status_pub = nepi_sdk.create_publisher(
            self._ns + '/status', NepiAppNmeaSimStatus, queue_size=1, latch=True
        )
        self._subs = []
        self._controls_ifs = {}
        self._routes = {}
        self._registerSubs(self._ns)
        self._setupControls()
        threading.Thread(target=self._moveThreadLoop, daemon=True).start()

    def _registerSubs(self, ns):
        # Only the master enable stays a topic. It is not a row control -- the
        # RUI draws it as the toggle in the instance header, beside the port
        # readout. Every per-field setter below moved to the control sets built
        # in _setupControls; the setter METHODS are unchanged and the control
        # routes call exactly the same ones the subscribers used to.
        S = self._subs.append
        S(nepi_sdk.create_subscriber(ns + '/set_nmea_enabled',   Bool,    self._setNmeaEnabledCb))

    def _controlRoutes(self):
        # control name -> callable taking the stand-in value message. Built in
        # the same shape the old _registerSubs used, so the per-field behavior
        # (heading wrapped to 360, speed floored at 0) is the setter's, not a
        # reimplementation here.
        routes = {
            'nmea_latitude':    self._setLatitudeCb,
            'nmea_longitude':   self._setLongitudeCb,
            'nmea_altitude_m':  self._setAltitudeCb,
            'nmea_heading_deg': self._setHeadingCb,
            'nmea_speed_ms':    self._setSpeedCb,
        }
        for field in _NMEA_MOVE_FIELDS:
            routes['enable_move_'  + field] = lambda m, f=field: self._setEnableMoveCb(m, f)
            routes['move_step_'    + field] = lambda m, f=field: self._setMoveStepCb(m, f)
            routes['move_rate_hz_' + field] = lambda m, f=field: self._setMoveRateHzCb(m, f)
        return routes

    def _setupControls(self):
        self._controls_ifs = {}
        self._routes = self._controlRoutes()
        for title, suffix, fields in _NMEA_SECTIONS:
            setupInstanceControlSection(self, title, suffix, fields,
                                        sin_fields = ())
        self.syncRowVisibility()

    def syncRowVisibility(self):
        for title, suffix, fields in _NMEA_SECTIONS:
            for field, label, value_default in fields:
                auto = bool(getattr(self, 'enable_move_' + field))
                setControlHidden(self, 'move_step_' + field,    auto == False)
                setControlHidden(self, 'move_rate_hz_' + field, auto == False)

    def controlsUpdatedCb(self, control_name):
        applyControlUpdate(self, control_name)
        # An Auto toggle changes which boxes the row shows, so re-derive
        # visibility after every update rather than special-casing the name.
        self.syncRowVisibility()

    def _setNmeaEnabledCb(self, msg):
        with self._lock: self.nmea_sim_enabled = msg.data
        self._applyState()

    def _setLatitudeCb(self, msg):
        with self._lock: self.nmea_latitude = float(msg.data)
        self.publish_status()

    def _setLongitudeCb(self, msg):
        with self._lock: self.nmea_longitude = float(msg.data)
        self.publish_status()

    def _setAltitudeCb(self, msg):
        with self._lock: self.nmea_altitude_m = float(msg.data)
        self.publish_status()

    def _setHeadingCb(self, msg):
        with self._lock: self.nmea_heading_deg = float(msg.data) % 360.0
        self.publish_status()

    def _setSpeedCb(self, msg):
        with self._lock: self.nmea_speed_ms = max(0.0, float(msg.data))
        self.publish_status()

    def _setEnableMoveCb(self, msg, field):
        with self._lock: setattr(self, 'enable_move_' + field, msg.data)
        self.publish_status()

    def _setMoveStepCb(self, msg, field):
        with self._lock: setattr(self, 'move_step_' + field, float(msg.data))
        self.publish_status()

    def _setMoveRateHzCb(self, msg, field):
        with self._lock: setattr(self, 'move_rate_hz_' + field, float(msg.data))
        self.publish_status()

    def _moveThreadLoop(self):
        while self._alive:
            now = time.time()
            changed = False
            for field in _NMEA_MOVE_FIELDS:
                if not getattr(self, 'enable_move_' + field):
                    continue
                rate = getattr(self, 'move_rate_hz_' + field)
                if rate <= 0.0:
                    continue
                if (now - self._move_last_tick[field]) >= 1.0 / rate:
                    with self._lock:
                        setattr(self, field,
                                getattr(self, field) + getattr(self, 'move_step_' + field))
                    self._move_last_tick[field] = now
                    changed = True
            if changed:
                self.publish_status()
            time.sleep(1.0 / 20)

    def _applyState(self):
        with self._lock:
            enabled = self.nmea_sim_enabled
            port    = self.nmea_port
        if enabled and self._stop_evt is None:
            self._navpose_if = NavPoseIF(
                namespace=self._ns + '/navpose',
                data_source_description=f'nav_sim_{self.name}',
                data_ref_description='WGS84',
                pub_navpose=True, pub_location=True,
                pub_heading=True, pub_altitude=True,
                msg_if=self._msg_if,
            )
            stop_evt = threading.Event()
            self._stop_evt = stop_evt
            threading.Thread(target=self._serverLoop,
                             args=('127.0.0.1', port, stop_evt), daemon=True).start()
        elif not enabled and self._stop_evt is not None:
            self._stop_evt.set()
            self._stop_evt = None
            with self._lock: self.nmea_connected = False
            self._navpose_if = None
        self.publish_status()

    def _serverLoop(self, host, port, stop_evt):
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
                with self._lock: self.nmea_connected = True
                self.publish_status()
                self._serveClient(conn, stop_evt)
                with self._lock: self.nmea_connected = False
                self.publish_status()
        except Exception as e:
            self._msg_if.pub_warn(f"NMEA server error on {self.name}: {e}")
        finally:
            try: srv.close()
            except Exception: pass

    def _serveClient(self, conn, stop_evt):
        try:
            while not stop_evt.is_set():
                with self._lock:
                    lat, lon, alt = self.nmea_latitude, self.nmea_longitude, self.nmea_altitude_m
                    heading, speed = self.nmea_heading_deg, self.nmea_speed_ms
                sog_kts = speed * 1.944
                lines = [_make_GGA(lat, lon, alt),
                         _make_RMC(lat, lon, sog_kts, heading),
                         _make_VTG(heading, sog_kts),
                         _make_HDG(heading)]
                conn.sendall(("\r\n".join(lines) + "\r\n").encode("ascii"))
                if speed > 0:
                    self._deadReckon(1.0 / 5)
                time.sleep(1.0 / 5)
        except Exception:
            pass
        finally:
            try: conn.close()
            except Exception: pass

    def _deadReckon(self, dt_sec):
        with self._lock:
            heading_rad = math.radians(self.nmea_heading_deg)
            speed, lat  = self.nmea_speed_ms, self.nmea_latitude
        dist_m = speed * dt_sec
        dlat   = (dist_m / 111320.0) * math.cos(heading_rad)
        dlon   = (dist_m / (111320.0 * max(math.cos(math.radians(lat)), 1e-6))) * math.sin(heading_rad)
        with self._lock:
            self.nmea_latitude  += dlat
            self.nmea_longitude += dlon

    def publishNavpose(self):
        if self._navpose_if is None:
            return
        with self._lock:
            lat, lon, alt, heading = (self.nmea_latitude, self.nmea_longitude,
                                      self.nmea_altitude_m, self.nmea_heading_deg)
        t = nepi_sdk.get_time()
        d = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        d['has_location'] = True; d['latitude']    = lat;     d['longitude']   = lon; d['time_location'] = t
        d['has_heading']  = True; d['heading_deg'] = heading;                         d['time_heading']  = t
        d['has_altitude'] = True; d['altitude_m']  = alt;                             d['time_altitude'] = t
        self._navpose_if.publish_navpose(d)

    def publish_status(self):
        self._status_pub.publish(self._buildStatusMsg())

    def _buildStatusMsg(self):
        msg = NepiAppNmeaSimStatus()
        with self._lock:
            msg.nmea_sim_enabled = self.nmea_sim_enabled
            msg.nmea_connected   = self.nmea_connected
            msg.nmea_port        = self.nmea_port
            msg.nmea_latitude    = self.nmea_latitude
            msg.nmea_longitude   = self.nmea_longitude
            msg.nmea_altitude_m  = self.nmea_altitude_m
            msg.nmea_heading_deg = self.nmea_heading_deg
            msg.nmea_speed_ms    = self.nmea_speed_ms
            for f in _NMEA_MOVE_FIELDS:
                setattr(msg, 'enable_move_' + f,  getattr(self, 'enable_move_' + f))
                setattr(msg, 'move_step_' + f,     getattr(self, 'move_step_' + f))
                setattr(msg, 'move_rate_hz_' + f,  getattr(self, 'move_rate_hz_' + f))
        return msg

    def to_dict(self):
        with self._lock:
            d = {
                'nmea_sim_enabled': self.nmea_sim_enabled,
                'nmea_latitude':    self.nmea_latitude,
                'nmea_longitude':   self.nmea_longitude,
                'nmea_altitude_m':  self.nmea_altitude_m,
                'nmea_heading_deg': self.nmea_heading_deg,
                'nmea_speed_ms':    self.nmea_speed_ms,
                'nmea_port':        self.nmea_port,
            }
            for f in _NMEA_MOVE_FIELDS:
                d['enable_move_'  + f] = getattr(self, 'enable_move_'  + f)
                d['move_step_'    + f] = getattr(self, 'move_step_'    + f)
                d['move_rate_hz_' + f] = getattr(self, 'move_rate_hz_' + f)
        return d

    def apply_dict(self, d):
        need_apply = False
        with self._lock:
            for key in ('nmea_latitude', 'nmea_longitude', 'nmea_altitude_m',
                        'nmea_heading_deg', 'nmea_speed_ms'):
                if key in d:
                    setattr(self, key, float(d[key]))
            if 'nmea_port' in d:
                self.nmea_port = int(d['nmea_port'])
            for f in _NMEA_MOVE_FIELDS:
                k = 'enable_move_' + f
                if k in d: setattr(self, k, bool(d[k]))
                k = 'move_step_' + f
                if k in d: setattr(self, k, float(d[k]))
                k = 'move_rate_hz_' + f
                if k in d: setattr(self, k, float(d[k]))
            if 'nmea_sim_enabled' in d:
                new_enabled = bool(d['nmea_sim_enabled'])
                if new_enabled != self.nmea_sim_enabled:
                    self.nmea_sim_enabled = new_enabled
                    need_apply = True
        # The attributes above were written directly, so the control sets still
        # hold whatever they were built with. Push the restored state into them
        # and re-derive row visibility, or the RUI shows factory values next to
        # a sim that is already running on restored ones.
        pushInstanceControlValues(self, _NMEA_SECTIONS)
        self.syncRowVisibility()
        if need_apply:
            self._applyState()
        else:
            self.publish_status()

    def cleanup(self):
        self._alive = False
        if self._stop_evt is not None:
            self._stop_evt.set()
            self._stop_evt = None
        for sub in self._subs:
            sub.unregister()
        cleanupInstanceControls(self)
        if self._navpose_if is not None:
            self._navpose_if.unregister_pubs()
            self._navpose_if = None
        self._status_pub.unregister()


#########################################
# HNav sim instance

class HNavSimInstance:
    """Self-contained HNav nav sim: own ROS subs/pub, TCP server, move+sin+wave thread."""

    def __init__(self, name, base_ns, msg_if):
        self.name    = name
        self._ns     = base_ns + '/hnav_instances/' + name
        self._base_ns = base_ns
        self._msg_if = msg_if
        self._lock   = threading.Lock()

        self.hnav_sim_enabled = FACTORY_HNAV_ENABLED
        self.hnav_latitude    = FACTORY_LATITUDE
        self.hnav_longitude   = FACTORY_LONGITUDE
        self.hnav_altitude_m  = FACTORY_ALTITUDE_M
        self.hnav_depth_m     = FACTORY_DEPTH_M
        self.hnav_heading_deg = FACTORY_HEADING_DEG
        self.hnav_roll_deg    = FACTORY_ROLL_DEG
        self.hnav_pitch_deg   = FACTORY_PITCH_DEG
        self.hnav_speed_ms    = FACTORY_SPEED_MS
        self.hnav_port        = FACTORY_HNAV_PORT
        self.hnav_connected   = False

        for f in _HNAV_MOVE_FIELDS:
            setattr(self, 'enable_move_' + f,   False)
            setattr(self, 'move_step_' + f,      0.0)
            setattr(self, 'move_rate_hz_' + f,   1.0)
        for f in _SIN_FIELDS:
            setattr(self, 'enable_sin_' + f,    False)
            setattr(self, 'sin_amplitude_' + f, 5.0)
            setattr(self, 'sin_period_s_' + f,  10.0)
            setattr(self, 'enable_wave_' + f,   False)
            setattr(self, 'sin_spread_' + f,    0.5)

        self._stop_evt        = None
        self._navpose_if      = None
        self._move_last_tick  = {f: 0.0  for f in _HNAV_MOVE_FIELDS}
        self._sin_start_time  = {f: None for f in _SIN_FIELDS}
        self._sin_base        = {f: 0.0  for f in _SIN_FIELDS}
        self._wave_components = {f: None for f in _SIN_FIELDS}
        self._alive           = True

        self._status_pub = nepi_sdk.create_publisher(
            self._ns + '/status', NepiAppHNavSimStatus, queue_size=1, latch=True
        )
        self._subs = []
        self._controls_ifs = {}
        self._routes = {}
        self._registerSubs(self._ns)
        self._setupControls()
        threading.Thread(target=self._moveThreadLoop, daemon=True).start()

    def _registerSubs(self, ns):
        # As in NmeaSimInstance: only the master enable stays a topic, because
        # the RUI draws it in the instance header rather than as a row. Every
        # per-field setter moved to the control sets and the setter methods
        # themselves are untouched.
        S = self._subs.append
        S(nepi_sdk.create_subscriber(ns + '/set_hnav_enabled',   Bool,    self._setHnavEnabledCb))

    def _controlRoutes(self):
        routes = {
            'hnav_latitude':    self._setLatitudeCb,
            'hnav_longitude':   self._setLongitudeCb,
            'hnav_altitude_m':  self._setAltitudeCb,
            'hnav_depth_m':     self._setDepthCb,
            'hnav_heading_deg': self._setHeadingCb,
            'hnav_roll_deg':    self._setRollCb,
            'hnav_pitch_deg':   self._setPitchCb,
            'hnav_speed_ms':    self._setSpeedCb,
        }
        for field in _HNAV_MOVE_FIELDS:
            routes['enable_move_'  + field] = lambda m, f=field: self._setEnableMoveCb(m, f)
            routes['move_step_'    + field] = lambda m, f=field: self._setMoveStepCb(m, f)
            routes['move_rate_hz_' + field] = lambda m, f=field: self._setMoveRateHzCb(m, f)
        for field in _SIN_FIELDS:
            routes['enable_sin_'    + field] = lambda m, f=field: self._setEnableSinCb(m, f)
            routes['sin_amplitude_' + field] = lambda m, f=field: self._setSinAmplitudeCb(m, f)
            routes['sin_period_s_'  + field] = lambda m, f=field: self._setSinPeriodCb(m, f)
            routes['enable_wave_'   + field] = lambda m, f=field: self._setEnableWaveCb(m, f)
            routes['sin_spread_'    + field] = lambda m, f=field: self._setSinSpreadCb(m, f)
        return routes

    def _setupControls(self):
        self._controls_ifs = {}
        self._routes = self._controlRoutes()
        for title, suffix, fields in _HNAV_SECTIONS:
            setupInstanceControlSection(self, title, suffix, fields,
                                        sin_fields = _SIN_FIELDS)
        self.syncRowVisibility()

    def syncRowVisibility(self):
        # The reveal chain, mirroring what the hand-written rows did: Auto
        # reveals Step/Rate and the Sin toggle; Sin reveals Amp/Period and the
        # Wave toggle; Wave reveals Spread. Step and Rate hide again while Sin
        # owns the field, because the two motion modes are exclusive.
        for title, suffix, fields in _HNAV_SECTIONS:
            for field, label, value_default in fields:
                auto = bool(getattr(self, 'enable_move_' + field))
                has_sin = field in _SIN_FIELDS
                sin = has_sin and bool(getattr(self, 'enable_sin_' + field))
                wave = has_sin and bool(getattr(self, 'enable_wave_' + field))
                show_step = auto and (sin == False)
                setControlHidden(self, 'move_step_' + field,    show_step == False)
                setControlHidden(self, 'move_rate_hz_' + field, show_step == False)
                if has_sin == False:
                    continue
                setControlHidden(self, 'enable_sin_' + field,    auto == False)
                setControlHidden(self, 'sin_amplitude_' + field, sin == False)
                setControlHidden(self, 'sin_period_s_' + field,  sin == False)
                setControlHidden(self, 'enable_wave_' + field,   sin == False)
                setControlHidden(self, 'sin_spread_' + field,    wave == False)

    def controlsUpdatedCb(self, control_name):
        applyControlUpdate(self, control_name)
        self.syncRowVisibility()

    def _setHnavEnabledCb(self, msg):
        with self._lock: self.hnav_sim_enabled = msg.data
        self._applyState()

    def _setLatitudeCb(self, msg):
        with self._lock: self.hnav_latitude = float(msg.data)
        self.publish_status()

    def _setLongitudeCb(self, msg):
        with self._lock: self.hnav_longitude = float(msg.data)
        self.publish_status()

    def _setAltitudeCb(self, msg):
        with self._lock: self.hnav_altitude_m = float(msg.data)
        self.publish_status()

    def _setDepthCb(self, msg):
        with self._lock: self.hnav_depth_m = float(msg.data)
        self.publish_status()

    def _setHeadingCb(self, msg):
        with self._lock: self.hnav_heading_deg = float(msg.data) % 360.0
        self.publish_status()

    def _setRollCb(self, msg):
        with self._lock: self.hnav_roll_deg = float(msg.data)
        self.publish_status()

    def _setPitchCb(self, msg):
        with self._lock: self.hnav_pitch_deg = float(msg.data)
        self.publish_status()

    def _setSpeedCb(self, msg):
        with self._lock: self.hnav_speed_ms = max(0.0, float(msg.data))
        self.publish_status()

    def _setEnableMoveCb(self, msg, field):
        with self._lock: setattr(self, 'enable_move_' + field, msg.data)
        self.publish_status()

    def _setMoveStepCb(self, msg, field):
        with self._lock: setattr(self, 'move_step_' + field, float(msg.data))
        self.publish_status()

    def _setMoveRateHzCb(self, msg, field):
        with self._lock: setattr(self, 'move_rate_hz_' + field, float(msg.data))
        self.publish_status()

    def _setEnableSinCb(self, msg, field):
        with self._lock:
            setattr(self, 'enable_sin_' + field, msg.data)
            if msg.data:
                self._sin_start_time[field] = time.time()
                self._sin_base[field] = getattr(self, field)
                if getattr(self, 'enable_wave_' + field):
                    self._wave_components[field] = self._generateWaveComponents(field)
            else:
                self._sin_start_time[field] = None
        self.publish_status()

    def _setSinAmplitudeCb(self, msg, field):
        with self._lock:
            setattr(self, 'sin_amplitude_' + field, float(msg.data))
            if getattr(self, 'enable_wave_' + field):
                self._wave_components[field] = self._generateWaveComponents(field)
        self.publish_status()

    def _setSinPeriodCb(self, msg, field):
        with self._lock:
            setattr(self, 'sin_period_s_' + field, float(msg.data))
            if getattr(self, 'enable_wave_' + field):
                self._wave_components[field] = self._generateWaveComponents(field)
        self.publish_status()

    def _setEnableWaveCb(self, msg, field):
        with self._lock:
            setattr(self, 'enable_wave_' + field, msg.data)
            if msg.data:
                self._wave_components[field] = self._generateWaveComponents(field)
        self.publish_status()

    def _setSinSpreadCb(self, msg, field):
        with self._lock:
            setattr(self, 'sin_spread_' + field, float(msg.data))
            if getattr(self, 'enable_wave_' + field):
                self._wave_components[field] = self._generateWaveComponents(field)
        self.publish_status()

    def _generateWaveComponents(self, field):
        period    = getattr(self, 'sin_period_s_' + field)
        amplitude = getattr(self, 'sin_amplitude_' + field)
        spread    = float(getattr(self, 'sin_spread_' + field))
        f0 = 1.0 / period if period > 0.0 else 1.0
        if spread <= 0.0:
            return [(f0, amplitude, 0.0)]
        N  = 7
        bw = spread * f0
        freqs   = [f0 + bw * (i / (N - 1) - 0.5) for i in range(N)]
        sigma   = bw / 3.0
        weights = [math.exp(-0.5 * ((f - f0) / sigma) ** 2) for f in freqs]
        total   = sum(weights) or 1.0
        return [(f, amplitude * w / total, random.uniform(0.0, 2.0 * math.pi))
                for f, w in zip(freqs, weights)]

    def _moveThreadLoop(self):
        while self._alive:
            now     = time.time()
            changed = False
            for field in _HNAV_MOVE_FIELDS:
                if not getattr(self, 'enable_move_' + field):
                    continue
                if field in _SIN_FIELDS and getattr(self, 'enable_sin_' + field):
                    start = self._sin_start_time.get(field)
                    if start is not None:
                        elapsed = now - start
                        if getattr(self, 'enable_wave_' + field):
                            components = self._wave_components.get(field)
                            if components:
                                new_val = self._sin_base[field] + sum(
                                    a * math.sin(2.0 * math.pi * f * elapsed + ph)
                                    for f, a, ph in components
                                )
                                with self._lock: setattr(self, field, new_val)
                                changed = True
                        else:
                            period = getattr(self, 'sin_period_s_' + field)
                            if period > 0.0:
                                new_val = (self._sin_base[field] +
                                           getattr(self, 'sin_amplitude_' + field) *
                                           math.sin(2.0 * math.pi * elapsed / period))
                                with self._lock: setattr(self, field, new_val)
                                changed = True
                    continue
                rate = getattr(self, 'move_rate_hz_' + field)
                if rate <= 0.0:
                    continue
                if (now - self._move_last_tick[field]) >= 1.0 / rate:
                    with self._lock:
                        setattr(self, field,
                                getattr(self, field) + getattr(self, 'move_step_' + field))
                    self._move_last_tick[field] = now
                    changed = True
            if changed:
                self.publish_status()
            time.sleep(1.0 / 20)

    def _applyState(self):
        with self._lock:
            enabled = self.hnav_sim_enabled
            port    = self.hnav_port
        if enabled and self._stop_evt is None:
            self._navpose_if = NavPoseIF(
                namespace=self._ns + '/navpose',
                data_source_description=f'nav_sim_{self.name}',
                data_ref_description='WGS84',
                pub_navpose=True, pub_location=True,
                pub_heading=True, pub_orientation=True,
                pub_altitude=True, pub_depth=True,
                msg_if=self._msg_if,
            )
            stop_evt = threading.Event()
            self._stop_evt = stop_evt
            threading.Thread(target=self._serverLoop,
                             args=('127.0.0.1', port, stop_evt), daemon=True).start()
        elif not enabled and self._stop_evt is not None:
            self._stop_evt.set()
            self._stop_evt = None
            with self._lock: self.hnav_connected = False
            self._navpose_if = None
        self.publish_status()

    def _serverLoop(self, host, port, stop_evt):
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
                with self._lock: self.hnav_connected = True
                self.publish_status()
                self._serveClient(conn, stop_evt)
                with self._lock: self.hnav_connected = False
                self.publish_status()
        except Exception as e:
            self._msg_if.pub_warn(f"HNav server error on {self.name}: {e}")
        finally:
            try: srv.close()
            except Exception: pass

    def _serveClient(self, conn, stop_evt):
        rate_hz = 10
        period  = 1.0 / rate_hz
        try:
            while not stop_evt.is_set():
                with self._lock:
                    lat, lon   = self.hnav_latitude, self.hnav_longitude
                    alt, depth = self.hnav_altitude_m, self.hnav_depth_m
                    heading    = self.hnav_heading_deg
                    roll, pitch, speed = self.hnav_roll_deg, self.hnav_pitch_deg, self.hnav_speed_ms
                packet = _build_hnav_packet(
                    lat_deg=lat, lon_deg=lon, depth_m=depth, alt_m=alt,
                    roll_deg=roll, pitch_deg=pitch, heading_deg=heading, vel_fwd_ms=speed,
                )
                conn.sendall(packet)
                if speed > 0:
                    self._deadReckon(period)
                time.sleep(period)
        except Exception:
            pass
        finally:
            try: conn.close()
            except Exception: pass

    def _deadReckon(self, dt_sec):
        with self._lock:
            heading_rad = math.radians(self.hnav_heading_deg)
            speed, lat  = self.hnav_speed_ms, self.hnav_latitude
        dist_m = speed * dt_sec
        dlat   = (dist_m / 111320.0) * math.cos(heading_rad)
        dlon   = (dist_m / (111320.0 * max(math.cos(math.radians(lat)), 1e-6))) * math.sin(heading_rad)
        with self._lock:
            self.hnav_latitude  += dlat
            self.hnav_longitude += dlon

    def publishNavpose(self):
        if self._navpose_if is None:
            return
        with self._lock:
            lat, lon  = self.hnav_latitude, self.hnav_longitude
            alt, depth = self.hnav_altitude_m, self.hnav_depth_m
            heading, roll, pitch = self.hnav_heading_deg, self.hnav_roll_deg, self.hnav_pitch_deg
        t = nepi_sdk.get_time()
        d = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        d['has_location']    = True; d['latitude']    = lat;    d['longitude'] = lon;   d['time_location']    = t
        d['has_heading']     = True; d['heading_deg'] = heading;                        d['time_heading']     = t
        d['has_orientation'] = True; d['roll_deg']    = roll;   d['pitch_deg'] = pitch; d['yaw_deg'] = heading; d['time_orientation'] = t
        d['has_altitude']    = True; d['altitude_m']  = alt;                            d['time_altitude']    = t
        d['has_depth']       = True; d['depth_m']     = depth;                          d['time_depth']       = t
        self._navpose_if.publish_navpose(d)

    def publish_status(self):
        self._status_pub.publish(self._buildStatusMsg())

    def _buildStatusMsg(self):
        msg = NepiAppHNavSimStatus()
        with self._lock:
            msg.hnav_sim_enabled = self.hnav_sim_enabled
            msg.hnav_connected   = self.hnav_connected
            msg.hnav_port        = self.hnav_port
            msg.hnav_latitude    = self.hnav_latitude
            msg.hnav_longitude   = self.hnav_longitude
            msg.hnav_altitude_m  = self.hnav_altitude_m
            msg.hnav_depth_m     = self.hnav_depth_m
            msg.hnav_heading_deg = self.hnav_heading_deg
            msg.hnav_roll_deg    = self.hnav_roll_deg
            msg.hnav_pitch_deg   = self.hnav_pitch_deg
            msg.hnav_speed_ms    = self.hnav_speed_ms
            for f in _HNAV_MOVE_FIELDS:
                setattr(msg, 'enable_move_' + f,  getattr(self, 'enable_move_' + f))
                setattr(msg, 'move_step_' + f,     getattr(self, 'move_step_' + f))
                setattr(msg, 'move_rate_hz_' + f,  getattr(self, 'move_rate_hz_' + f))
            for f in _SIN_FIELDS:
                setattr(msg, 'enable_sin_' + f,    getattr(self, 'enable_sin_' + f))
                setattr(msg, 'sin_amplitude_' + f, getattr(self, 'sin_amplitude_' + f))
                setattr(msg, 'sin_period_s_' + f,  getattr(self, 'sin_period_s_' + f))
                setattr(msg, 'enable_wave_' + f,   getattr(self, 'enable_wave_' + f))
                setattr(msg, 'sin_spread_' + f,    getattr(self, 'sin_spread_' + f))
        return msg

    def to_dict(self):
        with self._lock:
            d = {
                'hnav_sim_enabled': self.hnav_sim_enabled,
                'hnav_latitude':    self.hnav_latitude,
                'hnav_longitude':   self.hnav_longitude,
                'hnav_altitude_m':  self.hnav_altitude_m,
                'hnav_depth_m':     self.hnav_depth_m,
                'hnav_heading_deg': self.hnav_heading_deg,
                'hnav_roll_deg':    self.hnav_roll_deg,
                'hnav_pitch_deg':   self.hnav_pitch_deg,
                'hnav_speed_ms':    self.hnav_speed_ms,
                'hnav_port':        self.hnav_port,
            }
            for f in _HNAV_MOVE_FIELDS:
                d['enable_move_'  + f] = getattr(self, 'enable_move_'  + f)
                d['move_step_'    + f] = getattr(self, 'move_step_'    + f)
                d['move_rate_hz_' + f] = getattr(self, 'move_rate_hz_' + f)
            for f in _SIN_FIELDS:
                d['enable_sin_'    + f] = getattr(self, 'enable_sin_'    + f)
                d['sin_amplitude_' + f] = getattr(self, 'sin_amplitude_' + f)
                d['sin_period_s_'  + f] = getattr(self, 'sin_period_s_'  + f)
                d['enable_wave_'   + f] = getattr(self, 'enable_wave_'   + f)
                d['sin_spread_'    + f] = getattr(self, 'sin_spread_'    + f)
        return d

    def apply_dict(self, d):
        need_apply = False
        with self._lock:
            for key in ('hnav_latitude', 'hnav_longitude', 'hnav_altitude_m', 'hnav_depth_m',
                        'hnav_heading_deg', 'hnav_roll_deg', 'hnav_pitch_deg', 'hnav_speed_ms'):
                if key in d:
                    setattr(self, key, float(d[key]))
            if 'hnav_port' in d:
                self.hnav_port = int(d['hnav_port'])
            for f in _HNAV_MOVE_FIELDS:
                k = 'enable_move_' + f
                if k in d: setattr(self, k, bool(d[k]))
                k = 'move_step_' + f
                if k in d: setattr(self, k, float(d[k]))
                k = 'move_rate_hz_' + f
                if k in d: setattr(self, k, float(d[k]))
            for f in _SIN_FIELDS:
                k = 'enable_sin_' + f
                if k in d: setattr(self, k, bool(d[k]))
                k = 'sin_amplitude_' + f
                if k in d: setattr(self, k, float(d[k]))
                k = 'sin_period_s_' + f
                if k in d: setattr(self, k, float(d[k]))
                k = 'enable_wave_' + f
                if k in d: setattr(self, k, bool(d[k]))
                k = 'sin_spread_' + f
                if k in d: setattr(self, k, float(d[k]))
            if 'hnav_sim_enabled' in d:
                new_enabled = bool(d['hnav_sim_enabled'])
                if new_enabled != self.hnav_sim_enabled:
                    self.hnav_sim_enabled = new_enabled
                    need_apply = True
        # As in NmeaSimInstance.apply_dict: the attributes above were written
        # directly, so push the restored state into the control sets and
        # re-derive row visibility before anything publishes.
        pushInstanceControlValues(self, _HNAV_SECTIONS)
        self.syncRowVisibility()
        if need_apply:
            self._applyState()
        else:
            self.publish_status()

    def cleanup(self):
        self._alive = False
        if self._stop_evt is not None:
            self._stop_evt.set()
            self._stop_evt = None
        for sub in self._subs:
            sub.unregister()
        cleanupInstanceControls(self)
        if self._navpose_if is not None:
            self._navpose_if.unregister_pubs()
            self._navpose_if = None
        self._status_pub.unregister()


#########################################
# GPS sim instance

class GpsSimInstance:
    """Self-contained simulated GPS: own ROS subs/pubs, move thread, MAVLink injection.

    Absorbed from nepi_app_fake_gps. Unlike the NMEA and HNav kinds it serves no
    TCP stream: it publishes a simulated NavSatFix and Odometry on its own
    instance namespace and injects a MAVLink GPS_INPUT into a selected mavros
    node. Discovery of the available mavros nodes is process-global and pushed
    down from the master class; this instance owns only the selection and the
    binding.
    """

    def __init__(self, name, base_ns, msg_if):
        self.name     = name
        self._ns      = base_ns + '/gps_instances/' + name
        self._base_ns = base_ns
        self._msg_if  = msg_if
        self._lock    = threading.Lock()

        self.gps_sim_enabled = FACTORY_GPS_ENABLED

        # Control-backed values. Each attribute name IS its control name, the
        # same convention the other two kinds use, which is what lets
        # pushControlValuesByName push restored state back into the sets.
        self.start_latitude     = FACTORY_GPS_LATITUDE
        self.start_longitude    = FACTORY_GPS_LONGITUDE
        self.start_altitude_m   = FACTORY_GPS_ALTITUDE_M
        self.goto_location_geo  = [FACTORY_GPS_LATITUDE, FACTORY_GPS_LONGITUDE,
                                   FACTORY_GPS_ALTITUDE_M]
        self.goto_position_m    = [0.0, 0.0, 0.0]
        self.mavros_node        = FACTORY_SELECTED_MAVROS
        self.gps_pub_rate_hz    = float(GPS_PUB_RATE_HZ)
        self.satellites_visible = FACTORY_SAT_COUNT

        # Simulated state (protected by _lock)
        self.current_location_wgs84_geo = self._makeGeoPoint(
            self.start_latitude, self.start_longitude, self.start_altitude_m)
        self.current_point = self._zeroPoint()
        # Previous ENU point + monotonic time for finite-difference velocity
        self._prev_vel_point = None
        self._prev_vel_time  = None
        # Simulated orientation: heading (deg true north) and ENU yaw (deg).
        # Derived from the horizontal direction of travel during a move; held at
        # the last value when stopped. Vehicle is treated as level (roll/pitch 0).
        self.current_heading_deg = 0.0
        self.current_yaw_enu_deg = 0.0
        self._move_plan = None
        self._alive     = True

        # Discovered list is pushed in by the master; the binding is ours.
        self.available_mavros_nodes = []
        self.mavlink_pub       = None
        self.bound_mavros_node = None
        self._navpose_if       = None

        self._status_pub = nepi_sdk.create_publisher(
            self._ns + '/status', NepiAppGpsSimStatus, queue_size=1, latch=True
        )
        self._fix_pub = nepi_sdk.create_publisher(
            self._ns + '/gps_fix', NavSatFix, queue_size=1
        )
        self._odom_pub = nepi_sdk.create_publisher(
            self._ns + '/odom', Odometry, queue_size=1
        )
        self._subs = []
        self._controls_ifs = {}
        self._routes = {}
        self._registerSubs(self._ns)
        self._setupControls()
        threading.Thread(target=self._moveThreadLoop, daemon=True).start()

    #######################
    ### Static Helpers

    def _zeroPoint(self):
        p = Point()
        p.x = 0.0
        p.y = 0.0
        p.z = 0.0
        return p

    def _makeGeoPoint(self, lat, lon, alt):
        geo = GeoPoint()
        geo.latitude  = float(lat)
        geo.longitude = float(lon)
        geo.altitude  = float(alt)
        return geo

    #######################
    ### ROS Interface

    def _registerSubs(self, ns):
        # As in the other two kinds, the master enable stays a topic because the
        # RUI draws it as the toggle in the instance header rather than as a row.
        #
        # The four command topics below came over from nepi_app_fake_gps and
        # stay topics for the reason the migration pattern gives: each carries a
        # whole geopoint or ENU offset in ONE message, which a control set --
        # where every value is written independently -- cannot express
        # atomically. They are this app's programmatic API for the GPS kind. The
        # Button controls in the Move section call the same private methods
        # these callbacks call, so there is exactly one implementation per
        # command.
        S = self._subs.append
        S(nepi_sdk.create_subscriber(ns + '/set_gps_enabled', Bool,     self._setGpsEnabledCb))
        S(nepi_sdk.create_subscriber(ns + '/reset_location',  GeoPoint, self.gpsResetLocCb))
        S(nepi_sdk.create_subscriber(ns + '/go_stop',         Empty,    self.gpsGoStopCb))
        S(nepi_sdk.create_subscriber(ns + '/goto_position',   Point,    self.gpsGoPosCb))
        S(nepi_sdk.create_subscriber(ns + '/goto_location',   GeoPoint, self.gpsGoLocCb))

    #######################
    ### Controls

    def _controlRoutes(self):
        # control name -> callable taking the stand-in value message, built in
        # the same shape the other two kinds use. The value routes go to the
        # setter that owns the clamping and the side effect; the Button routes
        # go to the same private command methods the command topics call, and
        # ignore the stand-in value (a trigger's value is its press time).
        return {
            'start_latitude':       self._setStartLatitudeCb,
            'start_longitude':      self._setStartLongitudeCb,
            'start_altitude_m':     self._setStartAltitudeCb,
            'goto_location_geo':    self._setGotoLocationGeoCb,
            'goto_position_m':      self._setGotoPositionMCb,
            'mavros_node':          self._setMavrosNodeCb,
            'gps_pub_rate_hz':      self._setGpsPubRateCb,
            'satellites_visible':   self._setSatellitesVisibleCb,
            'use_current_location': (lambda m: self.useCurrentLocation()),
            'set_location':         (lambda m: self.setLocation()),
            'goto_location':        (lambda m: self.gotoBufferedGeoLocation()),
            'goto_position':        (lambda m: self.gotoBufferedEnuPosition()),
            'stop':                 (lambda m: self.stopMove()),
        }

    def _setupControls(self):
        self._controls_ifs = {}
        self._routes = self._controlRoutes()
        checkGpsControls(self._msg_if)
        for title, suffix, control_names in _GPS_SECTIONS:
            setupInstanceControlSection(
                self, title, suffix, (),
                init_dict = build_gps_section_controls(control_names))
        # Seed the option list of the mavros selection from whatever the master
        # has already discovered, so an instance created after startup does not
        # come up with an empty selector until the next discovery pass.
        self.applyMavrosOptions()

    def controlsUpdatedCb(self, control_name):
        # No conditional rows in the GPS sets, so unlike the other two kinds
        # there is no syncRowVisibility pass to re-derive after an update.
        applyControlUpdate(self, control_name)

    def _getControlValue(self, control_name, fallback = None):
        controls_if = findControlsIf(self, control_name)
        if controls_if is None:
            return fallback
        try:
            value = controls_if.get_control_value(control_name)
        except Exception:
            return fallback
        if value is None:
            return fallback
        return value

    def _setControlValue(self, control_name, value):
        controls_if = findControlsIf(self, control_name)
        if controls_if is None:
            return
        try:
            controls_if.set_control_value(control_name, value)
        except Exception as e:
            if self._msg_if is not None:
                self._msg_if.pub_warn('Nav Sim: failed to write control ' +
                                      str(control_name) + ': ' + str(e))

    def _setControlOptions(self, control_name, options):
        controls_if = findControlsIf(self, control_name)
        if controls_if is None:
            return
        try:
            if controls_if.get_control_options(control_name) != options:
                # set_control_options publishes the set's status itself when the
                # list actually changes, so the RUI picks the new options up on
                # the same call.
                controls_if.set_control_options(control_name, options)
        except Exception as e:
            if self._msg_if is not None:
                self._msg_if.pub_warn('Nav Sim: failed to set options for control ' +
                                      str(control_name) + ': ' + str(e))

    #######################
    ### Setter Callbacks

    def _setGpsEnabledCb(self, msg):
        with self._lock: self.gps_sim_enabled = msg.data
        self._applyState()

    def _setStartLatitudeCb(self, msg):
        with self._lock: self.start_latitude = float(msg.data)
        self.publish_status()

    def _setStartLongitudeCb(self, msg):
        with self._lock: self.start_longitude = float(msg.data)
        self.publish_status()

    def _setStartAltitudeCb(self, msg):
        with self._lock: self.start_altitude_m = float(msg.data)
        self.publish_status()

    def _setGotoLocationGeoCb(self, msg):
        # The per-axis limits live here, not in the control's bounds: the three
        # axes do not share a range and a Control carries only one pair, so
        # goto_location_geo declares none and nepi_controls clamps nothing on
        # the way in. Clamping here keeps the limits the three separate
        # controls used to enforce, against the same constants. Setter-side
        # clamping is the existing pattern -- see _setSpeedCb.
        geo = msg.data
        if isinstance(geo, (list, tuple)) == False or len(geo) != 3:
            return
        with self._lock:
            self.goto_location_geo = [
                min(max(float(geo[0]), MIN_LATITUDE_DEG),  MAX_LATITUDE_DEG),
                min(max(float(geo[1]), MIN_LONGITUDE_DEG), MAX_LONGITUDE_DEG),
                min(max(float(geo[2]), MIN_ALTITUDE_M),    MAX_ALTITUDE_M)]
        self.publish_status()

    def _setGotoPositionMCb(self, msg):
        offset = msg.data
        if isinstance(offset, (list, tuple)) == False or len(offset) != 3:
            return
        with self._lock:
            self.goto_position_m = [float(offset[0]), float(offset[1]), float(offset[2])]
        self.publish_status()

    def _setMavrosNodeCb(self, msg):
        with self._lock: self.mavros_node = str(msg.data)
        self.bindSelectedMavros()
        self.publish_status()

    def _setGpsPubRateCb(self, msg):
        rate = float(msg.data)
        rate = max(MIN_GPS_PUB_RATE_HZ, min(MAX_GPS_PUB_RATE_HZ, rate))
        with self._lock: self.gps_pub_rate_hz = rate
        self.publish_status()

    def _setSatellitesVisibleCb(self, msg):
        count = int(msg.data)
        count = max(MIN_SAT_COUNT, min(MAX_SAT_COUNT, count))
        with self._lock: self.satellites_visible = count
        self.publish_status()

    #######################
    ### Mavros Binding
    #
    # The SCAN is process-global and lives on the master class; what is per
    # instance is which discovered node this simulator injects into.

    def set_available_mavros_nodes(self, nodes):
        """Accept the master's discovered mavros node list.

        Args:
            nodes (list): Mavros node namespaces currently on the wire, in
                discovery order.
        """
        nodes = list(nodes)
        if nodes == self.available_mavros_nodes:
            return
        self.available_mavros_nodes = nodes
        self.applyMavrosOptions()
        self.publish_status()

    def applyMavrosOptions(self):
        # 'None' stays first so a selection that is no longer on the wire falls
        # back to it rather than to some other vehicle's mavros node. The option
        # list must be set BEFORE the value: a Selection value that is not in
        # the current option list is rejected outright.
        self._setControlOptions('mavros_node',
                                [FACTORY_SELECTED_MAVROS] + self.available_mavros_nodes)
        selected = self.mavros_node
        if selected == FACTORY_SELECTED_MAVROS and len(self.available_mavros_nodes) > 0:
            # Writes THROUGH the control, which republishes the set's status and
            # calls back into controlsUpdatedCb, so self.mavros_node is refreshed
            # by the route rather than assigned here.
            self._setControlValue('mavros_node', self.available_mavros_nodes[0])
        elif selected != FACTORY_SELECTED_MAVROS and selected not in self.available_mavros_nodes:
            self._setControlValue('mavros_node', FACTORY_SELECTED_MAVROS)
        else:
            self.bindSelectedMavros()

    def bindSelectedMavros(self):
        # Returns True when the binding changed, so a caller has one place to
        # decide whether to republish.
        changed = False
        selected = self.mavros_node
        if selected in self.available_mavros_nodes:
            if self.bound_mavros_node != selected:
                self.bindMavros(selected)
                changed = True
        else:
            if self.bound_mavros_node is not None:
                self.unbindMavros()
                changed = True
        return changed

    def bindMavros(self, mavros_ns):
        self.unbindMavros()
        topic = nepi_sdk.create_namespace(mavros_ns, MAVROS_GPS_INPUT_TOPIC)
        self.mavlink_pub = nepi_sdk.create_publisher(topic, GPSINPUT, queue_size=1)
        self.bound_mavros_node = mavros_ns
        if self._msg_if is not None:
            self._msg_if.pub_info('GPS sim ' + self.name +
                                  ' will publish GPS_INPUT (with yaw) to: ' + topic)

    def unbindMavros(self):
        if self.mavlink_pub is not None:
            try:
                self.mavlink_pub.unregister()
            except Exception:
                pass
        self.mavlink_pub = None
        self.bound_mavros_node = None

    #######################
    # Command Callbacks
    #
    # One implementation per command, reached from two directions: these topics,
    # which carry a whole geopoint or offset in one message, and the Button
    # controls, which read the operator's buffered values out of the Move set.
    # Neither path reimplements the other.

    def gpsResetLocCb(self, geo_msg):
        geo_str = str([geo_msg.latitude, geo_msg.longitude, geo_msg.altitude])
        if self._msg_if is not None:
            self._msg_if.pub_info('GPS sim ' + self.name + ' reset to location: ' + geo_str)
        self.resetGpsLoc(geo_msg)

    def gpsGoStopCb(self, empty_msg):
        self.stopMove()

    def gpsGoPosCb(self, enu_point_msg):
        self.gotoEnuPosition(enu_point_msg.x, enu_point_msg.y, enu_point_msg.z)

    def gpsGoLocCb(self, geo_msg):
        self.gotoGeoLocation(geo_msg.latitude, geo_msg.longitude, geo_msg.altitude)

    #######################
    # Commands

    def useCurrentLocation(self):
        with self._lock:
            geo = copy.deepcopy(self.current_location_wgs84_geo)
        # Writes six controls, each of which calls back into controlsUpdatedCb.
        # Recursion terminates at depth two because none of those value routes
        # writes another control.
        self._setControlValue('start_latitude',    geo.latitude)
        self._setControlValue('start_longitude',   geo.longitude)
        self._setControlValue('start_altitude_m',  geo.altitude)
        self._setControlValue('goto_location_geo', [geo.latitude, geo.longitude,
                                                    geo.altitude])

    def setLocation(self):
        with self._lock:
            geo = self._makeGeoPoint(self.start_latitude, self.start_longitude,
                                     self.start_altitude_m)
        if self._msg_if is not None:
            self._msg_if.pub_info('GPS sim ' + self.name + ' setting location to start: ' +
                                  str([geo.latitude, geo.longitude, geo.altitude]))
        self.resetGpsLoc(geo)

    def resetGpsLoc(self, geo_msg):
        with self._lock:
            self._move_plan = None
            self.current_location_wgs84_geo = self._makeGeoPoint(
                geo_msg.latitude, geo_msg.longitude, geo_msg.altitude)
            self.current_point = self._zeroPoint()
        self.publish_status()

    def stopMove(self):
        if self.gps_sim_enabled == False:
            return
        with self._lock:
            self._move_plan = None

    def gotoBufferedGeoLocation(self):
        with self._lock:
            geo = list(self.goto_location_geo)
        if len(geo) != 3:
            return
        self.gotoGeoLocation(geo[0], geo[1], geo[2])

    def gotoBufferedEnuPosition(self):
        with self._lock:
            offset = list(self.goto_position_m)
        if len(offset) != 3:
            return
        self.gotoEnuPosition(offset[0], offset[1], offset[2])

    def gotoEnuPosition(self, east_m, north_m, up_m):
        if self.gps_sim_enabled == False:
            return
        with self._lock:
            cur_geo = copy.deepcopy(self.current_location_wgs84_geo)
        enu_point_msg = self._zeroPoint()
        enu_point_msg.x = float(east_m)
        enu_point_msg.y = float(north_m)
        enu_point_msg.z = float(up_m)
        new_enu_position = [enu_point_msg.x, enu_point_msg.y, enu_point_msg.z]
        new_geopoint_wgs84 = nepi_nav.get_geopoint_at_enu_point(cur_geo, new_enu_position)
        self.startMove(new_geopoint_wgs84, enu_point_msg)

    def gotoGeoLocation(self, latitude, longitude, altitude):
        if self.gps_sim_enabled == False:
            return
        geo_msg = self._makeGeoPoint(latitude, longitude, altitude)
        self.startMove(geo_msg, self._zeroPoint())

    #######################
    # Move Planning (non-blocking)

    def startMove(self, geopoint_msg, ned_delta_point):
        with self._lock:
            org_geo = np.array([self.current_location_wgs84_geo.latitude,
                                self.current_location_wgs84_geo.longitude,
                                self.current_location_wgs84_geo.altitude])
            new_geo = np.array([geopoint_msg.latitude,
                                geopoint_msg.longitude,
                                geopoint_msg.altitude])
            for ind in range(len(new_geo)):
                if new_geo[ind] == HOLD_SENTINEL:  # use current value for this axis
                    new_geo[ind] = org_geo[ind]
            delta_geo = new_geo - org_geo
            move_dist_m = nepi_nav.distance_geopoints(org_geo, new_geo)

            # Update simulated heading from the horizontal direction of travel
            self._updateHeading(org_geo, new_geo)

            org_point = np.array([self.current_point.x, self.current_point.y, self.current_point.z])
            delta_point = np.array([ned_delta_point.x, ned_delta_point.y, ned_delta_point.z])

            if move_dist_m <= 0:
                # Nothing to interpolate; snap any NED delta and finish
                self._move_plan = None
                self.current_point.x = float(org_point[0] + delta_point[0])
                self.current_point.y = float(org_point[1] + delta_point[1])
                self.current_point.z = float(org_point[2] + delta_point[2])
                return

            move_time = MOVE_UPDATE_TIME_SEC_PER_M * move_dist_m
            if move_time > MAX_MOVE_TIME_S:
                move_time = MAX_MOVE_TIME_S
            move_steps = int(move_time * self.gps_pub_rate_hz)
            if move_steps < 2:
                # Too short to ramp; snap to the target
                self.current_location_wgs84_geo = self._makeGeoPoint(
                    new_geo[0], new_geo[1], new_geo[2])
                self.current_point.x = float(org_point[0] + delta_point[0])
                self.current_point.y = float(org_point[1] + delta_point[1])
                self.current_point.z = float(org_point[2] + delta_point[2])
                self._move_plan = None
                return

            # Hanning-squared cumulative ramp for smooth ease-in/ease-out
            ramp = np.hanning(move_steps)
            ramp = ramp ** 2
            ramp_norm = ramp / np.sum(ramp)
            step_norm = np.zeros(len(ramp_norm))
            for ind in range(len(ramp_norm)):
                step_norm[ind] = np.sum(ramp_norm[0:ind])

            self._move_plan = {
                'org_geo': org_geo,
                'delta_geo': delta_geo,
                'org_point': org_point,
                'delta_point': delta_point,
                'step_norm': step_norm,
                'idx': 0,
            }
            if self._msg_if is not None:
                self._msg_if.pub_info('GPS sim %s moving %.2f meters in %.2f seconds (%d steps)'
                                      % (self.name, move_dist_m, move_time, move_steps))

    def _updateHeading(self, org_geo, new_geo):
        # Caller holds self._lock. Set heading/yaw from the horizontal direction of
        # travel (great-circle approximation over a short move). Holds the previous
        # heading when the move has no horizontal component (e.g. a pure altitude change).
        north = float(new_geo[0] - org_geo[0])
        east = float(new_geo[1] - org_geo[1]) * math.cos(math.radians(float(org_geo[0])))
        if math.hypot(north, east) > 1e-12:
            heading_deg = math.degrees(math.atan2(east, north)) % 360.0
            self.current_heading_deg = heading_deg
            self.current_yaw_enu_deg = nepi_nav.convert_yaw_ned2enu(heading_deg)

    def _advanceMove(self):
        # Caller holds self._lock. Advance the active move plan one step.
        plan = self._move_plan
        if plan is None:
            return
        step_norm = plan['step_norm']
        idx = plan['idx']
        if idx >= len(step_norm):
            # Snap exactly to the target and clear the plan
            final_geo = plan['org_geo'] + plan['delta_geo']
            final_point = plan['org_point'] + plan['delta_point']
            self.current_location_wgs84_geo = self._makeGeoPoint(
                final_geo[0], final_geo[1], final_geo[2])
            self.current_point.x = float(final_point[0])
            self.current_point.y = float(final_point[1])
            self.current_point.z = float(final_point[2])
            self._move_plan = None
            return
        val = step_norm[idx]
        cur_geo = plan['org_geo'] + plan['delta_geo'] * val
        cur_point = plan['org_point'] + plan['delta_point'] * val
        self.current_location_wgs84_geo.latitude = float(cur_geo[0])
        self.current_location_wgs84_geo.longitude = float(cur_geo[1])
        self.current_location_wgs84_geo.altitude = float(cur_geo[2])
        self.current_point.x = float(cur_point[0])
        self.current_point.y = float(cur_point[1])
        self.current_point.z = float(cur_point[2])
        plan['idx'] = idx + 1

    #######################
    # Simulate + Publish Thread

    def _moveThreadLoop(self):
        # The sibling kinds step their simulated fields here; this kind advances
        # its interpolated move plan and publishes the fix, which is what
        # fake_gps's _simThreadLoop did.
        while self._alive and not nepi_sdk.is_shutdown():
            with self._lock:
                self._advanceMove()
                geo     = copy.deepcopy(self.current_location_wgs84_geo)
                point   = copy.deepcopy(self.current_point)
                heading = self.current_heading_deg
                yaw_enu = self.current_yaw_enu_deg
                enabled = self.gps_sim_enabled
                rate    = self.gps_pub_rate_hz
            if enabled:
                self.publishFakeGps(geo, point, heading, yaw_enu)
            # Recompute each loop so a live rate change takes effect immediately
            if rate < MIN_GPS_PUB_RATE_HZ:
                rate = MIN_GPS_PUB_RATE_HZ
            time.sleep(1.0 / rate)

    def publishFakeGps(self, geo, point, heading_deg, yaw_enu_deg):
        if nepi_sdk.is_shutdown():
            return
        stamp = nepi_sdk.get_msg_stamp()

        # NavSatFix output
        navsatfix = NavSatFix()
        navsatfix.header.stamp = stamp
        navsatfix.latitude  = geo.latitude
        navsatfix.longitude = geo.longitude
        navsatfix.altitude  = geo.altitude

        # Odometry (ENU position relative to last reset) output, with orientation
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.pose.pose.position.x = point.x
        odom_msg.pose.pose.position.y = point.y
        odom_msg.pose.pose.position.z = point.z
        # Orientation quaternion: yaw about the ENU up-axis (roll/pitch 0, level vehicle)
        half_yaw_rad = math.radians(yaw_enu_deg) / 2.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(half_yaw_rad)
        odom_msg.pose.pose.orientation.w = math.cos(half_yaw_rad)

        self._fix_pub.publish(navsatfix)
        self._odom_pub.publish(odom_msg)

        # NavPose output at the simulated fix rate. The master status timer also
        # calls publishNavpose once a second, the way it does for the other two
        # kinds; this keeps the higher rate fake_gps published at.
        self.publishNavpose()

        # Earth-frame NED velocity for GPS_INPUT (0 while hovering, move velocity
        # during a goto). Computed every publish so the previous-sample state
        # stays fresh regardless of whether MAVLink injection is active.
        vn, ve, vd = self._computeNedVelocity(point)

        # MAVLink GPS_INPUT injection (the primary required output). GPS_INPUT
        # carries a yaw field, so the simulated GPS also supplies the heading the
        # EKF would otherwise take from a compass. With EK3_SRC1_YAW=2 (GPS) and
        # the compass disabled on the FCU, this lets a compass-less vehicle hold yaw.
        mavlink_pub = self.mavlink_pub
        if mavlink_pub is not None:
            gpsin = GPSINPUT()
            gpsin.header = Header(stamp=stamp, frame_id="mavlink_fake_gps")
            gpsin.fix_type = 3  # 3D fix
            gpsin.gps_id = 0
            # GPS time-of-week. Without valid, advancing GPS time AP_GPS reports
            # the receiver unhealthy and the EKF refuses a position solution, so
            # arming in a GPS mode fails with "Need Position Estimate". Derive it
            # from system UTC (absolute accuracy is not critical; it must be a
            # plausible current week and advance at real rate).
            gps_tow_s = time.time() - GPS_EPOCH_UNIX_S + GPS_LEAP_SECONDS
            gpsin.time_week = int(gps_tow_s // SECONDS_PER_WEEK)
            gpsin.time_week_ms = int((gps_tow_s % SECONDS_PER_WEEK) * 1000.0)
            gpsin.ignore_flags = 0  # provide position, altitude, and velocity
            gpsin.lat = int(round(geo.latitude * 1e7))
            gpsin.lon = int(round(geo.longitude * 1e7))
            gpsin.alt = float(geo.altitude)
            gpsin.hdop = 1.0
            gpsin.vdop = 1.0
            gpsin.vn = float(vn)
            gpsin.ve = float(ve)
            gpsin.vd = float(vd)
            gpsin.speed_accuracy = 0.5
            gpsin.horiz_accuracy = 1.0
            gpsin.vert_accuracy = 1.0
            gpsin.satellites_visible = self.satellites_visible
            # yaw: centidegrees true north, 0 = "not available", 36000 = north.
            # Map heading (0..360) so a 0-deg (north) heading still reports a value.
            yaw_cdeg = int(round(heading_deg * 100.0)) % 36000
            if yaw_cdeg == 0:
                yaw_cdeg = 36000
            gpsin.yaw = yaw_cdeg
            mavlink_pub.publish(gpsin)

    def _computeNedVelocity(self, point):
        # Finite-difference the simulated ENU position (point) into an
        # earth-frame NED velocity for GPS_INPUT. Returns (vn, ve, vd) m/s: 0
        # while hovering (point constant), the move velocity during a goto.
        # A reset/teleport produces a huge one-sample spike, which is rejected.
        now_s = time.monotonic()
        vn = ve = vd = 0.0
        prev = self._prev_vel_point
        prev_t = self._prev_vel_time
        if prev is not None and prev_t is not None:
            dt = now_s - prev_t
            if dt > 1e-3:
                cand_vn = (point.y - prev.y) / dt   # ENU north -> NED north
                cand_ve = (point.x - prev.x) / dt   # ENU east  -> NED east
                cand_vd = -(point.z - prev.z) / dt  # ENU up    -> NED down
                if max(abs(cand_vn), abs(cand_ve), abs(cand_vd)) < MAX_FAKE_GPS_SPEED_MPS:
                    vn, ve, vd = cand_vn, cand_ve, cand_vd
        self._prev_vel_point = copy.deepcopy(point)
        self._prev_vel_time = now_s
        return vn, ve, vd

    #######################
    ### State and Status

    def _applyState(self):
        # Mirrors the other two kinds: the NavPoseIF is mounted at the INSTANCE
        # namespace while the sim is enabled and dropped when it is not.
        # fake_gps mounted its NavPoseIF at the NODE namespace, which is the one
        # thing that could not be ported literally -- several GPS instances would
        # have collided on <node>/navpose.
        with self._lock:
            enabled = self.gps_sim_enabled
        if enabled and self._navpose_if is None:
            try:
                self._navpose_if = NavPoseIF(
                    namespace=self._ns + '/navpose',
                    data_source_description=f'nav_sim_{self.name}',
                    data_ref_description='WGS84',
                    pub_navpose=True, pub_location=True,
                    pub_heading=True, pub_orientation=True,
                    pub_altitude=True,
                    msg_if=self._msg_if,
                )
            except Exception as e:
                if self._msg_if is not None:
                    self._msg_if.pub_warn('Nav Sim: NavPose output unavailable for ' +
                                          self.name + ': ' + str(e))
                self._navpose_if = None
        elif enabled == False and self._navpose_if is not None:
            try:
                self._navpose_if.unregister_pubs()
            except Exception:
                pass
            self._navpose_if = None
        self.publish_status()

    def publishNavpose(self):
        if self._navpose_if is None:
            return
        with self._lock:
            geo     = copy.deepcopy(self.current_location_wgs84_geo)
            heading = self.current_heading_deg
            yaw_enu = self.current_yaw_enu_deg
        t = nepi_sdk.get_time()
        d = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        d['has_location']    = True; d['latitude']    = geo.latitude; d['longitude'] = geo.longitude; d['time_location']    = t
        d['has_altitude']    = True; d['altitude_m']  = geo.altitude;                                 d['time_altitude']    = t
        # The navpose dict is in the ENU frame, so yaw_deg is ENU yaw; roll and
        # pitch are 0 for a level vehicle.
        d['has_heading']     = True; d['heading_deg'] = heading;                                      d['time_heading']     = t
        d['has_orientation'] = True; d['roll_deg']    = 0.0; d['pitch_deg'] = 0.0; d['yaw_deg'] = yaw_enu; d['time_orientation'] = t
        try:
            self._navpose_if.publish_navpose(d)
        except Exception:
            pass

    def publish_status(self):
        self._status_pub.publish(self._buildStatusMsg())

    def _buildStatusMsg(self):
        msg = NepiAppGpsSimStatus()
        with self._lock:
            msg.gps_sim_enabled = self.gps_sim_enabled
            msg.available_mavros_nodes = list(self.available_mavros_nodes)
            selected = FACTORY_SELECTED_MAVROS
            if self.mavros_node in self.available_mavros_nodes:
                selected = self.mavros_node
            msg.selected_mavros_node = selected
            msg.mavros_connected = self.mavlink_pub is not None

            msg.current_latitude   = self.current_location_wgs84_geo.latitude
            msg.current_longitude  = self.current_location_wgs84_geo.longitude
            msg.current_altitude_m = self.current_location_wgs84_geo.altitude

            msg.current_point_x = self.current_point.x
            msg.current_point_y = self.current_point.y
            msg.current_point_z = self.current_point.z

            msg.current_heading_deg = float(self.current_heading_deg)
            msg.current_yaw_deg     = float(self.current_yaw_enu_deg)

            msg.moving = self._move_plan is not None
            msg.satellites_visible = int(self.satellites_visible)
            msg.gps_pub_rate_hz    = float(self.gps_pub_rate_hz)
        return msg

    def to_dict(self):
        with self._lock:
            d = {
                'gps_sim_enabled': self.gps_sim_enabled,
                # The live simulated position is carried too, so a rename --
                # which destroys and rebuilds the instance -- does not teleport
                # the vehicle back to its start location.
                'current_latitude':   self.current_location_wgs84_geo.latitude,
                'current_longitude':  self.current_location_wgs84_geo.longitude,
                'current_altitude_m': self.current_location_wgs84_geo.altitude,
                'current_heading_deg': self.current_heading_deg,
                'current_yaw_enu_deg': self.current_yaw_enu_deg,
            }
            for name in _GPS_VALUE_CONTROLS:
                value = getattr(self, name)
                d[name] = list(value) if isinstance(value, (list, tuple)) else value
        return d

    def apply_dict(self, d):
        need_apply = False
        with self._lock:
            for name in ('start_latitude', 'start_longitude', 'start_altitude_m',
                         'gps_pub_rate_hz'):
                if name in d:
                    setattr(self, name, float(d[name]))
            if 'satellites_visible' in d:
                self.satellites_visible = int(d['satellites_visible'])
            if 'mavros_node' in d:
                self.mavros_node = str(d['mavros_node'])
            if 'goto_location_geo' in d:
                geo = d['goto_location_geo']
                if isinstance(geo, (list, tuple)) and len(geo) == 3:
                    self.goto_location_geo = [float(v) for v in geo]
            if 'goto_position_m' in d:
                offset = d['goto_position_m']
                if isinstance(offset, (list, tuple)) and len(offset) == 3:
                    self.goto_position_m = [float(v) for v in offset]
            if ('current_latitude' in d and 'current_longitude' in d
                    and 'current_altitude_m' in d):
                self.current_location_wgs84_geo = self._makeGeoPoint(
                    d['current_latitude'], d['current_longitude'], d['current_altitude_m'])
            if 'current_heading_deg' in d:
                self.current_heading_deg = float(d['current_heading_deg'])
            if 'current_yaw_enu_deg' in d:
                self.current_yaw_enu_deg = float(d['current_yaw_enu_deg'])
            if 'gps_sim_enabled' in d:
                new_enabled = bool(d['gps_sim_enabled'])
                if new_enabled != self.gps_sim_enabled:
                    self.gps_sim_enabled = new_enabled
                    need_apply = True
        # As in the other two kinds: the attributes above were written directly,
        # so push the restored state into the control sets before anything
        # publishes, or the RUI shows factory values next to a sim already
        # running on restored ones.
        pushControlValuesByName(self, _GPS_VALUE_CONTROLS)
        self.applyMavrosOptions()
        if need_apply:
            self._applyState()
        else:
            self.publish_status()

    def cleanup(self):
        self._alive = False
        for sub in self._subs:
            sub.unregister()
        cleanupInstanceControls(self)
        self.unbindMavros()
        if self._navpose_if is not None:
            self._navpose_if.unregister_pubs()
            self._navpose_if = None
        self._fix_pub.unregister()
        self._odom_pub.unregister()
        self._status_pub.unregister()


#########################################
# Main coordinator node

class NepiNavSimApp:

    DEFAULT_NODE_NAME = "app_nav_sim"
    _CFG_FILE = '/mnt/nepi_storage/user_cfg/app_nav_sim_instances.yaml'

    def __init__(self):
        nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
        self.class_name     = type(self).__name__
        self.node_namespace = nepi_sdk.get_node_namespace()

        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Nav Sim App starting")

        self._inst_lock      = threading.Lock()
        self._nmea_instances = {}
        self._hnav_instances = {}
        self._gps_instances  = {}
        # Discovered mavros nodes. Process-global: one scan on this class, the
        # result pushed down into every GPS instance's selection control.
        self._available_mavros_nodes = []

        ns = self.node_namespace

        self._master_pub = nepi_sdk.create_publisher(
            ns + '/status', NepiAppNavSimMasterStatus, queue_size=1, latch=True
        )

        nepi_sdk.create_subscriber(ns + '/add_nmea_instance',    String,       self._addNmeaInstanceCb)
        nepi_sdk.create_subscriber(ns + '/remove_nmea_instance', String,       self._removeNmeaInstanceCb)
        nepi_sdk.create_subscriber(ns + '/rename_nmea_instance', UpdateString, self._renameNmeaInstanceCb)
        nepi_sdk.create_subscriber(ns + '/add_hnav_instance',    String,       self._addHnavInstanceCb)
        nepi_sdk.create_subscriber(ns + '/remove_hnav_instance', String,       self._removeHnavInstanceCb)
        nepi_sdk.create_subscriber(ns + '/rename_hnav_instance', UpdateString, self._renameHnavInstanceCb)
        nepi_sdk.create_subscriber(ns + '/add_gps_instance',     String,       self._addGpsInstanceCb)
        nepi_sdk.create_subscriber(ns + '/remove_gps_instance',  String,       self._removeGpsInstanceCb)
        nepi_sdk.create_subscriber(ns + '/rename_gps_instance',  UpdateString, self._renameGpsInstanceCb)

        nepi_sdk.create_subscriber(ns + '/save_config',          Empty, self._saveConfigCb)
        nepi_sdk.create_subscriber(ns + '/reset_config',         Empty, self._resetConfigCb)
        nepi_sdk.create_subscriber(ns + '/factory_reset_config', Empty, self._factoryResetConfigCb)

        saved_state = self._readConfigFile()
        nmea_cfg = saved_state.get('nmea', {}) if saved_state else {}
        hnav_cfg = saved_state.get('hnav', {}) if saved_state else {}
        gps_cfg  = saved_state.get('gps',  {}) if saved_state else {}
        if nmea_cfg:
            for name, inst_state in nmea_cfg.items():
                port = int(inst_state.get('nmea_port', FACTORY_NMEA_PORT)) if inst_state else FACTORY_NMEA_PORT
                self._createNmeaInstance(name, port)
        else:
            self._createNmeaInstance('nmea_0', FACTORY_NMEA_PORT)
        if hnav_cfg:
            for name, inst_state in hnav_cfg.items():
                port = int(inst_state.get('hnav_port', FACTORY_HNAV_PORT)) if inst_state else FACTORY_HNAV_PORT
                self._createHnavInstance(name, port)
        else:
            self._createHnavInstance('hnav_0', FACTORY_HNAV_PORT)
        if gps_cfg:
            for name in gps_cfg.keys():
                self._createGpsInstance(name)
        else:
            self._createGpsInstance('gps_0')
        if saved_state:
            self._applyConfigState(saved_state)
        self._publishMasterStatus()

        nepi_sdk.start_timer_process(1.0 / STATUS_RATE_HZ, self._timerCb)
        nepi_sdk.start_timer_process(1.0 / MAVROS_DISCOVER_RATE_HZ,
                                     self._discoverMavrosCb, oneshot=True)
        self.msg_if.pub_info("Nav Sim App ready")
        nepi_sdk.on_shutdown(self._cleanupAll)
        nepi_sdk.spin()

    def _timerCb(self, timer):
        self._publishMasterStatus()
        for inst in self._allInstances():
            inst.publish_status()
            inst.publishNavpose()

    def _allInstances(self):
        with self._inst_lock:
            return (list(self._nmea_instances.values()) +
                    list(self._hnav_instances.values()) +
                    list(self._gps_instances.values()))

    #######################
    ### Mavros Discovery
    #
    # PROCESS-GLOBAL, and deliberately not per instance: find_topics_by_msg
    # walks the whole master topic list, and every GPS instance would get the
    # same answer. One timer here, and the result is pushed down into each
    # instance's mavros-selection control options.

    def _discoverMavrosCb(self, timer):
        try:
            topics = nepi_sdk.find_topics_by_msg(MAVROS_STATE_MSG)
        except Exception as e:
            self.msg_if.pub_warn(f"Nav Sim: mavros discovery failed: {e}")
            topics = []
        available = []
        for topic in topics:
            if topic.endswith(MAVROS_STATE_SUFFIX):
                mavros_ns = topic[:-len(MAVROS_STATE_SUFFIX)]
                if mavros_ns not in available:
                    available.append(mavros_ns)
        if available != self._available_mavros_nodes:
            self._available_mavros_nodes = available
        with self._inst_lock:
            gps_insts = list(self._gps_instances.values())
        for inst in gps_insts:
            inst.set_available_mavros_nodes(available)
        nepi_sdk.start_timer_process(1.0 / MAVROS_DISCOVER_RATE_HZ,
                                     self._discoverMavrosCb, oneshot=True)

    def _createNmeaInstance(self, name, port):
        inst = NmeaSimInstance(name, self.node_namespace, self.msg_if)
        inst.nmea_port = port
        with self._inst_lock:
            self._nmea_instances[name] = inst
        self.msg_if.pub_info(f"NMEA sim instance '{name}' created (port:{port})")

    def _createHnavInstance(self, name, port):
        inst = HNavSimInstance(name, self.node_namespace, self.msg_if)
        inst.hnav_port = port
        with self._inst_lock:
            self._hnav_instances[name] = inst
        self.msg_if.pub_info(f"HNav sim instance '{name}' created (port:{port})")

    def _createGpsInstance(self, name):
        # No port: the GPS kind serves no TCP stream. It publishes its own
        # gps_fix/odom on its instance namespace and injects GPS_INPUT into a
        # selected mavros node, so the port plumbing the other two kinds carry
        # has no counterpart here -- which is why this is a third body rather
        # than a shared helper.
        inst = GpsSimInstance(name, self.node_namespace, self.msg_if)
        inst.set_available_mavros_nodes(self._available_mavros_nodes)
        with self._inst_lock:
            self._gps_instances[name] = inst
        self.msg_if.pub_info(f"GPS sim instance '{name}' created")

    def _addNmeaInstanceCb(self, msg):
        name = msg.data.strip() or self._autoNmeaName()
        with self._inst_lock:
            if name in self._nmea_instances:
                return
        port = self._nextNmeaPort()
        self._createNmeaInstance(name, port)
        self._publishMasterStatus()

    def _removeNmeaInstanceCb(self, msg):
        name = msg.data.strip()
        with self._inst_lock:
            if name not in self._nmea_instances or len(self._nmea_instances) <= 1:
                return
            inst = self._nmea_instances.pop(name)
        inst.cleanup()
        self._publishMasterStatus()

    def _addHnavInstanceCb(self, msg):
        name = msg.data.strip() or self._autoHnavName()
        with self._inst_lock:
            if name in self._hnav_instances:
                return
        port = self._nextHnavPort()
        self._createHnavInstance(name, port)
        self._publishMasterStatus()

    def _removeHnavInstanceCb(self, msg):
        name = msg.data.strip()
        with self._inst_lock:
            if name not in self._hnav_instances or len(self._hnav_instances) <= 1:
                return
            inst = self._hnav_instances.pop(name)
        inst.cleanup()
        self._publishMasterStatus()

    def _addGpsInstanceCb(self, msg):
        name = msg.data.strip() or self._autoGpsName()
        with self._inst_lock:
            if name in self._gps_instances:
                return
        self._createGpsInstance(name)
        self._publishMasterStatus()

    def _removeGpsInstanceCb(self, msg):
        name = msg.data.strip()
        with self._inst_lock:
            if name not in self._gps_instances or len(self._gps_instances) <= 1:
                return
            inst = self._gps_instances.pop(name)
        inst.cleanup()
        self._publishMasterStatus()

    def _renameNmeaInstanceCb(self, msg):
        old_name = msg.name.strip()
        new_name = msg.value.strip()
        if not new_name or old_name == new_name:
            return
        with self._inst_lock:
            if old_name not in self._nmea_instances or new_name in self._nmea_instances:
                return
            inst = self._nmea_instances.pop(old_name)
            port = inst.nmea_port
        old_state = inst.to_dict()
        inst.cleanup()
        self._createNmeaInstance(new_name, port)
        with self._inst_lock:
            new_inst = self._nmea_instances[new_name]
        new_inst.apply_dict(old_state)
        self._publishMasterStatus()

    def _renameHnavInstanceCb(self, msg):
        old_name = msg.name.strip()
        new_name = msg.value.strip()
        if not new_name or old_name == new_name:
            return
        with self._inst_lock:
            if old_name not in self._hnav_instances or new_name in self._hnav_instances:
                return
            inst = self._hnav_instances.pop(old_name)
            port = inst.hnav_port
        old_state = inst.to_dict()
        inst.cleanup()
        self._createHnavInstance(new_name, port)
        with self._inst_lock:
            new_inst = self._hnav_instances[new_name]
        new_inst.apply_dict(old_state)
        self._publishMasterStatus()

    def _renameGpsInstanceCb(self, msg):
        old_name = msg.name.strip()
        new_name = msg.value.strip()
        if not new_name or old_name == new_name:
            return
        with self._inst_lock:
            if old_name not in self._gps_instances or new_name in self._gps_instances:
                return
            inst = self._gps_instances.pop(old_name)
        old_state = inst.to_dict()
        inst.cleanup()
        self._createGpsInstance(new_name)
        with self._inst_lock:
            new_inst = self._gps_instances[new_name]
        new_inst.apply_dict(old_state)
        self._publishMasterStatus()

    def _publishMasterStatus(self):
        msg = NepiAppNavSimMasterStatus()
        with self._inst_lock:
            msg.nmea_instance_names = list(self._nmea_instances.keys())
            msg.hnav_instance_names = list(self._hnav_instances.keys())
            msg.gps_instance_names  = list(self._gps_instances.keys())
        self._master_pub.publish(msg)

    def _autoNmeaName(self):
        with self._inst_lock:
            i = 0
            while f'nmea_{i}' in self._nmea_instances:
                i += 1
        return f'nmea_{i}'

    def _autoHnavName(self):
        with self._inst_lock:
            i = 0
            while f'hnav_{i}' in self._hnav_instances:
                i += 1
        return f'hnav_{i}'

    def _autoGpsName(self):
        with self._inst_lock:
            i = 0
            while f'gps_{i}' in self._gps_instances:
                i += 1
        return f'gps_{i}'

    def _nextNmeaPort(self):
        with self._inst_lock:
            used = {inst.nmea_port for inst in self._nmea_instances.values()}
        p = FACTORY_NMEA_PORT
        while p in used:
            p += 1
        return p

    def _nextHnavPort(self):
        with self._inst_lock:
            used = {inst.hnav_port for inst in self._hnav_instances.values()}
        p = FACTORY_HNAV_PORT
        while p in used:
            p += 1
        return p

    def _readConfigFile(self):
        if not os.path.isfile(self._CFG_FILE):
            return None
        try:
            with open(self._CFG_FILE, 'r') as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            self.msg_if.pub_warn(f"Nav Sim: failed to read config: {e}")
            return None

    def _applyConfigState(self, state):
        with self._inst_lock:
            nmea_pairs = list(self._nmea_instances.items())
            hnav_pairs = list(self._hnav_instances.items())
            gps_pairs  = list(self._gps_instances.items())
        for name, inst in nmea_pairs:
            if name in state.get('nmea', {}):
                inst.apply_dict(state['nmea'][name])
        for name, inst in hnav_pairs:
            if name in state.get('hnav', {}):
                inst.apply_dict(state['hnav'][name])
        for name, inst in gps_pairs:
            if name in state.get('gps', {}):
                inst.apply_dict(state['gps'][name])

    def _loadConfig(self):
        state = self._readConfigFile()
        if state:
            self._applyConfigState(state)
            self.msg_if.pub_info("Nav Sim: config loaded")

    def _saveConfig(self):
        state = {'nmea': {}, 'hnav': {}, 'gps': {}}
        with self._inst_lock:
            for name, inst in self._nmea_instances.items():
                state['nmea'][name] = inst.to_dict()
            for name, inst in self._hnav_instances.items():
                state['hnav'][name] = inst.to_dict()
            for name, inst in self._gps_instances.items():
                state['gps'][name] = inst.to_dict()
        try:
            os.makedirs(os.path.dirname(self._CFG_FILE), exist_ok=True)
            with open(self._CFG_FILE, 'w') as f:
                yaml.safe_dump(state, f, default_flow_style=False)
            self.msg_if.pub_info("Nav Sim: config saved")
        except Exception as e:
            self.msg_if.pub_warn(f"Nav Sim: failed to save config: {e}")

    def _saveConfigCb(self, msg):
        self._saveConfig()

    def _resetConfigCb(self, msg):
        self._loadConfig()

    def _factoryResetConfigCb(self, msg):
        try:
            os.remove(self._CFG_FILE)
        except OSError:
            pass
        with self._inst_lock:
            all_nmea = list(self._nmea_instances.values())
            all_hnav = list(self._hnav_instances.values())
            all_gps  = list(self._gps_instances.values())
            self._nmea_instances.clear()
            self._hnav_instances.clear()
            self._gps_instances.clear()
        for inst in all_nmea + all_hnav + all_gps:
            inst.cleanup()
        self._createNmeaInstance('nmea_0', FACTORY_NMEA_PORT)
        self._createHnavInstance('hnav_0', FACTORY_HNAV_PORT)
        self._createGpsInstance('gps_0')
        self._publishMasterStatus()
        self.msg_if.pub_info("Nav Sim: config reset to factory defaults")

    def _cleanupAll(self):
        all_insts = self._allInstances()
        for inst in all_insts:
            inst.cleanup()
        self.msg_if.pub_info("Nav Sim App shutdown")


#########################################
if __name__ == '__main__':
    NepiNavSimApp()
