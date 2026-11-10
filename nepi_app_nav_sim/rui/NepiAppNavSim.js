/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi rui (nepi_apps) repo
# (see https://github.com/nepi-engine/nepi_apps)
#
# License: NEPI RUI repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#
 */

import React, { Component } from "react"
import { observer, inject } from "mobx-react"

import Section from "./Section"
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Input from "./Input"
import Button from "./Button"
import AsyncToggle from "./AsyncToggle"
// react-toggle (not AsyncToggle): the only Toggle this file renders with it is
// Advanced Options, whose checked value is local view state the click sets
// synchronously -- there is no backend round trip to confirm, so optimistic
// position would add a revert timer with nothing to clear it. Every toggle here
// that does reach the node stays an AsyncToggle.
import Toggle from "react-toggle"
import Styles from "./Styles"

import NepiIFConfig from "./Nepi_IF_Config"
import NepiIFControls from "./Nepi_IF_Controls"
import { setElementStyleModified, clearElementStyleModified } from "./Utilities"

// Section suffixes of the per-instance control sets, matching the
// CONTROLS_SUFFIX_* constants in nav_sim_app_node.py.
//
// A ControlsIF is always a direct child of the NODE namespace -- it builds
// create_namespace(node_namespace, controls_name) with no namespace argument,
// and get_clean_name() rewrites '/' to '_', so a set cannot live under the
// instance namespace. The instance identity therefore has to be carried in the
// NAME, and both sides derive the same string from the instance's ROS path:
//     <app_ns>/<kind>_instances_<instance name>_<suffix>
// e.g. <app_ns>/nmea_instances_nmea_0_position
const CONTROLS_SUFFIX_POSITION      = "position"
const CONTROLS_SUFFIX_ORIENTATION   = "orientation"
const CONTROLS_SUFFIX_DEADRECKONING = "dead_reckoning"
// GPS sections. The GPS kind reuses the position suffix for its start location
// and adds two of its own; it has no settable orientation and no dead-reckoning
// speed, so those two suffixes are not forced onto it.
const CONTROLS_SUFFIX_MOVE          = "move"
const CONTROLS_SUFFIX_OUTPUT        = "output"

// One configuration per simulator kind, replacing what used to be two
// near-identical render bodies. renderKindPanel is parameterised by these, so
// adding the GPS kind added a third entry rather than a third copy.
//
// Each entry declares: the instance-path segment the node roots that kind at,
// the master-status field listing its instances, the component-state keys the
// selector writes, the per-instance status message type and the fields the
// header reads out of it, the enable topic, and the section list.
const KIND_CONFIGS = {
  hnav: {
    key:            "hnav",
    label:          "HNav",
    title:          "HNav Sim",
    instancesPath:  "hnav_instances",
    masterField:    "hnav_instance_names",
    namesKey:       "hnavInstanceNames",
    selectedKey:    "selectedHnavInstance",
    renameKey:      "hnavRenameInput",
    renameInputId:  "hnavRenameInput",
    statusType:     "nepi_app_nav_sim/NepiAppHNavSimStatus",
    enableTopic:    "set_hnav_enabled",
    enabledField:   "hnav_sim_enabled",
    addTopic:       "add_hnav_instance",
    removeTopic:    "remove_hnav_instance",
    renameTopic:    "rename_hnav_instance",
    sections: [
      { title: "Position",       suffix: CONTROLS_SUFFIX_POSITION },
      { title: "Orientation",    suffix: CONTROLS_SUFFIX_ORIENTATION },
      { title: "Dead-Reckoning", suffix: CONTROLS_SUFFIX_DEADRECKONING },
    ],
    stopSuffix:  CONTROLS_SUFFIX_DEADRECKONING,
    stopControl: "hnav_speed_ms",
  },
  nmea: {
    key:            "nmea",
    label:          "NMEA",
    title:          "NMEA Sim",
    instancesPath:  "nmea_instances",
    masterField:    "nmea_instance_names",
    namesKey:       "nmeaInstanceNames",
    selectedKey:    "selectedNmeaInstance",
    renameKey:      "nmeaRenameInput",
    renameInputId:  "nmeaRenameInput",
    statusType:     "nepi_app_nav_sim/NepiAppNmeaSimStatus",
    enableTopic:    "set_nmea_enabled",
    enabledField:   "nmea_sim_enabled",
    addTopic:       "add_nmea_instance",
    removeTopic:    "remove_nmea_instance",
    renameTopic:    "rename_nmea_instance",
    sections: [
      { title: "Position",       suffix: CONTROLS_SUFFIX_POSITION },
      { title: "Orientation",    suffix: CONTROLS_SUFFIX_ORIENTATION },
      { title: "Dead-Reckoning", suffix: CONTROLS_SUFFIX_DEADRECKONING },
    ],
    stopSuffix:  CONTROLS_SUFFIX_DEADRECKONING,
    stopControl: "nmea_speed_ms",
  },
  gps: {
    key:            "gps",
    label:          "GPS",
    title:          "GPS Sim",
    instancesPath:  "gps_instances",
    masterField:    "gps_instance_names",
    namesKey:       "gpsInstanceNames",
    selectedKey:    "selectedGpsInstance",
    renameKey:      "gpsRenameInput",
    renameInputId:  "gpsRenameInput",
    statusType:     "nepi_app_nav_sim/NepiAppGpsSimStatus",
    enableTopic:    "set_gps_enabled",
    enabledField:   "gps_sim_enabled",
    addTopic:       "add_gps_instance",
    removeTopic:    "remove_gps_instance",
    renameTopic:    "rename_gps_instance",
    // advanced: true keeps a section behind the Advanced Options toggle. The
    // main view is the start position plus the two location buttons -- the GPS
    // equivalent of what the NMEA kind shows -- and the goto machinery and the
    // MAVLink injection settings sit underneath it.
    sections: [
      { title: "Position", suffix: CONTROLS_SUFFIX_POSITION },
      { title: "Move",     suffix: CONTROLS_SUFFIX_MOVE,   advanced: true },
      { title: "Output",   suffix: CONTROLS_SUFFIX_OUTPUT, advanced: true },
    ],
    // Every GPS control is bounded, and drawing each one's read-only Min/Max
    // block doubled the height of the panel. Grouped controls already omit it;
    // this turns it off for the two that cannot be grouped (the ENU axis row
    // and the publish-rate slider). The bounds still clamp on the node side.
    hideBounds: true,
    // The GPS kind's Stop is a Button control inside its Move set, so the page
    // draws no separate Stop button for it.
    stopSuffix:  null,
    stopControl: null,
  },
}

const KIND_ORDER = ["hnav", "nmea", "gps"]

@inject("ros")
@observer

class NepiAppNavSim extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_nav_sim",
      appNamespace: null,

      // Which simulator kind the page is showing. LOCAL view state only: all
      // three kinds keep running in the node regardless, so this is a filter
      // over what is rendered and has no topic, param or status field.
      kind: "hnav",

      // Header fields of the SELECTED kind's instance, filled by
      // instStatusListener. Only the selected kind is subscribed.
      inst_enabled:   false,
      nmea_connected: false,
      nmea_port:      50000,
      hnav_connected: false,
      hnav_port:      16718,

      // GPS header and read-only state, from NepiAppGpsSimStatus
      gps_mavros_connected:    false,
      gps_selected_mavros:     "None",
      gps_current_latitude:    0.0,
      gps_current_longitude:   0.0,
      gps_current_altitude_m:  0.0,
      gps_current_heading_deg: 0.0,
      gps_moving:              false,

      // Per-field values, move/sin/wave enables and their step, rate, amplitude,
      // period and spread buffers all live in the per-instance control sets.
      // Nepi_IF_Controls owns that state, read from each set's ControlsStatus,
      // so none of it is mirrored here.

      // Listeners
      masterStatusListener: null,
      instStatusListener:   null,
      connected: false,

      // Instance management
      nmeaInstanceNames:    ['nmea_0'],
      selectedNmeaInstance: 'nmea_0',
      nmeaRenameInput:      'nmea_0',
      hnavInstanceNames:    ['hnav_0'],
      selectedHnavInstance: 'hnav_0',
      hnavRenameInput:      'hnav_0',
      gpsInstanceNames:     ['gps_0'],
      selectedGpsInstance:  'gps_0',
      gpsRenameInput:       'gps_0',

      // Advanced Options. A view preference, like the kind selector above it:
      // the sections it hides keep running and keep publishing, they are just
      // not drawn. Per kind, so opening GPS's advanced sections does not also
      // open anything on a kind that later grows some.
      showAdvanced: {},
    }

    this.getAppNamespace           = this.getAppNamespace.bind(this)
    this.getInstanceNamespace      = this.getInstanceNamespace.bind(this)
    this.masterStatusListener      = this.masterStatusListener.bind(this)
    this.instStatusListener        = this.instStatusListener.bind(this)
    this.updateMasterStatusListener = this.updateMasterStatusListener.bind(this)
    this.updateInstStatusListener  = this.updateInstStatusListener.bind(this)
    this.getControlsNamespace      = this.getControlsNamespace.bind(this)
    this.renderControlSection      = this.renderControlSection.bind(this)
    this.renderKindSelector        = this.renderKindSelector.bind(this)
    this.renderInstanceSelector    = this.renderInstanceSelector.bind(this)
    this.renderKindPanel           = this.renderKindPanel.bind(this)
    this.renderGpsState            = this.renderGpsState.bind(this)
    this.renderConfig              = this.renderConfig.bind(this)
  }

  getAppNamespace() {
    const { namespacePrefix, deviceId } = this.props.ros
    if (namespacePrefix !== null && deviceId !== null) {
      return "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
    }
    return null
  }

  // Instance namespace of one kind, defaulting to the kind the page is showing.
  getInstanceNamespace(kind = null) {
    const cfg = KIND_CONFIGS[kind === null ? this.state.kind : kind]
    const appNs = this.getAppNamespace()
    if (appNs === null || cfg === undefined) return null
    return appNs + '/' + cfg.instancesPath + '/' + this.state[cfg.selectedKey]
  }

  masterStatusListener(message) {
    const pick = (field, fallback) => (
      Array.isArray(message[field]) && message[field].length > 0
        ? message[field] : fallback
    )
    this.setState({
      nmeaInstanceNames: pick('nmea_instance_names', ['nmea_0']),
      hnavInstanceNames: pick('hnav_instance_names', ['hnav_0']),
      gpsInstanceNames:  pick('gps_instance_names',  ['gps_0']),
      connected: true,
    })
  }

  // One listener for whichever kind is selected. The header reads the enable
  // flag plus a couple of kind-specific readouts; every per-field value lives in
  // the control sets, which publish their own ControlsStatus and are rendered by
  // Nepi_IF_Controls.
  instStatusListener(message) {
    const kind = this.state.kind
    const cfg = KIND_CONFIGS[kind]
    const next = { inst_enabled: message[cfg.enabledField] }
    if (kind === 'nmea') {
      next.nmea_connected = message.nmea_connected
      next.nmea_port      = message.nmea_port
    } else if (kind === 'hnav') {
      next.hnav_connected = message.hnav_connected
      next.hnav_port      = message.hnav_port
    } else if (kind === 'gps') {
      next.gps_mavros_connected    = message.mavros_connected
      next.gps_selected_mavros     = message.selected_mavros_node
      next.gps_current_latitude    = message.current_latitude
      next.gps_current_longitude   = message.current_longitude
      next.gps_current_altitude_m  = message.current_altitude_m
      next.gps_current_heading_deg = message.current_heading_deg
      next.gps_moving              = message.moving
    }
    this.setState(next)
  }

  updateMasterStatusListener(appNamespace) {
    if (this.state.masterStatusListener) {
      this.state.masterStatusListener.unsubscribe()
    }
    const masterStatusListener = this.props.ros.setupStatusListener(
      appNamespace + '/status',
      "nepi_app_nav_sim/NepiAppNavSimMasterStatus",
      this.masterStatusListener
    )
    this.setState({ masterStatusListener })
  }

  // Subscribes the given kind's selected instance and drops whatever was
  // subscribed before, so only one kind's status is on the wire at a time. The
  // kind dropdown, the instance dropdown and a rename all route through here.
  updateInstStatusListener(kind, instanceNs) {
    if (this.state.instStatusListener) {
      this.state.instStatusListener.unsubscribe()
    }
    const cfg = KIND_CONFIGS[kind]
    if (cfg === undefined || instanceNs === null) {
      this.setState({ instStatusListener: null })
      return
    }
    const instStatusListener = this.props.ros.setupStatusListener(
      instanceNs + '/status',
      cfg.statusType,
      this.instStatusListener
    )
    this.setState({ instStatusListener })
  }

  componentDidMount() {
    const namespace = this.getAppNamespace()
    if (namespace !== null) {
      this.setState({ appNamespace: namespace })
      this.updateMasterStatusListener(namespace)
      this.updateInstStatusListener(this.state.kind, this.getInstanceNamespace())
    }
  }

  componentDidUpdate(prevProps, prevState) {
    const namespace = this.getAppNamespace()
    const updated = this.state.appNamespace !== namespace && namespace !== null
    if (updated && namespace.indexOf('null') === -1) {
      this.setState({ appNamespace: namespace })
      this.updateMasterStatusListener(namespace)
      this.updateInstStatusListener(this.state.kind, this.getInstanceNamespace())
    }
    KIND_ORDER.forEach((k) => {
      const cfg = KIND_CONFIGS[k]
      if (prevState[cfg.selectedKey] !== this.state[cfg.selectedKey]) {
        this.setState({ [cfg.renameKey]: this.state[cfg.selectedKey] })
      }
    })
  }

  componentWillUnmount() {
    if (this.state.masterStatusListener) this.state.masterStatusListener.unsubscribe()
    if (this.state.instStatusListener)   this.state.instStatusListener.unsubscribe()
  }

  // Mirror of instanceControlsName() in the node: the instance's path below the
  // app namespace, flattened, plus the section suffix. The set hangs off the
  // APP namespace, not the instance namespace, because ControlsIF can only root
  // itself at the node. Both the panel mount and the Stop button go through
  // this, so the two can never drift onto different namespaces.
  getControlsNamespace(instanceNamespace, suffix) {
    const appNs = this.getAppNamespace()
    if (appNs == null || instanceNamespace == null || instanceNamespace === '') {
      return null
    }
    const tail = instanceNamespace.replace(appNs, '').replace(/^\/+|\/+$/g, '').replace(/\//g, '_')
    return appNs + '/' + tail + '_' + suffix
  }

  // One section's control set. The page draws the heading itself (divider plus
  // Label, as it always did), so this passes title={null} -- the same mount
  // shape fake_gps and Nepi_IF_Process use when the parent owns the heading.
  //
  // key={namespace} is load bearing: selecting a different instance changes the
  // namespace, and without a key React reuses the mounted component, which
  // keeps its old status subscription and leaves the previous instance's values
  // on screen. stereo_cam keys its per-process set for the same reason.
  // show_bounds={false} suppresses the read-only Min/Max block a bounded
  // control draws above its widget. Passed only for kinds that ask for it, and
  // undefined otherwise, which is what Nepi_IF_Control has always defaulted to.
  renderControlSection(instanceNamespace, suffix, disabled, hideBounds) {
    const namespace = this.getControlsNamespace(instanceNamespace, suffix)
    if (namespace == null) {
      return null
    }
    return (
      <NepiIFControls
        key={namespace}
        namespace={namespace}
        title={null}
        make_section={false}
        allways_show_controls={true}
        disabled={disabled}
        show_bounds={(hideBounds === true) ? false : undefined}
      />
    )
  }

  // Which simulator the page is showing. A view filter, not node state: all
  // three kinds keep running whichever one is on screen.
  renderKindSelector() {
    const { kind, connected } = this.state
    const selectStyle = {
      backgroundColor: '#333', color: '#ddd',
      border: '1px solid #555', borderRadius: 3,
      padding: '2px 6px', fontSize: 12,
    }
    return (
      <div style={{ display: 'flex', alignItems: 'center', gap: 6, marginBottom: 8 }}>
        <span style={{ fontSize: 11, color: '#aaa' }}>Simulator:</span>
        <select
          value={kind}
          onChange={(e) => {
            const newKind = e.target.value
            this.setState({ kind: newKind }, () => {
              this.updateInstStatusListener(newKind, this.getInstanceNamespace(newKind))
            })
          }}
          disabled={!connected}
          style={selectStyle}
        >
          {KIND_ORDER.map(k => (
            <option key={k} value={k}>{KIND_CONFIGS[k].label}</option>
          ))}
        </select>
      </div>
    )
  }

  renderInstanceSelector(cfg, dis) {
    const { sendStringMsg, sendUpdateStringMsg } = this.props.ros
    const names    = this.state[cfg.namesKey]
    const selected = this.state[cfg.selectedKey]
    const renameIn = this.state[cfg.renameKey]
    const selectStyle = {
      backgroundColor: '#333', color: '#ddd',
      border: '1px solid #555', borderRadius: 3,
      padding: '2px 6px', fontSize: 12,
    }
    return (
      <div style={{ display: 'flex', alignItems: 'center', gap: 6, marginBottom: 6 }}>
        <span style={{ fontSize: 11, color: '#aaa' }}>Instance:</span>
        <select
          value={selected}
          onChange={(e) => {
            const name = e.target.value
            this.setState({ [cfg.selectedKey]: name }, () => {
              this.updateInstStatusListener(cfg.key, this.getInstanceNamespace(cfg.key))
            })
          }}
          disabled={dis}
          style={selectStyle}
        >
          {names.map(n => <option key={n} value={n}>{n}</option>)}
        </select>
        <Button
          onClick={() => {
            const newName = cfg.key + '_' + names.length
            sendStringMsg(this.getAppNamespace() + '/' + cfg.addTopic, newName)
          }}
          disabled={dis}
          style={{ padding: '1px 8px', fontSize: 15, lineHeight: 1 }}
        >+</Button>
        <Button
          onClick={() => {
            sendStringMsg(this.getAppNamespace() + '/' + cfg.removeTopic, selected)
          }}
          disabled={dis || names.length <= 1}
          style={{ padding: '1px 8px', fontSize: 15, lineHeight: 1 }}
        >×</Button>
        <span style={{ fontSize: 11, color: '#aaa' }}>Name:</span>
        <Input
          id={cfg.renameInputId}
          value={renameIn}
          onChange={(e) => {
            const el = document.getElementById(cfg.renameInputId)
            setElementStyleModified(el)
            this.setState({ [cfg.renameKey]: e.target.value })
          }}
          onKeyDown={(e) => {
            if (e.key === 'Enter') {
              const newName = e.target.value.trim()
              if (newName && newName !== selected) {
                const el = document.getElementById(cfg.renameInputId)
                clearElementStyleModified(el)
                sendUpdateStringMsg(this.getAppNamespace() + '/' + cfg.renameTopic, selected, newName)
                this.setState({ [cfg.selectedKey]: newName }, () => {
                  this.updateInstStatusListener(cfg.key, this.getInstanceNamespace(cfg.key))
                })
              }
            }
          }}
          disabled={dis}
          style={{ width: 70, fontSize: 12 }}
        />
      </div>
    )
  }

  // Read-only simulation state for the GPS kind -- the half of the old fake_gps
  // panel the control sets do not replace: the node reports it, the operator
  // cannot set it.
  renderGpsState() {
    const { gps_mavros_connected, gps_selected_mavros, gps_current_latitude,
            gps_current_longitude, gps_current_altitude_m,
            gps_current_heading_deg, gps_moving } = this.state
    return (
      <React.Fragment>
        <Columns>
          <Column>
            <Label title={"Mavros Node"} />
            <span style={{ fontSize: 12, color: gps_mavros_connected ? '#00cc44' : '#999' }}>
              {gps_selected_mavros}
            </span>
          </Column>
          <Column>
            <Label title={"Moving"} />
            <span style={{ fontSize: 12, color: gps_moving ? '#ff9900' : '#999' }}>
              {gps_moving ? "yes" : "no"}
            </span>
          </Column>
        </Columns>
        <Columns>
          <Column>
            <Label title={"Latitude"} />
            <span style={{ fontSize: 12, color: '#ddd' }}>{Number(gps_current_latitude).toFixed(7)}</span>
          </Column>
          <Column>
            <Label title={"Longitude"} />
            <span style={{ fontSize: 12, color: '#ddd' }}>{Number(gps_current_longitude).toFixed(7)}</span>
          </Column>
        </Columns>
        <Columns>
          <Column>
            <Label title={"Altitude (m)"} />
            <span style={{ fontSize: 12, color: '#ddd' }}>{Number(gps_current_altitude_m).toFixed(1)}</span>
          </Column>
          <Column>
            <Label title={"Heading (deg)"} />
            <span style={{ fontSize: 12, color: '#ddd' }}>{Number(gps_current_heading_deg).toFixed(1)}</span>
          </Column>
        </Columns>
      </React.Fragment>
    )
  }

  // The single rendered panel, parameterised by the kind config. This replaced
  // renderNmeaSide and renderHnavSide, which were near-identical bodies
  // differing only in prefix, status fields and enable topic. The NMEA and HNav
  // output is unchanged apart from no longer sitting in a column.
  renderKindPanel(cfg) {
    const { connected, inst_enabled } = this.state
    const dis = !connected
    const ns  = this.getInstanceNamespace(cfg.key)
    const { sendBoolMsg, sendUpdateControlValue } = this.props.ros

    // The header readout and the state dot mean different things per kind: the
    // TCP kinds report a client connection on a port, the GPS kind reports
    // whether it is bound to a mavros node.
    let readout = null
    let linked  = false
    if (cfg.key === 'nmea') {
      readout = 'port ' + this.state.nmea_port
      linked  = this.state.nmea_connected
    } else if (cfg.key === 'hnav') {
      readout = 'port ' + this.state.hnav_port
      linked  = this.state.hnav_connected
    } else if (cfg.key === 'gps') {
      readout = this.state.gps_mavros_connected ? 'mavlink bound' : 'no mavlink target'
      linked  = this.state.gps_mavros_connected
    }
    const dotColor = linked ? '#00cc44' : (inst_enabled ? '#ff9900' : '#555')
    const dotTitle = linked
      ? (cfg.key === 'gps' ? "Injecting into a mavros node" : "Client connected")
      : (inst_enabled
          ? (cfg.key === 'gps' ? "Sim running, no mavlink target" : "Server running, no client")
          : "Sim disabled")
    const dotStyle = {
      display: 'inline-block', width: 10, height: 10,
      borderRadius: '50%', backgroundColor: dotColor, marginLeft: 8,
    }
    const divider = (
      <div style={{ borderTop: "1px solid #555",
                    marginTop: Styles.vars.spacing.small,
                    marginBottom: Styles.vars.spacing.xs }} />
    )

    // Sections split on the advanced flag rather than being sliced by index, so
    // the order a kind declares them in is the order they render in on both
    // sides of the toggle. A kind that declares no advanced section gets an
    // empty second list and no toggle at all.
    const main_sections     = cfg.sections.filter(s => s.advanced !== true)
    const advanced_sections = cfg.sections.filter(s => s.advanced === true)
    const show_advanced     = this.state.showAdvanced[cfg.key] === true

    const renderSection = (section) => (
      <React.Fragment key={section.suffix}>
        {divider}
        <Label title={section.title} />
        {this.renderControlSection(ns, section.suffix, dis, cfg.hideBounds)}
      </React.Fragment>
    )

    return (
      <div>
        {this.renderInstanceSelector(cfg, dis)}

        <Columns>
          <Column>
            <Label title={cfg.title} />
            <span style={{ fontSize: 11, color: '#aaa' }}>{readout}</span>
            <span style={dotStyle} title={dotTitle} />
          </Column>
          <Column>
            <AsyncToggle
              checked={inst_enabled}
              onClick={() => sendBoolMsg(ns + '/' + cfg.enableTopic, !inst_enabled)}
              disabled={dis}
            />
          </Column>
        </Columns>

        {cfg.key === 'gps' ? this.renderGpsState() : null}

        {main_sections.map(renderSection)}

        {/* Advanced Options. Rendered only for a kind that actually declares an
            advanced section, so the NMEA and HNav panels -- which declare none
            -- are byte-for-byte what they were. The hidden sections are not
            unmounted state: each is its own ControlsIF set that keeps running
            and keeps publishing whether or not it is drawn. */}
        {advanced_sections.length === 0 ? null : (
          <React.Fragment>
            {divider}
            <Columns>
              <Column>
                <Label title={"Advanced Options"}>
                  {/* react-toggle (not AsyncToggle): checked is local view state, already immediate -- no backend round trip to confirm. */}
                  <Toggle
                    checked={show_advanced}
                    onClick={() => this.setState({
                      showAdvanced: Object.assign({}, this.state.showAdvanced,
                                                  { [cfg.key]: !show_advanced })
                    })}
                  />
                </Label>
              </Column>
              <Column>
              </Column>
            </Columns>
          </React.Fragment>
        )}

        {show_advanced === true ? advanced_sections.map(renderSection) : null}

        {cfg.stopControl === null ? null : (
          <Columns>
            <Column>
              {/* Stop zeroes the speed control rather than its own topic: the
                  per-field set_* topics moved into the control sets, so this
                  publishes the same UpdateControl the speed box does. The GPS
                  kind has no Stop here because its Stop is a Button control
                  inside its own Move set. */}
              <Button
                onClick={() => sendUpdateControlValue(
                  this.getControlsNamespace(ns, cfg.stopSuffix) + '/update_control',
                  cfg.stopControl, 0.0)}
                disabled={dis}
              >Stop</Button>
            </Column>
          </Columns>
        )}
      </div>
    )
  }

  renderConfig() {
    return (
      <React.Fragment>
        <NepiIFConfig namespace={this.getAppNamespace()} title={"Nepi_IF_Config"} />
      </React.Fragment>
    )
  }

  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true
    const cfg = KIND_CONFIGS[this.state.kind]

    // key={this.state.kind} on the panel: switching kinds remounts it cleanly
    // rather than reusing the previous kind's subtree, the same reason each
    // NepiIFControls mount is keyed by its namespace.
    const content = (
      <React.Fragment>
        {this.renderKindSelector()}
        <div key={this.state.kind}>
          {this.renderKindPanel(cfg)}
        </div>
        {this.renderConfig()}
      </React.Fragment>
    )

    if (make_section === false) {
      return <Columns><Column>{content}</Column></Columns>
    }
    return <Section>{content}</Section>
  }
}

export default NepiAppNavSim
