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
import Styles from "./Styles"

import NepiIFConfig from "./Nepi_IF_Config"
import NepiIFControls from "./Nepi_IF_Controls"
import { setElementStyleModified, clearElementStyleModified } from "./Utilities"

// Controls names of the three per-instance control sets, matching the
// CONTROLS_NAME_* constants in nav_sim_app_node.py. ControlsIF roots each set
// at create_namespace(instance_namespace, controls_name), so a set lives one
// level BELOW the instance namespace and Nepi_IF_Controls subscribes to
// <instance_ns>/<name>/status.
const CONTROLS_NAME_POSITION      = "controls_position"
const CONTROLS_NAME_ORIENTATION   = "controls_orientation"
const CONTROLS_NAME_DEADRECKONING = "controls_dead_reckoning"

@inject("ros")
@observer

class NepiAppNavSim extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_nav_sim",
      appNamespace: null,

      // NMEA status fields (filled by nmeaStatusListener)
      nmea_sim_enabled: false,
      nmea_connected:   false,
      nmea_port:        50000,
      nmea_latitude:    47.6205,
      nmea_longitude:   -122.3493,
      nmea_altitude_m:  10.0,
      nmea_heading_deg: 0.0,
      nmea_speed_ms:    0.0,

      // HNav status fields (filled by hnavStatusListener)
      hnav_sim_enabled: false,
      hnav_connected:   false,
      hnav_port:        16718,
      hnav_latitude:    47.6205,
      hnav_longitude:   -122.3493,
      hnav_altitude_m:  10.0,
      hnav_depth_m:     0.0,
      hnav_heading_deg: 0.0,
      hnav_roll_deg:    0.0,
      hnav_pitch_deg:   0.0,
      hnav_speed_ms:    0.0,

      // Per-field values, move/sin/wave enables and their step, rate, amplitude,
      // period and spread buffers all moved into the per-instance control sets.
      // Nepi_IF_Controls owns that state now, read from each set's
      // ControlsStatus, so none of it is mirrored here any more.

      // Listeners
      masterStatusListener: null,
      nmeaStatusListener:   null,
      hnavStatusListener:   null,
      connected: false,

      // Instance management
      nmeaInstanceNames:    ['nmea_0'],
      selectedNmeaInstance: 'nmea_0',
      nmeaRenameInput:      'nmea_0',
      hnavInstanceNames:    ['hnav_0'],
      selectedHnavInstance: 'hnav_0',
      hnavRenameInput:      'hnav_0',
    }

    this.getAppNamespace          = this.getAppNamespace.bind(this)
    this.getNmeaInstanceNamespace = this.getNmeaInstanceNamespace.bind(this)
    this.getHnavInstanceNamespace = this.getHnavInstanceNamespace.bind(this)
    this.masterStatusListener     = this.masterStatusListener.bind(this)
    this.nmeaStatusListener       = this.nmeaStatusListener.bind(this)
    this.hnavStatusListener       = this.hnavStatusListener.bind(this)
    this.updateMasterStatusListener = this.updateMasterStatusListener.bind(this)
    this.updateNmeaStatusListener = this.updateNmeaStatusListener.bind(this)
    this.updateHnavStatusListener = this.updateHnavStatusListener.bind(this)
    this.renderNmeaSide           = this.renderNmeaSide.bind(this)
    this.renderHnavSide           = this.renderHnavSide.bind(this)
    this.renderConfig             = this.renderConfig.bind(this)
  }

  getAppNamespace() {
    const { namespacePrefix, deviceId } = this.props.ros
    if (namespacePrefix !== null && deviceId !== null) {
      return "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
    }
    return null
  }

  getNmeaInstanceNamespace() {
    const appNs = this.getAppNamespace()
    if (appNs === null) return null
    return appNs + '/nmea_instances/' + this.state.selectedNmeaInstance
  }

  getHnavInstanceNamespace() {
    const appNs = this.getAppNamespace()
    if (appNs === null) return null
    return appNs + '/hnav_instances/' + this.state.selectedHnavInstance
  }

  masterStatusListener(message) {
    const nmeaInstanceNames = Array.isArray(message.nmea_instance_names) && message.nmea_instance_names.length > 0
      ? message.nmea_instance_names : ['nmea_0']
    const hnavInstanceNames = Array.isArray(message.hnav_instance_names) && message.hnav_instance_names.length > 0
      ? message.hnav_instance_names : ['hnav_0']
    this.setState({ nmeaInstanceNames, hnavInstanceNames, connected: true })
  }

  // Only the instance header still reads this status: the enable toggle, the
  // port number and the connection dot. Every per-field value and the whole
  // dirty-buffer sync moved to the control sets, which publish their own
  // ControlsStatus and are rendered by Nepi_IF_Controls.
  nmeaStatusListener(message) {
    this.setState({
      nmea_sim_enabled: message.nmea_sim_enabled,
      nmea_connected:   message.nmea_connected,
      nmea_port:        message.nmea_port,
    })
  }

  hnavStatusListener(message) {
    this.setState({
      hnav_sim_enabled: message.hnav_sim_enabled,
      hnav_connected:   message.hnav_connected,
      hnav_port:        message.hnav_port,
    })
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

  updateNmeaStatusListener(nmeaInstanceNs) {
    if (this.state.nmeaStatusListener) {
      this.state.nmeaStatusListener.unsubscribe()
    }
    const nmeaStatusListener = this.props.ros.setupStatusListener(
      nmeaInstanceNs + '/status',
      "nepi_app_nav_sim/NepiAppNmeaSimStatus",
      this.nmeaStatusListener
    )
    this.setState({ nmeaStatusListener })
  }

  updateHnavStatusListener(hnavInstanceNs) {
    if (this.state.hnavStatusListener) {
      this.state.hnavStatusListener.unsubscribe()
    }
    const hnavStatusListener = this.props.ros.setupStatusListener(
      hnavInstanceNs + '/status',
      "nepi_app_nav_sim/NepiAppHNavSimStatus",
      this.hnavStatusListener
    )
    this.setState({ hnavStatusListener })
  }

  componentDidMount() {
    const namespace = this.getAppNamespace()
    if (namespace !== null) {
      this.setState({ appNamespace: namespace })
      this.updateMasterStatusListener(namespace)
      this.updateNmeaStatusListener(namespace + '/nmea_instances/' + this.state.selectedNmeaInstance)
      this.updateHnavStatusListener(namespace + '/hnav_instances/' + this.state.selectedHnavInstance)
    }
  }

  componentDidUpdate(prevProps, prevState) {
    const namespace = this.getAppNamespace()
    const updated = this.state.appNamespace !== namespace && namespace !== null
    if (updated && namespace.indexOf('null') === -1) {
      this.setState({ appNamespace: namespace })
      this.updateMasterStatusListener(namespace)
      this.updateNmeaStatusListener(namespace + '/nmea_instances/' + this.state.selectedNmeaInstance)
      this.updateHnavStatusListener(namespace + '/hnav_instances/' + this.state.selectedHnavInstance)
    }
    if (prevState.selectedNmeaInstance !== this.state.selectedNmeaInstance) {
      this.setState({ nmeaRenameInput: this.state.selectedNmeaInstance })
    }
    if (prevState.selectedHnavInstance !== this.state.selectedHnavInstance) {
      this.setState({ hnavRenameInput: this.state.selectedHnavInstance })
    }
  }

  componentWillUnmount() {
    if (this.state.masterStatusListener) this.state.masterStatusListener.unsubscribe()
    if (this.state.nmeaStatusListener)   this.state.nmeaStatusListener.unsubscribe()
    if (this.state.hnavStatusListener)   this.state.hnavStatusListener.unsubscribe()
  }

  // instanceNs: the per-instance namespace to route all set_* topics through

  // One section's control set. The page draws the heading itself (divider plus
  // Label, as it always did), so this passes title={null} -- the same mount
  // shape fake_gps and Nepi_IF_Process use when the parent owns the heading.
  //
  // key={namespace} is load bearing: selecting a different instance changes the
  // namespace, and without a key React reuses the mounted component, which
  // keeps its old status subscription and leaves the previous instance's values
  // on screen. stereo_cam keys its per-process set for the same reason.
  renderControlSection(instanceNamespace, controlsName, disabled) {
    if (instanceNamespace == null || instanceNamespace === '') {
      return null
    }
    const namespace = instanceNamespace + '/' + controlsName
    return (
      <NepiIFControls
        key={namespace}
        namespace={namespace}
        title={null}
        make_section={false}
        allways_show_controls={true}
        disabled={disabled}
      />
    )
  }

  renderNmeaSide() {
    const { connected, nmea_sim_enabled, nmea_connected, nmea_port,
            nmeaInstanceNames, selectedNmeaInstance, nmeaRenameInput } = this.state
    const dis = !connected
    const ns  = this.getNmeaInstanceNamespace()
    const { sendBoolMsg, sendStringMsg, sendUpdateStringMsg,
            sendUpdateControlValue } = this.props.ros

    const dotColor = nmea_connected ? '#00cc44' : (nmea_sim_enabled ? '#ff9900' : '#555')
    const dotTitle = nmea_connected ? "Client connected"
                   : (nmea_sim_enabled ? "Server running, no client" : "Sim disabled")
    const dotStyle = {
      display: 'inline-block', width: 10, height: 10,
      borderRadius: '50%', backgroundColor: dotColor, marginLeft: 8,
    }
    const selectStyle = {
      backgroundColor: '#333', color: '#ddd',
      border: '1px solid #555', borderRadius: 3,
      padding: '2px 6px', fontSize: 12,
    }
    const divider = (
      <div style={{ borderTop: "1px solid #555",
                    marginTop: Styles.vars.spacing.small,
                    marginBottom: Styles.vars.spacing.xs }} />
    )

    return (
      <div>
        {/* Instance selector */}
        <div style={{ display: 'flex', alignItems: 'center', gap: 6, marginBottom: 6 }}>
          <span style={{ fontSize: 11, color: '#aaa' }}>Instance:</span>
          <select
            value={selectedNmeaInstance}
            onChange={(e) => {
              const name = e.target.value
              this.setState({ selectedNmeaInstance: name }, () => {
                this.updateNmeaStatusListener(this.getNmeaInstanceNamespace())
              })
            }}
            disabled={dis}
            style={selectStyle}
          >
            {nmeaInstanceNames.map(n => <option key={n} value={n}>{n}</option>)}
          </select>
          <Button
            onClick={() => {
              const newName = 'nmea_' + nmeaInstanceNames.length
              sendStringMsg(this.getAppNamespace() + '/add_nmea_instance', newName)
            }}
            disabled={dis}
            style={{ padding: '1px 8px', fontSize: 15, lineHeight: 1 }}
          >+</Button>
          <Button
            onClick={() => {
              sendStringMsg(this.getAppNamespace() + '/remove_nmea_instance', selectedNmeaInstance)
            }}
            disabled={dis || nmeaInstanceNames.length <= 1}
            style={{ padding: '1px 8px', fontSize: 15, lineHeight: 1 }}
          >×</Button>
          <span style={{ fontSize: 11, color: '#aaa' }}>Name:</span>
          <Input
            id={'nmeaRenameInput'}
            value={nmeaRenameInput}
            onChange={(e) => {
              const el = document.getElementById('nmeaRenameInput')
              setElementStyleModified(el)
              this.setState({ nmeaRenameInput: e.target.value })
            }}
            onKeyDown={(e) => {
              if (e.key === 'Enter') {
                const newName = e.target.value.trim()
                if (newName && newName !== selectedNmeaInstance) {
                  const el = document.getElementById('nmeaRenameInput')
                  clearElementStyleModified(el)
                  sendUpdateStringMsg(this.getAppNamespace() + '/rename_nmea_instance', selectedNmeaInstance, newName)
                  this.setState({ selectedNmeaInstance: newName }, () => {
                    this.updateNmeaStatusListener(this.getNmeaInstanceNamespace())
                  })
                }
              }
            }}
            disabled={dis}
            style={{ width: 70, fontSize: 12 }}
          />
        </div>

        <Columns>
          <Column>
            <Label title={"NMEA Sim"} />
            <span style={{ fontSize: 11, color: '#aaa' }}>port {nmea_port}</span>
            <span style={dotStyle} title={dotTitle} />
          </Column>
          <Column>
            <AsyncToggle
              checked={nmea_sim_enabled}
              onClick={() => sendBoolMsg(ns + '/set_nmea_enabled', !nmea_sim_enabled)}
              disabled={dis}
            />
          </Column>
        </Columns>

        {divider}
        <Label title={"Position"} />
        {this.renderControlSection(ns, CONTROLS_NAME_POSITION, dis)}

        {divider}
        <Label title={"Orientation"} />
        {this.renderControlSection(ns, CONTROLS_NAME_ORIENTATION, dis)}

        {divider}
        <Label title={"Dead-Reckoning"} />
        {this.renderControlSection(ns, CONTROLS_NAME_DEADRECKONING, dis)}
        <Columns>
          <Column>
            {/* Stop zeroes the speed control rather than its own topic: the
                per-field set_* topics moved into the control sets, so this
                publishes the same UpdateControl the speed box does. */}
            <Button
              onClick={() => sendUpdateControlValue(
                ns + '/' + CONTROLS_NAME_DEADRECKONING + '/update_control',
                'nmea_speed_ms', 0.0)}
              disabled={dis}
            >Stop</Button>
          </Column>
        </Columns>
      </div>
    )
  }

  renderHnavSide() {
    const { connected, hnav_sim_enabled, hnav_connected, hnav_port,
            hnavInstanceNames, selectedHnavInstance, hnavRenameInput } = this.state
    const dis = !connected
    const ns  = this.getHnavInstanceNamespace()
    const { sendBoolMsg, sendStringMsg, sendUpdateStringMsg,
            sendUpdateControlValue } = this.props.ros

    const dotColor = hnav_connected ? '#00cc44' : (hnav_sim_enabled ? '#ff9900' : '#555')
    const dotTitle = hnav_connected ? "Client connected"
                   : (hnav_sim_enabled ? "Server running, no client" : "Sim disabled")
    const dotStyle = {
      display: 'inline-block', width: 10, height: 10,
      borderRadius: '50%', backgroundColor: dotColor, marginLeft: 8,
    }
    const selectStyle = {
      backgroundColor: '#333', color: '#ddd',
      border: '1px solid #555', borderRadius: 3,
      padding: '2px 6px', fontSize: 12,
    }
    const divider = (
      <div style={{ borderTop: "1px solid #555",
                    marginTop: Styles.vars.spacing.small,
                    marginBottom: Styles.vars.spacing.xs }} />
    )

    return (
      <div>
        {/* Instance selector */}
        <div style={{ display: 'flex', alignItems: 'center', gap: 6, marginBottom: 6 }}>
          <span style={{ fontSize: 11, color: '#aaa' }}>Instance:</span>
          <select
            value={selectedHnavInstance}
            onChange={(e) => {
              const name = e.target.value
              this.setState({ selectedHnavInstance: name }, () => {
                this.updateHnavStatusListener(this.getHnavInstanceNamespace())
              })
            }}
            disabled={dis}
            style={selectStyle}
          >
            {hnavInstanceNames.map(n => <option key={n} value={n}>{n}</option>)}
          </select>
          <Button
            onClick={() => {
              const newName = 'hnav_' + hnavInstanceNames.length
              sendStringMsg(this.getAppNamespace() + '/add_hnav_instance', newName)
            }}
            disabled={dis}
            style={{ padding: '1px 8px', fontSize: 15, lineHeight: 1 }}
          >+</Button>
          <Button
            onClick={() => {
              sendStringMsg(this.getAppNamespace() + '/remove_hnav_instance', selectedHnavInstance)
            }}
            disabled={dis || hnavInstanceNames.length <= 1}
            style={{ padding: '1px 8px', fontSize: 15, lineHeight: 1 }}
          >×</Button>
          <span style={{ fontSize: 11, color: '#aaa' }}>Name:</span>
          <Input
            id={'hnavRenameInput'}
            value={hnavRenameInput}
            onChange={(e) => {
              const el = document.getElementById('hnavRenameInput')
              setElementStyleModified(el)
              this.setState({ hnavRenameInput: e.target.value })
            }}
            onKeyDown={(e) => {
              if (e.key === 'Enter') {
                const newName = e.target.value.trim()
                if (newName && newName !== selectedHnavInstance) {
                  const el = document.getElementById('hnavRenameInput')
                  clearElementStyleModified(el)
                  sendUpdateStringMsg(this.getAppNamespace() + '/rename_hnav_instance', selectedHnavInstance, newName)
                  this.setState({ selectedHnavInstance: newName }, () => {
                    this.updateHnavStatusListener(this.getHnavInstanceNamespace())
                  })
                }
              }
            }}
            disabled={dis}
            style={{ width: 70, fontSize: 12 }}
          />
        </div>

        <Columns>
          <Column>
            <Label title={"HNav Sim"} />
            <span style={{ fontSize: 11, color: '#aaa' }}>port {hnav_port}</span>
            <span style={dotStyle} title={dotTitle} />
          </Column>
          <Column>
            <AsyncToggle
              checked={hnav_sim_enabled}
              onClick={() => sendBoolMsg(ns + '/set_hnav_enabled', !hnav_sim_enabled)}
              disabled={dis}
            />
          </Column>
        </Columns>

        {divider}
        <Label title={"Position"} />
        {this.renderControlSection(ns, CONTROLS_NAME_POSITION, dis)}

        {divider}
        <Label title={"Orientation"} />
        {this.renderControlSection(ns, CONTROLS_NAME_ORIENTATION, dis)}

        {divider}
        <Label title={"Dead-Reckoning"} />
        {this.renderControlSection(ns, CONTROLS_NAME_DEADRECKONING, dis)}
        <Columns>
          <Column>
            {/* Same as the NMEA side: Stop zeroes the speed control, not a
                topic of its own. */}
            <Button
              onClick={() => sendUpdateControlValue(
                ns + '/' + CONTROLS_NAME_DEADRECKONING + '/update_control',
                'hnav_speed_ms', 0.0)}
              disabled={dis}
            >Stop</Button>
          </Column>
        </Columns>
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

    const content = (
      <React.Fragment>
        <div style={{ display: 'flex', gap: 0 }}>
          <div style={{ flex: 1, paddingRight: 16, borderRight: '1px solid #444' }}>
            {this.renderNmeaSide()}
          </div>
          <div style={{ flex: 1, paddingLeft: 16 }}>
            {this.renderHnavSide()}
          </div>
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
