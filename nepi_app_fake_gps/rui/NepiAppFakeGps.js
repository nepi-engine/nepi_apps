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
import Toggle from "react-toggle"
import Select, { Option } from "./Select"
import Styles from "./Styles"

import NepiIFConfig from "./Nepi_IF_Config"

// Factory start geopoint — used to seed the control inputs (matches the node)
const FACTORY_LAT = 46.6540828
const FACTORY_LON = -122.3187578
const FACTORY_ALT = 0.0

@inject("ros")
@observer

// Fake GPS Application page
class NepiAppFakeGps extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_fake_gps",
      appNamespace: null,

      // Status fields — mirror NepiAppFakeGpsStatus.msg
      enabled: false,
      available_mavros_nodes: [],
      selected_mavros_node: "None",
      mavros_connected: false,
      current_latitude: 0.0,
      current_longitude: 0.0,
      current_altitude_m: 0.0,
      current_heading_deg: 0.0,
      moving: false,
      satellites_visible: 0,
      gps_pub_rate_hz: 0.0,

      // Control input buffers (edited by the operator, committed on button press)
      homeLat: String(FACTORY_LAT),
      homeLon: String(FACTORY_LON),
      homeAlt: String(FACTORY_ALT),
      gotoLat: String(FACTORY_LAT),
      gotoLon: String(FACTORY_LON),
      gotoAlt: String(FACTORY_ALT),
      posX: "0.0",
      posY: "0.0",
      posZ: "0.0",
      rateHz: "50",

      statusListener: null,
      connected: false,
    }

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.onToggleEnabled = this.onToggleEnabled.bind(this)
    this.onSelectMavrosNode = this.onSelectMavrosNode.bind(this)
    this.onSetLocation = this.onSetLocation.bind(this)
    this.onUseCurrent = this.onUseCurrent.bind(this)
    this.onGotoLocation = this.onGotoLocation.bind(this)
    this.onGotoPosition = this.onGotoPosition.bind(this)
    this.onSetRate = this.onSetRate.bind(this)
    this.onGoStop = this.onGoStop.bind(this)
    this.renderNumInput = this.renderNumInput.bind(this)
    this.renderControls = this.renderControls.bind(this)
    this.renderConfig = this.renderConfig.bind(this)
  }

  getBaseNamespace() {
    const { namespacePrefix, deviceId } = this.props.ros
    if (namespacePrefix !== null && deviceId !== null) {
      return "/" + namespacePrefix + "/" + deviceId
    }
    return null
  }

  getAppNamespace() {
    const base = this.getBaseNamespace()
    if (base !== null) {
      return base + "/" + this.state.appName
    }
    return null
  }

  statusListener(message) {
    this.setState({
      enabled: message.enabled,
      available_mavros_nodes: message.available_mavros_nodes,
      selected_mavros_node: message.selected_mavros_node,
      mavros_connected: message.mavros_connected,
      current_latitude: message.current_latitude,
      current_longitude: message.current_longitude,
      current_altitude_m: message.current_altitude_m,
      current_heading_deg: message.current_heading_deg,
      moving: message.moving,
      satellites_visible: message.satellites_visible,
      gps_pub_rate_hz: message.gps_pub_rate_hz,
      connected: true,
    })
  }

  updateStatusListener(namespace) {
    const statusNamespace = namespace + '/status'
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    var statusListener = this.props.ros.setupStatusListener(
      statusNamespace,
      "nepi_app_fake_gps/NepiAppFakeGpsStatus",
      this.statusListener
    )
    this.setState({
      appNamespace: namespace,
      statusListener: statusListener,
    })
  }

  componentDidMount() {
    const namespace = this.getAppNamespace()
    if (namespace !== null) {
      this.updateStatusListener(namespace)
    }
  }

  componentDidUpdate(prevProps, prevState) {
    const namespace = this.getAppNamespace()
    const namespace_updated = (this.state.appNamespace !== namespace && namespace !== null)
    if (namespace_updated) {
      if (namespace.indexOf('null') === -1) {
        this.updateStatusListener(namespace)
      }
    }
  }

  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
  }

  onToggleEnabled() {
    const { sendBoolMsg } = this.props.ros
    sendBoolMsg(this.getAppNamespace() + '/enable', !this.state.enabled)
  }

  onSelectMavrosNode(event) {
    const { sendStringMsg } = this.props.ros
    const topic = this.getAppNamespace() + '/select_mavros_node'
    sendStringMsg(topic, event.target.value)
  }

  // Teleport the simulated GPS to a geopoint (works whether or not enabled)
  onSetLocation() {
    const { sendGeoPointMsg } = this.props.ros
    const ns = this.getAppNamespace()
    sendGeoPointMsg(ns + '/reset', this.state.homeLat, this.state.homeLon, this.state.homeAlt)
  }

  // Copy the live position into the Set/Goto Location input buffers
  onUseCurrent() {
    this.setState({
      homeLat: String(Number(this.state.current_latitude).toFixed(7)),
      homeLon: String(Number(this.state.current_longitude).toFixed(7)),
      homeAlt: String(Number(this.state.current_altitude_m).toFixed(2)),
      gotoLat: String(Number(this.state.current_latitude).toFixed(7)),
      gotoLon: String(Number(this.state.current_longitude).toFixed(7)),
      gotoAlt: String(Number(this.state.current_altitude_m).toFixed(2)),
    })
  }

  // Simulate a move to an absolute geopoint
  onGotoLocation() {
    const { sendGeoPointMsg } = this.props.ros
    const ns = this.getAppNamespace()
    sendGeoPointMsg(ns + '/goto_location', this.state.gotoLat, this.state.gotoLon, this.state.gotoAlt)
  }

  // Simulate a relative move in local ENU meters (East=x, North=y, Up=z)
  onGotoPosition() {
    const ns = this.getAppNamespace()
    const num = (v) => { const n = Number(v); return isNaN(n) ? 0.0 : n }
    this.props.ros.publishMessage({
      name: ns + '/goto_position',
      messageType: "geometry_msgs/Point",
      data: { x: num(this.state.posX), y: num(this.state.posY), z: num(this.state.posZ) },
      noPrefix: true,
    })
  }

  // Set the fake GPS publish rate (Hz); node clamps to 1-100
  onSetRate() {
    const { sendFloatMsg } = this.props.ros
    sendFloatMsg(this.getAppNamespace() + '/set_gps_pub_rate', this.state.rateHz)
  }

  onGoStop() {
    const { ros } = this.props
    ros.publishEmpty(this.getAppNamespace() + '/go_stop')
  }

  renderNumInput(key, label) {
    return (
      <div style={{ display: 'flex', alignItems: 'center', gap: 8, marginBottom: Styles.vars.spacing.xs }}>
        <div style={{ minWidth: 90, fontSize: 12, color: '#aaa' }}>{label}</div>
        <Input
          id={key}
          value={this.state[key]}
          onChange={(e) => this.setState({ [key]: e.target.value })}
          disabled={!this.state.connected}
          style={{ width: 140 }}
        />
      </div>
    )
  }

  renderControls() {
    const { connected, enabled, available_mavros_nodes, selected_mavros_node,
            mavros_connected, current_latitude, current_longitude,
            current_altitude_m, current_heading_deg, moving, satellites_visible,
            gps_pub_rate_hz } = this.state
    const options = ["None"].concat(available_mavros_nodes || [])
    const can_move = connected && enabled

    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <Columns>
          <Column>
            <Label title={"Enabled"} />
          </Column>
          <Column>
            <Toggle
              checked={enabled}
              onClick={this.onToggleEnabled}
              disabled={!connected}
            />
          </Column>
        </Columns>

        <Columns>
          <Column>
            <Label title={"Target Mavros Node"} />
          </Column>
          <Column>
            <Select
              onChange={this.onSelectMavrosNode}
              value={selected_mavros_node}
              disabled={!connected}
            >
              {options.map((opt) => (
                <Option key={opt} value={opt}>{opt}</Option>
              ))}
            </Select>
          </Column>
        </Columns>

        <div style={{ borderTop: "1px solid #555", marginTop: Styles.vars.spacing.small, marginBottom: Styles.vars.spacing.xs }} />

        <Columns>
          <Column>
            <Label title={"Mavros Connected"} />
            <span style={{ fontSize: 12, color: mavros_connected ? '#00cc44' : '#999' }}>
              {mavros_connected ? "yes" : "no"}
            </span>
          </Column>
          <Column>
            <Label title={"Moving"} />
            <span style={{ fontSize: 12, color: moving ? '#ff9900' : '#999' }}>
              {moving ? "yes" : "no"}
            </span>
          </Column>
        </Columns>

        <Columns>
          <Column>
            <Label title={"Latitude"} />
            <span style={{ fontSize: 12, color: '#ddd' }}>{Number(current_latitude).toFixed(7)}</span>
          </Column>
          <Column>
            <Label title={"Longitude"} />
            <span style={{ fontSize: 12, color: '#ddd' }}>{Number(current_longitude).toFixed(7)}</span>
          </Column>
        </Columns>

        <Columns>
          <Column>
            <Label title={"Altitude (m)"} />
            <span style={{ fontSize: 12, color: '#ddd' }}>{Number(current_altitude_m).toFixed(1)}</span>
          </Column>
          <Column>
            <Label title={"Satellites"} />
            <span style={{ fontSize: 12, color: '#ddd' }}>{satellites_visible}</span>
          </Column>
        </Columns>

        <Columns>
          <Column>
            <Label title={"Heading (deg)"} />
            <span style={{ fontSize: 12, color: '#ddd' }}>{Number(current_heading_deg).toFixed(1)}</span>
          </Column>
          <Column>
          </Column>
        </Columns>

        <Columns>
          <Column>
            <Button onClick={this.onUseCurrent} disabled={!connected}>
              Use Current Position
            </Button>
          </Column>
          <Column>
          </Column>
        </Columns>

        {/* Set (teleport) GPS home location */}
        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />
        <Label title={"Set GPS Location (teleport)"} />
        {this.renderNumInput('homeLat', 'Latitude')}
        {this.renderNumInput('homeLon', 'Longitude')}
        {this.renderNumInput('homeAlt', 'Altitude (m)')}
        <Button onClick={this.onSetLocation} disabled={!connected}>
          Set Location
        </Button>

        {/* Simulate flight to an absolute geopoint */}
        <div style={{ borderTop: "1px solid #555", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />
        <Label title={"Goto Location (simulated move)"} />
        {this.renderNumInput('gotoLat', 'Latitude')}
        {this.renderNumInput('gotoLon', 'Longitude')}
        {this.renderNumInput('gotoAlt', 'Altitude (m)')}
        <Button onClick={this.onGotoLocation} disabled={!can_move}>
          Goto Location
        </Button>

        {/* Simulate a relative move in local ENU meters */}
        <div style={{ borderTop: "1px solid #555", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />
        <Label title={"Goto Position (relative, meters)"} />
        {this.renderNumInput('posX', 'East (m)')}
        {this.renderNumInput('posY', 'North (m)')}
        {this.renderNumInput('posZ', 'Up (m)')}
        <Button onClick={this.onGotoPosition} disabled={!can_move}>
          Goto Position
        </Button>

        {/* GPS publish rate (Hz) */}
        <div style={{ borderTop: "1px solid #555", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />
        <Label title={"GPS Publish Rate (Hz)"} />
        <div style={{ fontSize: 12, color: '#999', marginBottom: Styles.vars.spacing.xs }}>
          {"current: " + Number(gps_pub_rate_hz).toFixed(1) + " Hz"}
        </div>
        {this.renderNumInput('rateHz', 'Rate (Hz)')}
        <Button onClick={this.onSetRate} disabled={!connected}>
          Set Rate
        </Button>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <Columns>
          <Column>
            <Button
              style={{}}
              onClick={this.onGoStop}
              disabled={!can_move}
            >
              Stop
            </Button>
          </Column>
        </Columns>

      </React.Fragment>
    )
  }

  renderConfig() {
    const appNamespace = this.getAppNamespace()
    return (
      <React.Fragment>
        <NepiIFConfig
          namespace={appNamespace}
          title={"Nepi_IF_Config"}
        />
      </React.Fragment>
    )
  }

  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true

    if (make_section === false) {
      return (
        <Columns>
          <Column>
            {this.renderControls()}
            {this.renderConfig()}
          </Column>
        </Columns>
      )
    } else {
      return (
        <Section>
          {this.renderControls()}
          {this.renderConfig()}
        </Section>
      )
    }
  }
}

export default NepiAppFakeGps
