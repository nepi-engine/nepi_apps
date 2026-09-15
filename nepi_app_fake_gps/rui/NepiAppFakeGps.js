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
import Styles from "./Styles"

import NepiIFControls from "./Nepi_IF_Controls"
import NepiIFConfig from "./Nepi_IF_Config"

// Leaf of the app's ControlsIF namespace. ControlsIF roots itself at
// create_namespace(node_namespace, controls_name), so the control set sits one
// level BELOW the app node namespace. Appending '/controls' here is what makes
// Nepi_IF_Controls subscribe to .../app_fake_gps/controls/status and publish
// its updates to .../app_fake_gps/controls/update_control.
const CONTROLS_NAME = "controls"

@inject("ros")
@observer

// Fake GPS Application page
class NepiAppFakeGps extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_fake_gps",
      appNamespace: null,

      // Status fields -- mirror NepiAppFakeGpsStatus.msg. Everything the
      // operator ADJUSTS now comes from the control set below; what is left
      // here is what the node REPORTS.
      mavros_connected: false,
      current_latitude: 0.0,
      current_longitude: 0.0,
      current_altitude_m: 0.0,
      current_heading_deg: 0.0,
      moving: false,

      statusListener: null,
      connected: false,
    }

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getControlsNamespace = this.getControlsNamespace.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.renderState = this.renderState.bind(this)
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

  getControlsNamespace() {
    const appNamespace = this.getAppNamespace()
    if (appNamespace !== null) {
      return appNamespace + "/" + CONTROLS_NAME
    }
    return null
  }

  statusListener(message) {
    this.setState({
      mavros_connected: message.mavros_connected,
      current_latitude: message.current_latitude,
      current_longitude: message.current_longitude,
      current_altitude_m: message.current_altitude_m,
      current_heading_deg: message.current_heading_deg,
      moving: message.moving,
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

  // Read-only simulation state. This is the half of the old panel the control
  // set does not replace: the node reports it, the operator cannot set it.
  renderState() {
    const { mavros_connected, current_latitude, current_longitude,
            current_altitude_m, current_heading_deg, moving } = this.state

    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />
        <Label title={"Simulated GPS State"} />

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
            <Label title={"Heading (deg)"} />
            <span style={{ fontSize: 12, color: '#ddd' }}>{Number(current_heading_deg).toFixed(1)}</span>
          </Column>
        </Columns>

      </React.Fragment>
    )
  }

  // Every operator adjustment the app has, rendered by the shared control
  // renderer from the node's ControlsStatus. Mounted the way the in-workspace
  // examples mount it -- a divider and a Label, then the set inlined with
  // make_section={false}. title is passed as null (the Nepi_IF_Process
  // treatment) because the Label above is already this block's heading, and
  // the component's own default title would print a second one under it.
  // allways_show_controls keeps the set open: these controls ARE the page, so
  // there is nothing left to see if they are collapsed behind a toggle.
  renderControls() {
    const controlsNamespace = this.getControlsNamespace()
    if (controlsNamespace === null || controlsNamespace.indexOf('null') !== -1) {
      return null
    }
    return (
      <React.Fragment>
        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />
        <Label title={"Fake GPS Controls"} />
        <NepiIFControls
          namespace={controlsNamespace}
          title={null}
          make_section={false}
          allways_show_controls={true}
        />
      </React.Fragment>
    )
  }

  // Config stays on the APP namespace, not the controls namespace: a save there
  // dumps the whole node subtree, which includes <app>/controls, so one box
  // still persists and resets the control values.
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
            {this.renderState()}
            {this.renderControls()}
            {this.renderConfig()}
          </Column>
        </Columns>
      )
    } else {
      return (
        <Section>
          {this.renderState()}
          {this.renderControls()}
          {this.renderConfig()}
        </Section>
      )
    }
  }
}

export default NepiAppFakeGps
