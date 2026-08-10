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

import { Columns, Column } from "./Columns"
import Select, { Option } from "./Select"
import Button, { ButtonMenu } from "./Button"
import Label from "./Label"
import Input from "./Input"
import AsyncToggle from "./AsyncToggle"
import Styles from "./Styles"
import BooleanIndicator from "./BooleanIndicator"


import NepiIFImageViewer from "./Nepi_IF_ImageViewer"
import NepiIFConfig from "./Nepi_IF_Config"

import { onUpdateSetStateValue, onEnterSendFloatValue} from "./Utilities"

// Static pose fields, in RUI display order: [status field / state key, label,
// set topic]. The status field name and the state key are the same string, so
// one table drives both the refresh in statusListener() and the input rows in
// renderNavPoseControls().
const NAVPOSE_STATIC_VALUE_FIELDS = [
  ['navpose_static_latitude', 'Latitude (deg)', 'set_navpose_static_latitude'],
  ['navpose_static_longitude', 'Longitude (deg)', 'set_navpose_static_longitude'],
  ['navpose_static_altitude_m', 'Altitude (m)', 'set_navpose_static_altitude'],
  ['navpose_static_depth_m', 'Depth (m)', 'set_navpose_static_depth'],
  ['navpose_static_heading_deg', 'Heading (deg)', 'set_navpose_static_heading'],
  ['navpose_static_roll_deg', 'Roll (deg)', 'set_navpose_static_roll'],
  ['navpose_static_pitch_deg', 'Pitch (deg)', 'set_navpose_static_pitch'],
  ['navpose_static_yaw_deg', 'Yaw (deg)', 'set_navpose_static_yaw'],
  ['navpose_static_x_m', 'Position X (m)', 'set_navpose_static_x'],
  ['navpose_static_y_m', 'Position Y (m)', 'set_navpose_static_y'],
  ['navpose_static_z_m', 'Position Z (m)', 'set_navpose_static_z']
]


@inject("ros")
@observer

class FilePubDepthmapApp extends Component {
  constructor(props) {
    super(props)

    this.state = {

      appName: 'app_file_pub_depthmap',
	    appNamespace: null,
      status_msg: null,

      color_image_text: 'file_pub_depthmap/color_image',
      depth_map_image_text: 'file_pub_depthmap/depth_map/depth_map_image',

      viewableFolders: false,

      home_folder: 'None',
      current_folder: null,
      selected_folder: 'Home',
      current_folders: [],
      supported_file_types: [],
      collection_count: 0,
      current_collection: 'None',

      paused: false,

      set_random: false,
      set_overlay: false,
      min_max_rate: [0.1,20],
      set_rate: 1,
      pub_running: false,

      // NavPose source. navpose_active_mode is the RESOLVED source, and it is
      // what the static fields below are gated on -- a navpose_source_mode of
      // 'auto' says nothing on its own about which source is publishing.
      navpose_source_mode: 'auto',
      navpose_source_mode_options: [],
      navpose_active_mode: 'static',
      navpose_system_available: false,
      navpose_static_frame_nav: '',
      navpose_static_frame_altitude: '',
      navpose_static_frame_depth: '',
      navpose_frame_nav_options: [],
      navpose_frame_altitude_options: [],
      navpose_frame_depth_options: [],

      // Static pose values. Held in local state so an operator can type into a
      // field without the next status message overwriting it mid-edit; the
      // status listener only refreshes them when the node reports a change.
      navpose_static_latitude: 0,
      navpose_static_longitude: 0,
      navpose_static_heading_deg: 0,
      navpose_static_roll_deg: 0,
      navpose_static_pitch_deg: 0,
      navpose_static_yaw_deg: 0,
      navpose_static_x_m: 0,
      navpose_static_y_m: 0,
      navpose_static_z_m: 0,
      navpose_static_altitude_m: 0,
      navpose_static_depth_m: 0,

      statusListener: null,
      connected: false,
      needs_update: true

    }

    this.createFolderOptions = this.createFolderOptions.bind(this)
    this.onChangeFolderSelection = this.onChangeFolderSelection.bind(this)
    this.toggleViewableFolders = this.toggleViewableFolders.bind(this)
    this.createOptions = this.createOptions.bind(this)
    this.renderNavPoseStaticValue = this.renderNavPoseStaticValue.bind(this)
    this.renderNavPoseControls = this.renderNavPoseControls.bind(this)

    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)


  }


  getAppNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var appNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      appNamespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
    }
    return appNamespace
  }

  // Callback for handling ROS Status messages
  statusListener(message) {
    this.setState({
      status_msg: message,
      home_folder: message.home_folder ,
      current_folders: message.current_folders ,
      selected_folder: message.selected_folder,
      supported_file_types: message.supported_file_types,
      collection_count: message.collection_count ,
      current_collection: message.current_collection ,
      paused: message.paused ,

      set_random: message.set_random ,
      set_overlay: message.set_overlay ,
      min_max_rate: message.min_max_rate ,

      pub_running: message.running,

      navpose_source_mode: message.navpose_source_mode ,
      navpose_source_mode_options: message.navpose_source_mode_options ,
      navpose_active_mode: message.navpose_active_mode ,
      navpose_system_available: message.navpose_system_available ,
      navpose_static_frame_nav: message.navpose_static_frame_nav ,
      navpose_static_frame_altitude: message.navpose_static_frame_altitude ,
      navpose_static_frame_depth: message.navpose_static_frame_depth ,
      navpose_frame_nav_options: message.navpose_frame_nav_options ,
      navpose_frame_altitude_options: message.navpose_frame_altitude_options ,
      navpose_frame_depth_options: message.navpose_frame_depth_options

  })

  // Same guard the set_rate field below uses: a static pose input is refreshed
  // from the status message only when the NODE's value changed, so a status tick
  // cannot overwrite what the operator is part-way through typing.
  const prev_msg = this.state.status_msg
  const value_fields = NAVPOSE_STATIC_VALUE_FIELDS
  for (var vi = 0; vi < value_fields.length; vi++) {
    const field = value_fields[vi][0]
    const value_changed = (prev_msg != null) ? (prev_msg[field] !== message[field]) : true
    if (value_changed === true) {
      this.setState({[field]: message[field]})
    }
  }

  var current_folder = 'None'
  if (message.current_folder === message.home_folder ){
    current_folder = 'Home'
  }
  else {
    current_folder = message.current_folder
  }

  this.setState({
      current_folder: current_folder,
      connected: true
    })

  const needs_update = (this.state.status_msg != null) ? (this.state.status_msg.set_rate !== message.set_rate) : false

  if (needs_update === true){
  this.setState({
      set_rate: message.set_rate
    })

  }

  }

    // Function for configuring and subscribing to Status
    updateStatusListener() {
      const namespace = this.getAppNamespace()
      const statusNamespace = namespace + '/status'
      if (this.state.statusListener) {
        this.state.statusListener.unsubscribe()
      }
      var statusListener = this.props.ros.setupStatusListener(
            statusNamespace,
            "nepi_app_file_pub_depthmap/FilePubDepthmapStatus",
            this.statusListener
          )
      this.setState({
        statusListener: statusListener,
      })
    }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getAppNamespace()
    const namespace_updated = (prevState.appNamespace !== namespace && namespace !== null)
    if (namespace_updated) {
      if (namespace.indexOf('null') === -1){
        this.setState({appNamespace: namespace})
        this.updateStatusListener()
      }
    }
  }


  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
  }



  renderPubControls() {
    const {sendBoolMsg} = this.props.ros
    const appNamespace = this.state.appNamespace
    const pubRunning = this.state.pub_running

    return (


    <Columns>
    <Column>


        <div hidden={!this.state.connected}>


            <Label title={"Collection Count"}>
            <Input disabled value={this.state.collection_count} />
            </Label>


          <Label title={"Publishing"}>
              <BooleanIndicator value={pubRunning} />
            </Label>

              <div hidden={pubRunning}>
            <ButtonMenu>
              <Button
                disabled={pubRunning}
                onClick={() => this.props.ros.sendTriggerMsg(appNamespace + "/start_pub")}>{"Start Publishing"}</Button>
            </ButtonMenu>
            </div>

            <div hidden={!pubRunning}>
            <ButtonMenu>
              <Button onClick={() => this.props.ros.sendTriggerMsg(appNamespace + "/stop_pub")}>{"Stop Publishing"}</Button>
            </ButtonMenu>
            </div>

            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

            <Columns>
                  <Column>


                      <Label title="Pause">
                            <AsyncToggle
                            checked={this.state.paused===true}
                            onClick={() => sendBoolMsg(appNamespace + "/pause_pub",!this.state.paused)}>
                            </AsyncToggle>
                      </Label>

                </Column>
                  <Column>


                      <div hidden={this.state.paused === true}>

                            <Label title={"Set Rate (Hz)"}>
                              <Input id="set_rate"
                                value={this.state.set_rate}
                                onChange={(event) => onUpdateSetStateValue.bind(this)(event,"set_rate")}
                                onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event,appNamespace + "/set_rate")} />
                            </Label>


                            <Label title="Set Random Order">
                                  <AsyncToggle
                                  checked={this.state.set_random===true}
                                  onClick={() => sendBoolMsg(appNamespace + "/set_random",!this.state.set_random)}>
                                  </AsyncToggle>
                            </Label>

                      </div>


                      <div hidden={this.state.paused === false}>

                                  <ButtonMenu>
                                  <Button onClick={() => this.props.ros.sendTriggerMsg(appNamespace + "/step_forward")}>{"Forward"}</Button>
                                </ButtonMenu>

                                <ButtonMenu>
                                  <Button onClick={() => this.props.ros.sendTriggerMsg(appNamespace + "/step_backward")}>{"Back"}</Button>
                                </ButtonMenu>



                        </div>


            </Column>
            </Columns>


            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>




          <Label title={"Current Collection"} >
          </Label>
          <pre style={{ height: "25px", overflowY: "auto" }}>
            {this.state.current_collection}
          </pre>


        <Label title="Overlay Filename">
              <AsyncToggle
              checked={this.state.set_overlay===true}
              onClick={() => sendBoolMsg(appNamespace + "/set_overlay",!this.state.set_overlay)}>
              </AsyncToggle>
        </Label>


            </div>


            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

              <NepiIFConfig
                              namespace={appNamespace}
                              title={"Nepi_IF_Conig"}
              />


        </Column>
        </Columns>




    )
  }




  // Option list from a status message string array.
  createOptions(options) {
    var items = []
    if (options) {
      for (var i = 0; i < options.length; i++) {
        items.push(<Option key={options[i]} value={options[i]}>{options[i]}</Option>)
      }
    }
    return items
  }

  // One static pose value row. Disabled while a system pose is being forwarded,
  // because the value would not reach the wire.
  renderNavPoseStaticValue(field, title, topic, disabled) {
    const appNamespace = this.state.appNamespace
    return (
      <Label key={field} title={title}>
        <Input id={field}
          disabled={disabled}
          value={this.state[field]}
          onChange={(event) => onUpdateSetStateValue.bind(this)(event,field)}
          onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event,appNamespace + "/" + topic)} />
      </Label>
    )
  }

  // NavPose source controls.
  //
  // Rendered OUTSIDE the collection_count gate that wraps renderPubControls():
  // the node publishes <app>/navpose on its own steady timer whether or not a
  // collection is being published, so the controls for it must be reachable
  // whether or not a folder with collections is selected.
  //
  // The static pose fields and the three frame dropdowns are disabled whenever
  // navpose_active_mode is 'system'. That is not cosmetic -- in system mode the
  // node forwards navpose_mgr's pose with the frames its AUTHOR set, and never
  // reads these values. Gating on navpose_active_mode rather than
  // navpose_source_mode is what makes 'auto' render correctly: 'auto' resolves to
  // one source or the other, and it is the resolved one that decides whether
  // these fields do anything.
  renderNavPoseControls() {
    const {sendStringMsg} = this.props.ros
    const appNamespace = this.state.appNamespace
    const active_mode = this.state.navpose_active_mode
    const forwarding = (active_mode === 'system')
    const value_fields = NAVPOSE_STATIC_VALUE_FIELDS

    return (

      <div hidden={!this.state.connected}>

        <Label title={"NavPose Source"} />

        <Label title={"Source Mode"}>
          <Select
            onChange={(event) => sendStringMsg(appNamespace + "/set_navpose_source_mode", event.target.value)}
            value={this.state.navpose_source_mode}
          >
            {this.createOptions(this.state.navpose_source_mode_options)}
          </Select>
        </Label>

        <Label title={"Publishing Source"}>
          <Input disabled value={active_mode} />
        </Label>

        <Label title={"System NavPose Available"}>
          <BooleanIndicator value={this.state.navpose_system_available===true} />
        </Label>

        <Label title={forwarding
          ? "Forwarding the system NavPose. Its frames are set by its source and are not changed here."
          : "Publishing a static NavPose authored below."} />

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <Label title={"Static Pose"} />

        {value_fields.map((value_field) =>
          this.renderNavPoseStaticValue(value_field[0], value_field[1], value_field[2], forwarding)
        )}

        <Label title={"Static Pose Frames"} />

        <Label title={"Nav Frame"}>
          <Select
            disabled={forwarding}
            onChange={(event) => sendStringMsg(appNamespace + "/set_navpose_static_frame_nav", event.target.value)}
            value={this.state.navpose_static_frame_nav}
          >
            {this.createOptions(this.state.navpose_frame_nav_options)}
          </Select>
        </Label>

        <Label title={"Altitude Frame"}>
          <Select
            disabled={forwarding}
            onChange={(event) => sendStringMsg(appNamespace + "/set_navpose_static_frame_altitude", event.target.value)}
            value={this.state.navpose_static_frame_altitude}
          >
            {this.createOptions(this.state.navpose_frame_altitude_options)}
          </Select>
        </Label>

        <Label title={"Depth Frame"}>
          <Select
            disabled={forwarding}
            onChange={(event) => sendStringMsg(appNamespace + "/set_navpose_static_frame_depth", event.target.value)}
            value={this.state.navpose_static_frame_depth}
          >
            {this.createOptions(this.state.navpose_frame_depth_options)}
          </Select>
        </Label>

      </div>

    )
  }


  // Function for creating image topic options.
  createFolderOptions() {
    const cur_folder = this.state.current_folder
    const sel_folder = this.state.selected_folder
    var items = []
    if (cur_folder){
      items.push(<Option value={"Home"}>{"Home"}</Option>)
      if (sel_folder !== 'Home'){
        items.push(<Option value={"Back"}>{"Back"}</Option>)
      }
      const folders = this.state.current_folders
      for (var i = 0; i < folders.length; i++) {
        items.push(<Option value={folders[i]}>{folders[i]}</Option>)
      }
    }
    return items
  }

  onChangeFolderSelection(event) {
    const {sendTriggerMsg, sendStringMsg} = this.props.ros
    const namespace = this.state.appNamespace
    const setNamespace = namespace + "/select_folder"
    const homeNamespace = namespace + "/home_folder"
    const backNamespace = namespace + "/back_folder"
    const value = event.target.value
    if (namespace !== null){
      if (value === 'Home') {
        sendTriggerMsg(homeNamespace)
      }
      else if (value === 'Back') {
        sendTriggerMsg(backNamespace)
      }
      else {
        sendStringMsg(setNamespace,value)
      }
    }
    this.setState({selected_folder: value})
  }



  toggleViewableFolders() {
    const viewable = !this.state.viewableFolders
    this.setState({viewableFolders: viewable})
  }


 render() {
    if (this.state.needs_update === true){
      this.setState({needs_update: false})
    }
    const folderOptions = this.createFolderOptions()
    const pubRunning = this.state.pub_running
    const colorImageTopic = pubRunning === true ? this.state.appNamespace + "/color_image" : null
    // The depth map image is published one level under the depth map, at
    // <app>/depth_map/depth_map_image -- the standard DepthMapIF placement, so
    // that depth map consumers find it where they look for it. See the ImageIF
    // construction in file_pub_depthmap_app_node.py.
    const depthMapImageTopic = pubRunning === true ? this.state.appNamespace + "/depth_map/depth_map_image" : null
    const viewableFolders = (this.state.viewableFolders || pubRunning === false)
    const collection_count = this.state.collection_count
    return (

    <Columns>
      <Column>




                        <div style={{ display: 'flex' }}>
                          <div style={{ width: '70%' }}>


                              <NepiIFImageViewer
                                image_topic={colorImageTopic}
                                title={this.state.color_image_text}
                                hideQualitySelector={false}
                              />

                              <NepiIFImageViewer
                                image_topic={depthMapImageTopic}
                                title={this.state.depth_map_image_text}
                                hideQualitySelector={false}
                              />

                          </div>

                          <div style={{ width: '3%' }}>
                            {}
                          </div>

                          <div style={{ width: '27%' }}>


                                  <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
                                  {"Select Folder"}
                                </label>

                                  <div onClick={this.toggleViewableFolders} style={{backgroundColor: Styles.vars.colors.grey0}}>
                                    <Select style={{width: "10px"}}/>
                                  </div>
                                  <div hidden={viewableFolders === false}>
                                  {folderOptions.map((folder) =>
                                  <div onClick={this.onChangeFolderSelection}>
                                    <body value = {folder} style={{color: Styles.vars.colors.black}}>{folder}</body>
                                  </div>
                                  )}
                                  </div>

                                  <Label title={"Current Folder"} >
                                  </Label>
                                  <pre style={{ height: "50px", overflowY: "auto" }}>
                                    {this.state.current_folder}
                                  </pre>

                                  { (collection_count > 0) ?
                                    this.renderPubControls()
                                  : null }

                                  <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                                  {this.renderNavPoseControls()}


                          </div>
                        </div>



  </Column>
    </Columns>

    )
  }

}

export default FilePubDepthmapApp
