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
import Label from "./Label"
import Input from "./Input"
import Styles from "./Styles"
import BooleanIndicator from "./BooleanIndicator"


import NepiIFImageViewer from "./Nepi_IF_ImageViewer"
import NepiIFControls from "./Nepi_IF_Controls"
import NepiIFConfig from "./Nepi_IF_Config"

// The app's THREE ControlsIF set names, matching the CONTROLS_NAME_* constants
// in file_pub_depthmap_app_node.py.
//
// A ControlsIF is always a direct child of the NODE namespace -- it builds
// create_namespace(node_namespace, controls_name) with no namespace argument,
// and get_clean_name() rewrites '/' to '_', so a set cannot live under a
// sub-namespace. The set NAME is the only thing that distinguishes them, and
// both sides derive the same three strings.
//
// The third is navpose_SOURCE, not navpose, and that is load bearing: the node
// mounts a NavPoseIF that roots itself at <app_ns>/navpose, so a control set
// named 'navpose' would collide with it and advertise a second
// <app_ns>/navpose/status of a different message type.
const CONTROLS_NAME_PLAYBACK        = "controls"
const CONTROLS_NAME_FOLDER_SETTINGS = "folder_settings"
const CONTROLS_NAME_NAVPOSE         = "navpose_source"

// The static pose fields, the field of view fields and their local edit buffers
// all moved into the control sets. Nepi_IF_Controls owns that state now, read
// from each set's ControlsStatus, so none of it is mirrored here any more.


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

      // Folder settings. The two report fields are read-only; they say whether
      // the toggle found anything in the current folder and what it did. The
      // toggle itself and the two field of view values are controls now.
      folder_settings_found: false,
      folder_settings_status: '',

      // NavPose source. navpose_active_mode is the RESOLVED source: it is what
      // the explanatory line below reads, and what the node gates the static
      // pose controls' visibility on -- a navpose_source_mode of 'auto' says
      // nothing on its own about which source is publishing.
      navpose_active_mode: 'static',
      navpose_system_available: false,

      statusListener: null,
      connected: false,
      needs_update: true

    }

    this.createFolderOptions = this.createFolderOptions.bind(this)
    this.onChangeFolderSelection = this.onChangeFolderSelection.bind(this)
    this.toggleViewableFolders = this.toggleViewableFolders.bind(this)
    this.renderPubControls = this.renderPubControls.bind(this)
    this.renderControlSet = this.renderControlSet.bind(this)
    this.renderFolderSettingsControls = this.renderFolderSettingsControls.bind(this)
    this.renderNavPoseControls = this.renderNavPoseControls.bind(this)

    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getControlsNamespace = this.getControlsNamespace.bind(this)


  }


  getAppNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var appNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      appNamespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
    }
    return appNamespace
  }

  // Mirror of the CONTROLS_NAME_* constants in the node: each set is a direct
  // child of the app node namespace, never of a sub-namespace.
  getControlsNamespace(controls_name){
    const appNamespace = this.getAppNamespace()
    if (appNamespace === null || appNamespace.indexOf('null') !== -1){
      return null
    }
    return appNamespace + "/" + controls_name
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

      folder_settings_found: message.folder_settings_found ,
      folder_settings_status: message.folder_settings_status ,

      pub_running: message.running,

      navpose_active_mode: message.navpose_active_mode ,
      navpose_system_available: message.navpose_system_available

  })

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



  // One control set, mounted the way the in-workspace examples mount it: the
  // PAGE draws the heading (a divider plus a Label, as it always did) and this
  // passes title={null}, so the component's own default "CONTROLS" heading does
  // not print a second one under it. allways_show_controls keeps the set open --
  // these controls ARE the panel.
  //
  // key={namespace} is the habit the other migrated pages keep: without a key,
  // a namespace change would leave React reusing the mounted component with its
  // old status subscription.
  renderControlSet(controls_name) {
    const namespace = this.getControlsNamespace(controls_name)
    if (namespace === null) {
      return null
    }
    return (
      <NepiIFControls
        key={namespace}
        namespace={namespace}
        title={null}
        make_section={false}
        allways_show_controls={true}
      />
    )
  }

  // Read-only publishing state plus the playback control set.
  //
  // Everything the operator ADJUSTS -- start/stop, pause, rate, random, step and
  // overlay -- is rendered by the shared control renderer from the node's
  // ControlsStatus. What is left here is what the node REPORTS.
  renderPubControls() {
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


          <Label title={"Current Collection"} >
          </Label>
          <pre style={{ height: "25px", overflowY: "auto" }}>
            {this.state.current_collection}
          </pre>


            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
            <Label title={"Playback Controls"} />

            {this.renderControlSet(CONTROLS_NAME_PLAYBACK)}


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




  // Field of view and folder settings.
  //
  // Rendered OUTSIDE the collection_count gate that wraps renderPubControls(),
  // for the same reason renderNavPoseControls() is: the folder settings toggle
  // decides what happens on the NEXT folder selection, so it has to be reachable
  // before a folder with collections has been picked.
  //
  // The two FOV controls stay editable while the toggle is on. The toggle
  // rewrites them on a folder change -- applyFolderSettings pushes the sidecar
  // values back into the controls -- but it does not own them afterwards, so an
  // operator can still correct a value a sidecar got wrong.
  //
  // The two report lines below are read-only status the node publishes; they say
  // whether the last apply found anything and what it did.
  renderFolderSettingsControls() {

    return (

      <div hidden={!this.state.connected}>

        <Label title={"Field of View"} />

        {this.renderControlSet(CONTROLS_NAME_FOLDER_SETTINGS)}

        <Label title={"Settings File Found"}>
          <BooleanIndicator value={this.state.folder_settings_found===true} />
        </Label>

        <Label title={"Folder Settings"} >
        </Label>
        <pre style={{ height: "50px", overflowY: "auto" }}>
          {this.state.folder_settings_status}
        </pre>

      </div>

    )
  }


  // NavPose source controls.
  //
  // Rendered OUTSIDE the collection_count gate that wraps renderPubControls():
  // the node publishes <app>/navpose on its own steady timer whether or not a
  // collection is being published, so the controls for it must be reachable
  // whether or not a folder with collections is selected.
  //
  // The static pose controls and the three frame selections are HIDDEN by the
  // node whenever navpose_active_mode is 'system'. That is not cosmetic -- in
  // system mode the node forwards navpose_mgr's pose with the frames its AUTHOR
  // set, and never reads these values. The node gates on navpose_active_mode
  // rather than navpose_source_mode, which is what makes 'auto' behave: 'auto'
  // resolves to one source or the other, and it is the resolved one that decides
  // whether these values do anything. This page only reports which it resolved
  // to; syncControlVisibility() in the node does the hiding.
  renderNavPoseControls() {
    const active_mode = this.state.navpose_active_mode
    const forwarding = (active_mode === 'system')

    return (

      <div hidden={!this.state.connected}>

        <Label title={"NavPose Source"} />

        {this.renderControlSet(CONTROLS_NAME_NAVPOSE)}

        <Label title={"Publishing Source"}>
          <Input disabled value={active_mode} />
        </Label>

        <Label title={"System NavPose Available"}>
          <BooleanIndicator value={this.state.navpose_system_available===true} />
        </Label>

        <Label title={forwarding
          ? "Forwarding the system NavPose. Its frames are set by its source and are not changed here."
          : "Publishing a static NavPose authored above."} />

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

                                  {this.renderFolderSettingsControls()}

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
