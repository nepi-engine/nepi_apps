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

// Leaf of the app's ControlsIF namespace. ControlsIF roots itself at
// create_namespace(node_namespace, controls_name), so the control set sits one
// level BELOW the app node namespace. Appending '/controls' here is what makes
// Nepi_IF_Controls subscribe to .../app_file_pub_img/controls/status and publish
// its updates to .../app_file_pub_img/controls/update_control.
const CONTROLS_NAME = "controls"


@inject("ros")
@observer

class FilePubImgApp extends Component {
  constructor(props) {
    super(props)

    this.state = {
		
      appName: 'app_file_pub_img',
	    appNamespace: null,
      status_msg: null,

      image_topic: 'images',
      image_text: 'file_pub_img/images',

      viewableFolders: false,

      home_folder: 'None',
      current_folder: null,
      selected_folder: 'Home',
      current_folders: [],
      supported_file_types: [],
      selected_file: 'Home',
      file_count: 0,
      current_file: 'None',

      paused: false,

      size_options_list: ['None'],
      set_size: 'None',
      encoding_options_list: ['None'],
      set_encoding: 'None',


      set_random: false,
      set_overlay: false,
      min_max_rate: [0.1,20],
      set_rate: 1,
      pub_running: false,

      statusListener: null,
      connected: false,
      needs_update: true

    }

    this.createFolderOptions = this.createFolderOptions.bind(this)
    this.onChangeFolderSelection = this.onChangeFolderSelection.bind(this)
    this.toggleViewableFolders = this.toggleViewableFolders.bind(this)

    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getControlsNamespace = this.getControlsNamespace.bind(this)
    this.renderPubControls = this.renderPubControls.bind(this)


  }


  getAppNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var appNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      appNamespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
    }
    return appNamespace
  }

  getControlsNamespace(){
    const appNamespace = this.getAppNamespace()
    if (appNamespace !== null){
      return appNamespace + "/" + CONTROLS_NAME
    }
    return null
  }

  // Callback for handling ROS Status messages
  statusListener(message) {
    this.setState({
      status_msg: message,
      home_folder: message.home_folder ,
      current_folders: message.current_folders ,
      selected_folder: message.selected_folder,
      supported_file_types: message.supported_file_types,
      file_count: message.file_count ,
      current_file: message.current_file ,
      paused: message.paused ,

      size_options_list: message.size_options_list ,
      set_size: message.set_size ,
      encoding_options_list: message.encoding_options_list ,
      set_encoding: message.set_encoding ,


      set_random: message.set_random ,
      set_overlay: message.set_overlay ,
      min_max_rate: message.min_max_rate ,

      pub_running: message.running

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
            "nepi_app_file_pub_img/FilePubImgStatus",
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



  // Read-only publishing state plus the app's control set.
  //
  // Everything the operator ADJUSTS -- start/stop, pause, rate, random, step,
  // size, encoding and overlay -- is rendered by the shared control renderer
  // from the node's ControlsStatus. What is left here is what the node REPORTS.
  //
  // title is passed as null (the Nepi_IF_Process treatment) because the Label
  // above is already this block's heading, and the component's own default
  // title would print a second one under it. allways_show_controls keeps the
  // set open: these controls ARE the panel, so there is nothing left to see if
  // they are collapsed behind a toggle.
  renderPubControls() {
    const appNamespace = this.state.appNamespace
    const controlsNamespace = this.getControlsNamespace()
    const pubRunning = this.state.pub_running

    return (


    <Columns>
    <Column>


        <div hidden={!this.state.connected}>


            <Label title={"Image Count"}>
            <Input disabled value={this.state.file_count} />
            </Label>


          <Label title={"Publishing"}>
              <BooleanIndicator value={pubRunning} />
            </Label>


          <Label title={"Current File"} >
          </Label>
          <pre style={{ height: "25px", overflowY: "auto" }}>
            {this.state.current_file}
          </pre>


            { (controlsNamespace !== null && controlsNamespace.indexOf('null') === -1) ?

              <React.Fragment>
                <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
                <Label title={"Publish Controls"} />
                <NepiIFControls
                  key={controlsNamespace}
                  namespace={controlsNamespace}
                  title={null}
                  make_section={false}
                  allways_show_controls={true}
                />
              </React.Fragment>

            : null }


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
    const appImageTopic = pubRunning === true ? this.state.appNamespace + "/color_image" : null
    const viewableFolders = (this.state.viewableFolders || pubRunning === false)
    const file_count = this.state.file_count
    return (

    <Columns>
      <Column>




                        <div style={{ display: 'flex' }}>
                          <div style={{ width: '70%' }}>


                              <NepiIFImageViewer
                                image_topic={appImageTopic}
                                title={this.state.image_text}
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

                                  { (file_count > 0) ?
                                    this.renderPubControls()
                                  : null }
                                  


                          </div>
                        </div>



  </Column>
    </Columns>

    )
  }

}

export default FilePubImgApp
