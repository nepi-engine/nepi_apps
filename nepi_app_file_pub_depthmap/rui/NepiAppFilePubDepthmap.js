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
import Toggle from "react-toggle"
import Styles from "./Styles"
import BooleanIndicator from "./BooleanIndicator"


import NepiIFImageViewer from "./Nepi_IF_ImageViewer"
import NepiIFConfig from "./Nepi_IF_Config"

import { onUpdateSetStateValue, onEnterSendFloatValue} from "./Utilities"


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
      depth_map_image_text: 'file_pub_depthmap/depth_map_image',

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
                            <Toggle
                            checked={this.state.paused===true}
                            onClick={() => sendBoolMsg(appNamespace + "/pause_pub",!this.state.paused)}>
                            </Toggle>
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
                                  <Toggle
                                  checked={this.state.set_random===true}
                                  onClick={() => sendBoolMsg(appNamespace + "/set_random",!this.state.set_random)}>
                                  </Toggle>
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
              <Toggle
              checked={this.state.set_overlay===true}
              onClick={() => sendBoolMsg(appNamespace + "/set_overlay",!this.state.set_overlay)}>
              </Toggle>
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
    const depthMapImageTopic = pubRunning === true ? this.state.appNamespace + "/depth_map_image" : null
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



                          </div>
                        </div>



  </Column>
    </Columns>

    )
  }

}

export default FilePubDepthmapApp
