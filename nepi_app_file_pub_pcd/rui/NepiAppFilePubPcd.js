/*
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

import NepiIFConfig from "./Nepi_IF_Config"

import { onEnterSendFloatValue,  onUpdateSetStateValue} from "./Utilities"


@inject("ros")
@observer

class FilePubPcdApp extends Component {
  constructor(props) {
    super(props)

    this.state = {
		
      appName: 'app_file_pub_pcd',
	    appNamespace: null,

      viewableFolders: false,

      home_folder: 'None',
      current_folder: null,
      selected_folder: 'Home',
      current_folders: [],
      supported_file_types: [],
      selected_file: 'Home',
      file_count: 0,
      max_pubs: 5,
      current_file_list: [],
      current_topic_list: [],

      paused: false,

      available_files_list: [],
      selected_files_list: [],

      min_max_delay: 1,
      set_delay: 1,

      pub_transforms: false,
      create_transforms: false,

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

    this.toggleViewableTopics = this.toggleViewableTopics.bind(this)
    this.onToggleFileSelection = this.onToggleFileSelection.bind(this)
    this.getFilesOptions = this.getFilesOptions.bind(this)
    this.getFileOptions = this.getFileOptions.bind(this)

    

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
      home_folder: message.home_folder ,
      current_folders: message.current_folders ,
      selected_folder: message.selected_folder,
      supported_file_types: message.supported_file_types,
      file_count: message.file_count ,
      max_pubs: message.max_pubs,
      current_file_list: message.current_file_list ,
      current_topic_list: message.current_topic_list ,
      
      paused: message.paused ,

      min_max_delay: message.min_max_delay ,
      set_delay: message.set_delay ,

      pub_transforms: message.pub_transforms,
      create_transforms: message.create_transforms,

      pub_running: message.running,

      available_files_list: message.available_files_list,
      selected_files_list: message.selected_files_list
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
            "nepi_app_file_pub_pcd/FilePubPcdStatus",
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

  toggleViewableTopics() {
    const set = !this.state.viewableTopics
    this.setState({viewableTopics: set})
  }

  onToggleFileSelection(event){
    const {sendTriggerMsg, sendStringMsg} = this.props.ros
    const appNamespace = this.getAppNamespace()
    const fileSelection = event.target.value
    const selectedFiles = this.state.selected_files_list
    const addAllNamespace = appNamespace + "/add_all_pcd_files"
    const removeAllNamespace = appNamespace + "/remove_all_pcd_files"
    const addNamespace = appNamespace + "/add_pcd_file"
    const removeNamespace = appNamespace + "/remove_pcd_file"
    if (appNamespace){
      if (fileSelection === "None"){
          sendTriggerMsg(removeAllNamespace)
      }
      else if (fileSelection === "All"){
        sendTriggerMsg(addAllNamespace)
    }
      else if (selectedFiles.indexOf(fileSelection) !== -1){
        sendStringMsg(removeNamespace,fileSelection)
      }
      else {
        sendStringMsg(addNamespace,fileSelection)
      }
    }
  }

  getFilesOptions() {
    const filesList = this.state.available_files_list
    var items = []
    items.push(<Option>{"None"}</Option>)
    items.push(<Option>{"All"}</Option>)
    if (filesList.length > 0 ){
      for (var i = 0; i < filesList.length; i++) {
          if (filesList[i] !== 'None'){
            items.push(<Option value={filesList[i]}>{filesList[i]}</Option>)
          }
      }
    }
    return items
    }

    getFileOptions() {
      const filesList = this.state.available_files_list
      var items = []
      items.push(<Option>{"None"}</Option>)
      items.push(<Option>{"All"}</Option>)
      if (filesList.length > 0 ){
        for (var i = 0; i < filesList.length; i++) {
            if (filesList[i] !== 'None'){
              items.push(<Option value={filesList[i]}>{filesList[i]}</Option>)
            }
        }
      }
      return items
      }

  renderPubControls() {
    const appNamespace = this.state.appNamespace
    const pubRunning = this.state.pub_running
    return (


    <Columns>
    <Column>


        <div hidden={!this.state.connected}>

            <Label title={"Current Folder"}>
            <Input disabled value={this.state.current_folder} />
            </Label>

            <Label title={"PointCloud File Count"}>
            <Input disabled value={this.state.file_count} />
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

            <NepiIFConfig
                            namespace={appNamespace}
                            title={"Nepi_IF_Conig"}
            />


            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>




            <Label title={"Max Files"}>
            <Input disabled value={this.state.max_pubs} />
            </Label>

              <Label title={"Set Delay (Seconds)"}>
            <Input id="set_delay" 
              value={this.state.set_delay} 
              onChange={(event) => onUpdateSetStateValue.bind(this)(event,"set_delay")} 
              onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event,appNamespace + "/set_delay")} />
             </Label>

             <Label title="Publish Transforms ">
                  <Toggle
                    checked={this.state.pub_transforms===true}
                    onClick={() => this.props.ros.sendBoolMsg(appNamespace + "/set_pub_transforms",!this.state.pub_transforms)}>
                  </Toggle>
            </Label>

            <Label title="Create Transforms">
                  <Toggle
                    checked={this.state.create_transforms===true}
                    onClick={() => this.props.ros.sendBoolMsg(appNamespace + "/set_create_transforms",!this.state.create_transforms)}>
                  </Toggle>
            </Label>
        </div>

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
    const fileOptions = this.getFileOptions()

    const selectedFiles = this.state.selected_files_list
    const pubRunning = this.state.pub_running
    const viewableFolders = (this.state.viewableFolders || pubRunning === false)
    return (

    <Columns>
      <Column>
      {/*
          <CameraViewer
            imageTopic={appImageTopic}
            title={this.state.imageText}
            hideQualitySelector={false}
          />
    */}

    </Column>
    <Column>


        <Columns>
        <Column>


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
    

        </Column>
        <Column>

        <Label title="Select Files"> </Label>

        <div onClick={this.toggleViewableTopics} style={{backgroundColor: Styles.vars.colors.grey0}}>
          <Select style={{width: "10px"}}/>
        </div>
        <div hidden={this.state.viewableTopics === false}>
        {fileOptions.map((file) =>
        <div onClick={this.onToggleFileSelection}
          style={{
            textAlign: "center",
            padding: `${Styles.vars.spacing.xs}`,
            color: Styles.vars.colors.black,
            backgroundColor: (selectedFiles.includes(file.props.value))? Styles.vars.colors.blue : Styles.vars.colors.grey0,
            cursor: "pointer",
            }}>
            <body file_name ={file} style={{color: Styles.vars.colors.black}}>{file}</body>
        </div>
        )}
        </div>


        </Column>
        <Column>

        {this.renderPubControls()}

        </Column>
        </Columns>





  </Column>
    </Columns>

    )
  }

}

export default FilePubPcdApp
