/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
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

import { onDropdownSelectedSendStr, createMenuListFromStrList} from "./Utilities"


@inject("ros")
@observer

class FilePubVidApp extends Component {
  constructor(props) {
    super(props)

    this.state = {
		
      appName: 'app_file_pub_vid',
	    appNamespace: null,
      image_topic: 'images',
      image_text: 'file_pub_vid/images',

      viewableFolders: false,

      home_folder: 'None',
      current_folder: null,
      selected_folder: 'Home',
      current_folders: [],
      selected_file: 'Home',
      supported_file_types: [],
      file_count: 0,
      current_file: 'None',

      paused: false,

      size_options_list: ['None'],
      set_size: 'None',
      encoding_options_list: ['None'],
      set_encoding: 'None',


      set_random: false,
      set_overlay: false,

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
            "nepi_app_file_pub_vid/FilePubVidStatus",
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
    const NoneOption = <Option>None</Option>

    return (


    <Columns>
    <Column>


        <div hidden={!this.state.connected}>


            <Label title={"Video Count"}>
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


                      <div hidden={this.state.paused === false}>
                                <ButtonMenu>
                                  <Button onClick={() => this.props.ros.sendTriggerMsg(appNamespace + "/step_forward")}>{"Forward"}</Button>
                                </ButtonMenu>

                        </div>

                
            </Column>
            </Columns>


            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>




          <Label title={"Current File"} >
          </Label>
          <pre style={{ height: "25px", overflowY: "auto" }}>
            {this.state.current_file}
          </pre>

            <Label title={"Set Image Size"}>
            <Select
              id="select_targset_sizeet"
              onChange={(event) => onDropdownSelectedSendStr.bind(this)(event, appNamespace + "/set_size")}
              value={this.state.set_size}
            >
              {this.state.size_options_list
                ? createMenuListFromStrList(this.state.size_options_list, false, [],[],[])
                : NoneOption}
            </Select>
            </Label>


            <Label title={"Set Image Encoding"}>
            <Select
              id="set_encoding"
              onChange={(event) => onDropdownSelectedSendStr.bind(this)(event, appNamespace + "/set_encoding")}
              value={this.state.set_encoding}
            >
              {this.state.encoding_options_list
                ? createMenuListFromStrList(this.state.encoding_options_list, false, [],[],[])
                : NoneOption}
            </Select>
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
    const pubRunning = this.state.pub_running
    const appImageTopic = pubRunning === true ? this.state.appNamespace + "/color_image" : null
    const viewableFolders = (this.state.viewableFolders || pubRunning === false)
    const file_count = this.state.file_count
    return (


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
                        <pre style={{ height: "25px", overflowY: "auto" }}>
                          {this.state.current_folder}
                        </pre>

                      { (file_count > 0) ?
                        this.renderPubControls()
                      : null }


              </div>
            </div>


    )
  }

}

export default FilePubVidApp
