/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
import React, { Component } from "react"
import { observer, inject } from "mobx-react"
//import Toggle from "react-toggle"

import Section from "./Section"
import { Columns, Column } from "./Columns"
import Select, { Option } from "./Select"
import Label from "./Label"
//import Input from "./Input"
import Styles from "./Styles"
//import Button, { ButtonMenu } from "./Button"
//import {setElementStyleModified, clearElementStyleModified, onUpdateSetStateValue} from "./Utilities"
//import {createShortValuesFromNamespaces} from "./Utilities"
import BooleanIndicator from "./BooleanIndicator"

//import {onChangeSwitchStateValue } from "./Utilities"

import NepiDevicePTXControls from "./NepiDevicePTX-Controls"

import NepiPTAutoImageViewer from "./NepiAppPTAuto-ImageViewer"
import NepiAppPTAutoControls from "./NepiAppPTAuto-Controls"
import NepiIFSettings from "./Nepi_IF_Settings"
//Unused import NepiIFSaveData from "./Nepi_IF_SaveData"
import NepiIFConfig from "./Nepi_IF_Config"
//import NepiSystemMessages from "./Nepi_IF_Messages"

import NavPoseViewer from "./Nepi_IF_NavPoseViewer"



function round(value, decimals = 0) {
  return Number(value).toFixed(decimals)
  //return value && Number(Math.round(value + "e" + decimals) + "e-" + decimals)
}

@inject("ros")
@observer

// Component that contains the  controls
class NepiAppPTAuto extends Component {
  constructor(props) {
    super(props)

    this.state = {
      appName: 'app_pan_tilt_auto',
      appNamespace: null,
      status_msg: null,

      available_pan_tilts: [],
      selected_pan_tilt: null,
      connected: false,
      connected_topic: null,

      selected_image_topics: ['None','None','None','None'],
      num_windows: 1,

      statusListener: null,
      needs_update: false,


    }

    this.renderControls = this.renderControls.bind(this)
    
    this.createPtMenuOptions = this.createPtMenuOptions.bind(this)
    this.onClickToggleShowSettings = this.onClickToggleShowSettings.bind(this)
    this.onPtDeviceSelected = this.onPtDeviceSelected.bind(this)


    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
  }


  getAppNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var namespace = null
    if (namespacePrefix != null && deviceId != null){
      if (this.props.namespace !== undefined){
        namespace = this.props.namespace
      }
      else{
        namespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
      }
    }
    return namespace
  }

  // Callback for handling ROS Status3DX messages
  statusListener(message) {
    this.setState({
      status_msg: message,
      available_pan_tilts: message.available_pan_tilts,
      selected_pan_tilt: message.selected_pan_tilt,
      connected: message.connected,
      selected_image_topics: message.selected_image_topics,
      num_windows: message.num_windows
    })
  }
  
  // Function for configuring and subscribing to Status
  updateStatusListener(namespace) {
    if (this.state.statusListener != null) {
      this.state.statusListener.unsubscribe()
      this.setState({statusListener: null, status_msg: null})
    }
    if (namespace != null && namespace !== 'None' && namespace.indexOf('null') === -1){
        const statusNamespace = namespace + '/status'
        var statusListener = this.props.ros.setupStatusListener(
              statusNamespace,
              "nepi_app_pan_tilt_auto/PanTiltAutoAppStatus",
              this.statusListener
            )
    this.setState({ 
      statusListener: statusListener,
    })
    }
    this.setState({ 
      appNamespace: namespace,
      needs_update: false
    })
  }
  
  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getAppNamespace()
    if ((namespace != null && namespace !== this.state.appNamespace) || this.state.needs_update === true){
        this.updateStatusListener(namespace)
    }
  }

  componentDidMount() {
    this.setState({needs_update: true
    })
  }


  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status3DX message
  componentWillUnmount() {
    if (this.state.listener) {
      this.state.listener.unsubscribe()
      this.setState({listener : null})
    }
  }



  // Function for creating topic options for Select input
  createPtMenuOptions() {
    const {sendStringMsg} = this.props.ros
    const namespace = this.getAppNamespace()
    const topics = this.state.available_pan_tilts
    const sel_topic = this.state.selected_pan_tilt
    var items = []
    var i
    //var unique_names = createShortUniqueValues(topics)
    var device_name = ""


    items.push(<Option value={"None Availble"}>{"None"}</Option>)

    if (topics.length > 0){
      for (i = 0; i < topics.length; i++) {
        device_name = topics[i].split('/ptx')[0].split('/').pop()
        items.push(<Option value={topics[i]}>{device_name}</Option>)
      }
    }
    if (sel_topic === 'None' && topics.length > 0){
          this.setState({selected_pan_tilt: topics[0]})
          const selectNamespace = namespace + "/select_pt_device"
          sendStringMsg(selectNamespace,topics[0])
    }
    return items
  }




  onClickToggleShowSettings(){
    const currentVal = this.state.showSettings 
    this.setState({showSettings: !currentVal})
    this.render()
  }

  onPtDeviceSelected(event) {
    const {sendStringMsg} = this.props.ros
    const namespace = this.getAppNamespace()
    const item = event.target.value
    //var item_ind = this.ordered_items_list.index(item)
    //if (item_ind != -1){
    this.setState({selected_pan_tilt: item})
    const selectNamespace = namespace + "/select_pt_device"
    sendStringMsg(selectNamespace,item)
   // }
  }


  renderControls() {

    const appNamespace = this.getAppNamespace()
    const selected_pan_tilt = this.state.selected_pan_tilt
    const ptConnected = this.state.connected
    const ptMenuItems = this.createPtMenuOptions()
    return (




      
      <React.Fragment>

          { (ptConnected === true) ? 

            <NepiAppPTAutoControls
                namespace={appNamespace}
                make_section={true}

                title={"Auto Controls"}
            />
          : null }

          <Section>
              <Label title={"Device"}>
                  <Select
                    onChange={this.onPtDeviceSelected}
                    value={selected_pan_tilt}
                  >
                    {ptMenuItems}
                  </Select>
                </Label>

            { (ptConnected === true) ? 
               <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
            : null }

          { (ptConnected === true) ? 

                <NepiDevicePTXControls
                    namespace={selected_pan_tilt}
                    make_section={false}

                    title={"Pan Tilt Controls"}
                />

            : null }
          </Section>




          </React.Fragment>
        )
  }


    
  render() {
    const selected_pan_tilt = this.state.selected_pan_tilt
    const ptConnected = this.state.connected
    const num_windows = this.state.num_windows
    const selected_image_topics = this.state.selected_image_topics
    const clicking_enabled = (this.state.status_msg != null) ? (this.state.status_msg.click_pan_enabled === true || this.state.status_msg.click_tilt_enabled === true) : false
    const namespace = this.getAppNamespace()
    const mouse_event_topic = (clicking_enabled === true) ? namespace + '/set_click_position' : null

    return (
      <React.Fragment>



      <div style={{ display: 'flex' }}>

            <div style={{ width: '68%' }}>

                { (ptConnected === true) ?
                    <Section>
                      <div id="ptAutoImageViewer">
                              <NepiPTAutoImageViewer
                                id="ptAutoImageViewer"
                                show_image_options={false}
                                namespace={namespace}
                                mouse_event_topic={mouse_event_topic}
                                num_windows={num_windows}
                                image_topics={selected_image_topics}
                              />
                            </div>
                      </Section>
                  : null }

            </div>

            <div style={{ width: '2%' }} centered={"true"} >
                  {}
            </div>

            <div style={{ width: '30%' }}>


            { this.renderControls()}


            { (ptConnected === true) ?
              <NepiIFSettings
                namespace={namespace}
                title={"Nepi_IF_Settings"}
              />
            : null }


            </div>

      </div>


      </React.Fragment>
    )
  }
}

export default NepiAppPTAuto
