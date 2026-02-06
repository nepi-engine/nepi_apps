/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi rui (nepi_rui) repo
# (see https://github.com/nepi-engine/nepi_rui)
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

//import moment from "moment"
import { observer, inject } from "mobx-react"

//import { TransformWrapper, TransformComponent } from "react-zoom-pan-pinch";

//import Section from "./Section"
import Button, { ButtonMenu } from "./Button"

import { Column, Columns } from "./Columns"
import Styles from "./Styles"


import { SliderAdjustment } from "./AdjustmentWidgets"

import ImageViewersSelector from "./NepiSelectorImageViewers"
import {createMenuFirstLastNames} from "./Utilities"

import NepiIFSaveData from "./Nepi_IF_SaveData"
import NepiIFConfig from "./Nepi_IF_Config"



function round(value, decimals = 0) {
  return Number(value).toFixed(decimals)
  //return value && Number(Math.round(value + "e" + decimals) + "e-" + decimals)
}


@inject("ros")
@observer
class NepiAppPTAutoImageViewer extends Component {
  constructor(props) {
    super(props)

    this.state = {

      appNamespace: null,
      status_msg: null, 

      available_topics: [],
      selected_topic: null,
      connected: false,
      connected_topic: null,

      image_topics: ['None','None','None','None'],
      num_windows: 1,


      statusPtListener: null,
      pt_status_msg: null,


 
      statusListener: null,
      needs_update: false
    }


    
    this.renderImageViewers = this.renderImageViewers.bind(this)
    this.renderSaveData = this.renderSaveData.bind(this)
    this.renderConfig = this.renderConfig.bind(this)

    this.updateStatusPtListener = this.updateStatusPtListener.bind(this)
    this.statusPtListener = this.statusPtListener.bind(this)

    this.getAllSaveNamespace = this.getAllSaveNamespace.bind(this)

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


  getAllSaveNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var allNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      allNamespace = "/" + namespacePrefix + "/" + deviceId + '/save_data'
    }
    return allNamespace
  }

  // Callback for handling ROS Status3DX messages
  statusListener(message) {
    if ((this.state.selected_topic != message.selected_topic) && (message.selected_topic !== '' && message.selected_topic !== 'None')) {
      this.updateStatusPtListener(message.selected_topic)
    }
    this.setState({
      status_msg: message,
      available_topics: message.available_topics,
      selected_topic: message.selected_topic,
      connected: message.connected,
      image_topics: message.image_topics,
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

  // Callback for handling ROS Status3DX messages
  statusPtListener(message) {
    this.setState({
      pt_status_msg: message
    })
  }
  
  // Function for configuring and subscribing to Status
  updateStatusPtListener(namespace) {
    if (this.state.statusPtListener != null) {
      this.state.statusPtListener.unsubscribe()
       this.setState({ pt_status_msg: null, statusPtListener: null})
    }
    if (namespace != null && namespace !== 'None'){
        var statusPtListener = this.props.ros.setupPTXStatusListener(
              namespace,
              this.statusPtListener
            )
      this.setState({ statusPtListener: statusPtListener})
    }

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
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
      this.setState({statusListener : null})
    }
  }

  renderConfig(){
    const namespace = this.getAppNamespace()
    return (
  
    <React.Fragment>

           <NepiIFConfig
                namespace={namespace}
                title={"Nepi_IF_Config"}
            />

    </React.Fragment>

    )
}



    renderSaveData(){
      const allSaveNamespace = this.getAllSaveNamespace()
      const saveNamespace = (this.props.saveNamespace !== undefined) ? this.props.saveNamespace : allSaveNamespace
      const show_save_controls = (this.props.show_save_controls !== undefined) ? this.props.show_save_controls : true

      if (show_save_controls === false){
          return (
            <Columns>
            <Column>

            </Column>
            </Columns>

          )

      }
      else {
          return (
        
              <React.Fragment>

                         
                          
                          <NepiIFSaveData
                            saveNamespace={saveNamespace}
                            make_section={false}
                            show_all_options={true}
                            show_topic_selector={true}
                          />
        
                  <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
              </React.Fragment>

          )
        }
  }



  renderImageViewers() {
     if (this.state.needs_update === true){
      this.setState({needs_update: false})
    }
    const {sendIntMsg} = this.props.ros
    const num_windows = (this.props.num_windows !== undefined) ? this.props.num_windows : this.state.num_windows
    const image_topics = (this.props.image_topics !== undefined) ? this.props.image_topics : ['None','None','None','None']
    const namespace = (this.props.namespace !== null) ? this.props.namespace : 'None'
    //Unused const baseNamespace = "/" + namespacePrefix + "/" + deviceId 
    const topics_text = createMenuFirstLastNames(image_topics)
    const image_filters = ['/all/']
    const num_windows_updated_topic = namespace + '/set_num_windows'
    const select_updated_topics = [
        namespace + '/set_topic_1',
        namespace + '/set_topic_2',
        namespace + '/set_topic_3',
        namespace + '/set_topic_4'
    ]

    const mouse_event_topics = [
        namespace + '/set_click_position',
        namespace + '/set_click_position',
        namespace + '/set_click_position',
        namespace + '/set_click_position'
    ]

      return (
     

      <React.Fragment>


                          <div id="imageviewers">
                            <ImageViewersSelector
                              id="imageviewers"
                              image_topics={image_topics}
                              titles={topics_text}
                              show_save_controls={false}
                              show_image_options={true}
                              num_windows={num_windows}
                              num_windows_updated_topic={num_windows_updated_topic}
                              select_updated_topics={select_updated_topics}
                              mouse_event_topics={mouse_event_topics}
                              image_filters={image_filters}
                              make_section={false}
                            />
                          </div>        
 
      </React.Fragment>

      )
  }



  render() {
    const { ptxDevices, onPTXJogPan, onPTXJogTilt, onPTXStop } = this.props.ros
    const namespace = this.getAppNamespace()
    const status_msg = this.state.status_msg
    const pt_status_msg = this.state.pt_status_msg
    var panGoalRatio = 0.5
    var tiltGoalRatio = 0.5
    if (pt_status_msg != null){
      panGoalRatio = pt_status_msg.pan_goal_ratio
      tiltGoalRatio = pt_status_msg.tilt_goal_ratio
    }
   

    const ptxDevicesList = Object.keys(ptxDevices)
    var has_abs_pos = false
    var has_timed_pos = false

    const ptNamespace = this.state.selected_topic
    if (ptxDevicesList.indexOf(ptNamespace) !== -1){
      const ptx_caps = ptxDevices[ptNamespace]
      has_abs_pos = ptx_caps && (ptx_caps.has_absolute_positioning === true)
      has_timed_pos = ptx_caps && (ptx_caps.has_timed_positioning === true)
    }

    const imageviewersElement = document.getElementById("imageviewers")
    const tiltSliderHeight = (imageviewersElement)? Math.floor(imageviewersElement.offsetHeight * 0.85) : 1
    const show_pt_controls = (tiltSliderHeight === 1) ? false : (has_abs_pos === true)


    return (

        <Columns>
          <Column equalWidth = {false} >

             

              {this.renderSaveData()}

          {this.renderImageViewers()}


              <div hidden={show_pt_controls === false}>

                  <SliderAdjustment
                    title={"Pan"}
                    msgType={"std_msgs/Float32"}
                    adjustment={panGoalRatio}
                    topic={ptNamespace + "/goto_pan_ratio"}
                    scaled={0.01}
                    min={0}
                    max={100}
                    tooltip={"Pan as a percentage (0%=min, 100%=max)"}
                    unit={"%"}
                    noTextBox={true}
                    noLabel={true}
                  />
        

              <ButtonMenu>

                  <Button 
                    buttonDownAction={() => onPTXJogPan(ptNamespace,  1)}
                    buttonUpAction={() => onPTXStop(ptNamespace)}>
                    {'\u25C0'}
                    </Button>
                  <Button 
                    buttonDownAction={() => onPTXJogPan(ptNamespace, - 1)}
                    buttonUpAction={() => onPTXStop(ptNamespace)}>
                    {'\u25B6'}
                  </Button>
                  <Button 
                    buttonDownAction={() => onPTXJogTilt(ptNamespace, 1)}
                    buttonUpAction={() => onPTXStop(ptNamespace)}>
                    {'\u25B2'}
                  </Button>
                  <Button 
                    buttonDownAction={() => onPTXJogTilt(ptNamespace, -1)}
                    buttonUpAction={() => onPTXStop(ptNamespace)}>
                    {'\u25BC'}
                  </Button>

                </ButtonMenu>


             

                <ButtonMenu>

                  <Button onClick={() => onPTXStop(ptNamespace)}>{"STOP"}</Button>
                  
                </ButtonMenu>

                <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

             </div>



              {this.renderConfig()}
          </Column>
          <Column style={{flex: 0.05}}>

           <div hidden={show_pt_controls === false}>

            <SliderAdjustment
              title={"Tilt"}
              msgType={"std_msgs/Float32"}
              adjustment={tiltGoalRatio}
              topic={ptNamespace + "/goto_tilt_ratio"}
              scaled={0.01}
              min={0}
              max={100}
              tooltip={"Tilt as a percentage (0%=min, 100%=max)"}
              unit={"%"}
              vertical={true}
              verticalHeight={tiltSliderHeight}
              noTextBox={true}
              noLabel={true}
            />

          </div>

        </Column>
        </Columns>




    )
  }


}

export default NepiAppPTAutoImageViewer
