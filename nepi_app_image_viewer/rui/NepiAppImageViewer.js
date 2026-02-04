/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
 */


import React, { Component } from "react"
import { observer, inject } from "mobx-react"

import Section from "./Section"
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Select, { Option } from "./Select"
import Button, { ButtonMenu } from "./Button"
import Styles from "./Styles"
import Toggle from "react-toggle"

import ImageViewersSelector from "./NepiSelectorImageViewers"
import NepiIFSaveData from "./Nepi_IF_SaveData"
import NepiIFConfig from "./Nepi_IF_Config"

import {createMenuFirstLastNames, onChangeSwitchStateValue} from "./Utilities"

@inject("ros")
@observer

// MultiImageViewer Application page
class ImageViewerApp extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_image_viewer",
      appNamespace: null,

      topics: ['None','None','None','None'],
      num_windows: 1,
      statusListener: null,
      connected: false,
      needs_update: false,

      show_selectors: false
    }

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getAllSaveNamespace = this.getAllSaveNamespace.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)

    this.renderImageViewers = this.renderImageViewers.bind(this)
    this.renderSaveData = this.renderSaveData.bind(this)
    this.renderConfig = this.renderConfig.bind(this)

    
    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)

  }


  getBaseNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var baseNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      baseNamespace = "/" + namespacePrefix + "/" + deviceId 
    }
    return baseNamespace
  }

    getAllSaveNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var allNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      allNamespace = "/" + namespacePrefix + "/" + deviceId + '/save_data'
    }
    return allNamespace
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
      topics: message.topics,
      num_windows: message.num_windows
  })
    this.setState({
      connected: true
    })


  }

    // Function for configuring and subscribing to Status
    updateStatusListener(namespace) {
      const statusNamespace = namespace + '/status'
      if (this.state.statusListener) {
        this.state.statusListener.unsubscribe()
      }
      var statusListener = this.props.ros.setupStatusListener(
            statusNamespace,
            "nepi_app_image_viewer/NepiAppImageViewerStatus",
            this.statusListener
          )
      this.setState({ 
        appNamespace: namespace,
        statusListener: statusListener,
      })
    }

    componentDidMount(){
      this.setState({needs_update: true})
    }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getAppNamespace()
    const namespace_updated = (this.state.appNamespace !== namespace && namespace !== null)
    if (namespace_updated) {
      if (namespace.indexOf('null') === -1){
        this.updateStatusListener(namespace)
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



  renderImageViewers() {
     if (this.state.needs_update === true){
      this.setState({needs_update: false})
    }
    const {sendIntMsg} = this.props.ros
    const topics = this.state.topics
    const num_windows = this.state.num_windows

    //Unused const baseNamespace = "/" + namespacePrefix + "/" + deviceId 
    const topics_text = createMenuFirstLastNames(topics)
    const image_filters = ['/all/']
    const appNamespace = this.getAppNamespace()
    const num_windows_updated_topic = appNamespace + '/set_num_windows'
    const select_updated_topics = [
        appNamespace + '/set_topic_1',
        appNamespace + '/set_topic_2',
        appNamespace + '/set_topic_3',
        appNamespace + '/set_topic_4'
    ]

    const mouse_event_topics = [
        appNamespace + '/set_click_pixel',
        appNamespace + '/set_click_pixel',
        appNamespace + '/set_click_pixel',
        appNamespace + '/set_click_pixel'
    ]

      return (
     

      <React.Fragment>


                          <div id="imageviewers">
                            <ImageViewersSelector
                              id="imageviewers"
                              image_topics={topics}
                              titles={topics_text}
                              show_save_controls={true}
                              show_image_options={true}
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



  renderSaveData(){
      const allSaveNamespace = this.getAllSaveNamespace()
      return (
    
      <React.Fragment>

                <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                  <NepiIFSaveData
                    saveNamespace={allSaveNamespace}
                    make_section={false}
                    show_all_options={true}
                    show_topic_selector={true}
                  />
 
      </React.Fragment>

      )
  }

  renderConfig(){
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
    const make_section = (this.props.make_section !== undefined)? this.props.make_section : true
    

    if (make_section === false){
      return (
        <Columns>
        <Column>
              {this.renderImageViewers()}
                {/* {this.renderSaveData()} */}
                  {this.renderConfig()}


        </Column>
        </Columns>
      )
    }
    else {
      return (

      <Section>

              {this.renderImageViewers()}
                {/* {this.renderSaveData()} */}
                  {this.renderConfig()}

      </Section>
      )

    }
  }


}

export default ImageViewerApp
