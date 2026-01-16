/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
 */


import React, { Component } from "react"
import { observer, inject } from "mobx-react"

//import Section from "./Section"
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Select, { Option } from "./Select"
import Button, { ButtonMenu } from "./Button"
import Styles from "./Styles"
import Toggle from "react-toggle"

import ImageViewerSelector from "./NepiSelectorImageViewer"
import NepiIFConfig from "./Nepi_IF_Config"
import NepiIFSaveData from "./Nepi_IF_SaveData"

import {createShortValuesFromNamespaces, onChangeSwitchStateValue} from "./Utilities"

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
    this.getAppNamespace = this.getAppNamespace.bind(this)

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


  render() {
    if (this.state.needs_update === true){
      this.setState({needs_update: false})
    }
    const {sendIntMsg} = this.props.ros
    const topics = this.state.topics
    const num_windows = this.state.num_windows

    //Unused const baseNamespace = "/" + namespacePrefix + "/" + deviceId 
    const topics_text = createShortValuesFromNamespaces(topics)
    const image_filters = ['all']
    const appNamespace = this.getAppNamespace()
    const num_windows_namespace = appNamespace + '/set_num_windows'
    const streamingImageQuality = (num_windows > 1) ? 50 : 95
    const has_col_2 = (num_windows > 1) ? true : false
    const colFlexSize_1 = (has_col_2 === false)? "100%" : "50%"
    const colFlexSize_2 = (has_col_2 === false)? "0%" : "50%"
    
    const connected = this.state.connected

    const show_image_controls = false
    const show_selectors = this.state.show_selectors
    
    
    if (connected === false){
      return (

            <Columns>
              <Column>


              </Column>
            </Columns>

      )


    }
    else {
      return (
     
        <div>
                          <div style={{ display: 'flex' }}>
                                <div style={{ width: '5%' }}>
                                  {}
                                </div>

                                <div style={{ width: '10%' }}>
                                  <ButtonMenu>
                                    <Button onClick={() => sendIntMsg( num_windows_namespace,1)}>{"1 Window"}</Button>
                                  </ButtonMenu>
                                </div>

                                <div style={{ width: '5%' }}>
                                  {}
                                </div>

                                <div style={{ width: '10%' }}>
                                  <ButtonMenu>
                                    <Button onClick={() => sendIntMsg( num_windows_namespace,2)}>{"2 Windows"}</Button>
                                  </ButtonMenu>
                                </div>

                                <div style={{ width: '5' }}>
                                  {}
                                </div>

                                <div style={{ width: '10%' }}>
                                  <ButtonMenu>
                                    <Button onClick={() => sendIntMsg( num_windows_namespace,3)}>{"3 Windows"}</Button>
                                  </ButtonMenu>
                                </div>

                                <div style={{ width: '5%' }}>
                                  {}
                                </div>
                
                                <div style={{ width: '10%' }}>
                                  <ButtonMenu>
                                    <Button onClick={() => sendIntMsg( num_windows_namespace,4)}>{"4 Windows"}</Button>
                                  </ButtonMenu>
                                </div>

                                <div style={{ width: '5%' }}>
                                  {}
                                </div>

                                <div style={{ width: '10%' }}></div>

                                  <Label title="Show Selectors">
                                    <Toggle
                                      checked={this.state.show_selectors===true}
                                      onClick={() => onChangeSwitchStateValue.bind(this)("show_selectors",this.state.show_selectors)}>
                                    </Toggle>
                                </Label>

                             </div>

                                <div style={{ width: '25%' }}>
                                  {}
                                </div>


            <div style={{ display: 'flex' }}>
            
                  <div style={{ width: colFlexSize_1 }}>
                        <div id="Image1Viewer">
                          <ImageViewerSelector
                            id="Image1Viewer"
                            imageTopic={topics[0]}
                            title={topics_text[0]}
                            show_image_options={show_image_controls}
                            select_updated_namespace={appNamespace + '/set_topic_1'}
                            streamingImageQuality={streamingImageQuality}
                            image_filters={image_filters}
                            show_selector={show_selectors}
                            show_buttons={show_selectors}
                          />
                        </div>

                        {(num_windows > 2)?
                          <div id="Image3Viewer">
                            <ImageViewerSelector
                              id="Image3Viewer"
                              imageTopic={topics[2]}
                              title={topics_text[2]}
                              show_image_options={show_image_controls}
                              select_updated_namespace={appNamespace + '/set_topic_3'}
                              streamingImageQuality={streamingImageQuality}
                              image_filters={image_filters}
                              show_selector={show_selectors}
                              show_buttons={show_selectors}
                            />
                          </div>        
                        : null
                        }
                  </div>


                  <div style={{ width: colFlexSize_2 }}>

                        {(num_windows > 1)?
                          <div id="Image2Viewer">
                            <ImageViewerSelector
                              id="Image2Viewer"
                              imageTopic={topics[1]}
                              title={topics_text[1]}
                              show_image_options={show_image_controls}
                              select_updated_namespace={appNamespace + '/set_topic_2'}
                              streamingImageQuality={streamingImageQuality}
                              image_filters={image_filters}
                            show_selector={show_selectors}
                            show_buttons={show_selectors}
                            />
                          </div>          
                        : null
                        }

                        {(num_windows === 4)?
                          <div id="Image4Viewer">
                            <ImageViewerSelector
                              id="Image4Viewer"
                              imageTopic={topics[3]}
                              title={topics_text[3]}
                              show_image_options={show_image_controls}
                              select_updated_namespace={appNamespace + '/set_topic_4'}
                              streamingImageQuality={streamingImageQuality}
                              image_filters={image_filters}
                            show_selector={show_selectors}
                            show_buttons={show_selectors}
                            />
                          </div>          
                        : null
                        }
                  </div>
   
          </div> 

            <NepiIFConfig
                    namespace={appNamespace}
                    title={"Nepi_IF_Config"}
                  />

            <NepiIFSaveData
                    namespace={appNamespace + "/save_data"}
                    title={"Nepi_IF_SaveData"}
                  />
      </div>
      )
    }
  }


}

export default ImageViewerApp
