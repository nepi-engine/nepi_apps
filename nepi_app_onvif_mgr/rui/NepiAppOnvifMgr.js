/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
import React, { Component, useEffect } from 'react';
import { observer, inject } from "mobx-react"

import Toggle from "react-toggle"
import Section from "./Section"
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Input from "./Input"
import Button, { ButtonMenu } from "./Button"
import ListBox from './ListBox';
import './ListBox.css';
import Select, { Option } from "./Select"
import BooleanIndicator from "./BooleanIndicator"
import Styles from "./Styles"


@inject("ros")
@observer
class OnvifMgr extends Component {
  onvifDeviceStatuses = []
  onvifDeviceConfigs = []
  onvifIDXDeviceDrivers = []
  onvifPTXDeviceDrivers = []

  constructor(props) {
    super(props);

    this.state = {
      selectedDeviceUUID: null,
      
      selectedDeviceConfigModified: false,

      selectedDeviceConfigUUID: '',
      selectedDeviceConfigDevicename: '',
      selectedDeviceConfigUsername: '',
      selectedDeviceConfigPassword: '',
      selectedDeviceConfigBasename: '',
      selectedDeviceConfigIDXEnabled: true,
      selectedDeviceConfigPTXEnabled: false,
      selectedDeviceConfigIDXDriver: '',
      selectedDeviceConfigPTXDriver: '',
      
      needs_update: false,
      app_connected: false

    };

    this.handleUUIDSelection = this.handleUUIDSelection.bind(this)
    this.handleNewConfigClick = this.handleNewConfigClick.bind(this)
    this.handleUpdateConfigClick = this.handleUpdateConfigClick.bind(this)
    this.handleDeleteConfigClick = this.handleDeleteConfigClick.bind(this)
    this.onChangeTextField = this.onChangeTextField.bind(this)
    this.onIDXDriverSelected = this.onIDXDriverSelected.bind(this)
    this.onPTXDriverSelected = this.onPTXDriverSelected.bind(this)
    this.createIDXDriverOptions = this.createIDXDriverOptions.bind(this)
    this.createPTXDriverOptions = this.createPTXDriverOptions.bind(this)


    this.callOnvifDeviceListQueryService = this.callOnvifDeviceListQueryService.bind(this)
    this.callOnvifDriverListQueryService = this.callOnvifDriverListQueryService.bind(this)

    this.onOnvifDeviceCfgUpdate = this.onOnvifDeviceCfgUpdate.bind(this)
    this.onOnvifDeviceCfgDelete = this.onOnvifDeviceCfgDelete.bind(this)

    // onvif mgr services
    //this.callOnvifDeviceListQueryService(true) // Start it polling
    //this.callOnvifDriverListQueryService(true) // Start it polling
  }


  componentDidMount(){
    // onvif mgr services
    this.callOnvifDeviceListQueryService(true) // Start it polling
    this.callOnvifDriverListQueryService(true) // Start it polling
    this.setState({needs_upadate: true, app_connected: false})
    
  }

  async callOnvifDeviceListQueryService(poll = true) {
    const _pollOnce = async () => {
        const {topicTypes} = this.props.ros
        const status_msg = "nepi_app_onvif_mgr/OnvifStatus"
        const ready = topicTypes.indexOf(status_msg) !== -1
        if (ready === true){
          const resp = await this.props.ros.callService({
            name: "app_onvif_mgr/device_list_query",
            messageType: "nepi_interfaces/OnvifDeviceListQuery",
          })

          this.onvifDeviceStatuses = resp['device_statuses']
          this.onvifDeviceConfigs = resp['device_cfgs']
        }

        const nu = this.state.needs_update
        this.setState({needs_update: !nu,app_connected: true})

        if (poll) {
          setTimeout(_pollOnce, 1000) // Slow because it doesn't really change
        }
    }
    
    _pollOnce()
  }

  async callOnvifDriverListQueryService(poll = true) {
    const _pollOnce = async () => {
      const {topicTypes} = this.props.ros
      const status_msg = "nepi_app_onvif_mgr/OnvifStatus"
      const ready = topicTypes.indexOf(status_msg) !== -1
      if (ready === true){
        const resp = await this.props.ros.callService({
          name: "app_onvif_mgr/driver_list_query",
          messageType: "nepi_interfaces/OnvifDriverListQuery"
        })

        this.onvifIDXDeviceDrivers = resp['idx_drivers']
        this.onvifPTXDeviceDrivers = resp['ptx_drivers']
      }
        const nu = this.state.needs_update
        this.setState({needs_update: !nu,app_connected: true})

        if (poll) {
          setTimeout(_pollOnce, 5000) // Slow because it doesn't really change
        }
    }
    
    _pollOnce()
  }




   async onOnvifDeviceCfgUpdate(updatedDeviceCfg) {
    await this.props.ros.callService({
      name: "app_onvif_mgr/set_device_cfg",
      messageType: "nepi_interfaces/OnvifDeviceCfgUpdate",
      args: {cfg : updatedDeviceCfg}
    })
  }


  async onOnvifDeviceCfgDelete(uuid) {
    await this.props.ros.callService({
      name: "app_onvif_mgr/delete_device_cfg",
      messageType: "nepi_interfaces/OnvifDeviceCfgDelete",
      args: {device_uuid : uuid}
    })
  }


  handleUUIDSelection(item) {
    //const { onvifDeviceConfigs, onvifIDXDeviceDrivers, onvifPTXDeviceDrivers } = this.props.ros;
    let selectedConfig = null
    var uuid = ''
    const statuses = this.onvifDeviceStatuses
    for (let i = 0; i < statuses.length; i++) {
      const status = statuses[i]
      const devname = status.device_name
      if (devname === item) {
        uuid = status.uuid
        break
      }
    }

    const configs = this.onvifDeviceConfigs
    for (let i = 0; i < configs.length; i++) {
      const config = configs[i]
      const devname = config.device_name
      if (devname === item) {
        selectedConfig = config
        break
      }
    }

    const defaultIDXDeviceDriver = (this.onvifIDXDeviceDrivers.length > 0)? this.onvifIDXDeviceDrivers[0] : ''
    const defaultPTXDeviceDriver = (this.onvifPTXDeviceDrivers.length > 0)? this.onvifPTXDeviceDrivers[0] : ''

    this.setState({ 
      selectedDeviceUUID: selectedConfig? selectedConfig.uuid : uuid,

      selectedDeviceConfigModified: false,
      selectedDeviceConfigUUID: selectedConfig? selectedConfig.uuid : '',
      selectedDeviceConfigDevicename: selectedConfig? selectedConfig.device_name : '',
      selectedDeviceConfigUsername: selectedConfig? selectedConfig.username : '',
      selectedDeviceConfigPassword: selectedConfig? selectedConfig.password : '',
      selectedDeviceConfigBasename: selectedConfig? selectedConfig.node_base_name : '',
      selectedDeviceConfigIDXEnabled: selectedConfig? selectedConfig.idx_enabled : true,
      selectedDeviceConfigPTXEnabled: selectedConfig? selectedConfig.ptx_enabled : false,
      selectedDeviceConfigIDXDriver: selectedConfig? selectedConfig.idx_driver : defaultIDXDeviceDriver,
      selectedDeviceConfigPTXDriver: selectedConfig? selectedConfig.ptx_driver : defaultPTXDeviceDriver,
    });
  };

  handleNewConfigClick() {
    this.setState({
      selectedDeviceUUID: null,
      selectedDeviceConfigModified: true,
      selectedDeviceConfigUUID: '',
      selectedDeviceConfigDeviceName: '',
      selectedDeviceConfigUsername: '',
      selectedDeviceConfigPassword: '',
      selectedDeviceConfigBasename: 'new_onvif_device',
      selectedDeviceIDXEnabled: false,
      selectedDevicePTXEnabled: false,
      selectedDeviceConfigIDXDriver: this.onvifIDXDeviceDrivers[0],
      selectedDeviceConfigPTXDriver: this.onvifPTXDeviceDrivers[0]
    })
  }

  async handleUpdateConfigClick(uuid) {
    let updated_config = {
      uuid : uuid,
      device_name : this.state.selectedDeviceConfigDevicename,
      username : this.state.selectedDeviceConfigUsername,
      password : this.state.selectedDeviceConfigPassword,
      node_base_name : this.state.selectedDeviceConfigBasename,
      idx_enabled : this.state.selectedDeviceConfigIDXEnabled,
      ptx_enabled : this.state.selectedDeviceConfigPTXEnabled,
      idx_driver : this.state.selectedDeviceConfigIDXDriver,
      ptx_driver : this.state.selectedDeviceConfigPTXDriver
    }
    this.onOnvifDeviceCfgUpdate(updated_config)
    this.setState({selectedDeviceConfigModified: false})
  }

  async handleDeleteConfigClick() {
    this.onOnvifDeviceCfgDelete(this.state.selectedDeviceConfigUUID)

    this.setState({
      selectedDeviceConfigUUID: '',
      selectedDeviceConfigDevicename: '',
      selectedDeviceConfigUsername: '',
      selectedDeviceConfigPassword: '',
      selectedDeviceConfigBasename: '',
      selectedDeviceConfigIDXEnabled: true,
      selectedDeviceConfigPTXEnabled: false,
      selectedDeviceConfigIDXDriver: '',
      selectedDeviceConfigPTXDriver: ''
    })
  };

  onChangeTextField(e) {
    if (e.target.id === "onvif_uuid_textbox")  {
      this.setState({selectedDeviceConfigUUID : e.target.value})
    }
    else if (e.target.id === "onvif_username_textbox") {
      this.setState({selectedDeviceConfigUsername : e.target.value})
    }
    else if (e.target.id === "onvif_password_textbox") {
      this.setState({selectedDeviceConfigPassword : e.target.value})
    }
    else if (e.target.id === "node_base_name_texbox") {
      this.setState({selectedDeviceConfigBasename : e.target.value})
    }
    
    // And in all cases, set the config-modified flag
    this.setState({selectedDeviceConfigModified : true})
  }

  onIDXDriverSelected(event) {
    if (event.target.value !== this.state.selectedDeviceConfigIDXDriver) {
      this.setState({ 
        selectedDeviceConfigIDXDriver:event.target.value,
        selectedDeviceConfigModified : true
      })
    }
  }

  onPTXDriverSelected(event) {
    if (event.target.value !== this.state.selectedDeviceConfigPTXDriver) {
      this.setState({ 
        selectedDeviceConfigPTXDriver:event.target.value,
        selectedDeviceConfigModified : true
      })
    }
  }

  createIDXDriverOptions() {
    //const { onvifIDXDeviceDrivers } = this.props.ros;
    var items = []
    for (var i = 0; i < this.onvifIDXDeviceDrivers.length; i++) {
      if (this.onvifIDXDeviceDrivers[i].indexOf("Generic") !== -1){
        items.push(<Option value={this.onvifIDXDeviceDrivers[i]}>{this.onvifIDXDeviceDrivers[i]}</Option>)
      }
    }
    for (var i2 = 0; i2 < this.onvifIDXDeviceDrivers.length; i2++) {
      if (this.onvifIDXDeviceDrivers[i2].indexOf("Generic") === -1){
        items.push(<Option value={this.onvifIDXDeviceDrivers[i2]}>{this.onvifIDXDeviceDrivers[i2]}</Option>)
      }
    }
    return items
  }

  createPTXDriverOptions() {
    //const { onvifPTXDeviceDrivers } = this.props.ros;
    var items = []
    for (var i = 0; i < this.onvifPTXDeviceDrivers.length; i++) {
      if (this.onvifPTXDeviceDrivers[i].indexOf("Generic") !== -1){
        items.push(<Option value={this.onvifPTXDeviceDrivers[i]}>{this.onvifPTXDeviceDrivers[i]}</Option>)
      }
    }
    for (var i2 = 0; i2 < this.onvifPTXDeviceDrivers.length; i2++) {
      if (this.onvifPTXDeviceDrivers[i2].indexOf("Generic") === -1){
        items.push(<Option value={this.onvifPTXDeviceDrivers[i2]}>{this.onvifPTXDeviceDrivers[i2]}</Option>)
      }
    }
    return items
  }

  render() {
    //const { onvifDeviceStatuses, onvifDeviceConfigs } = this.props.ros;
    const { selectedDeviceUUID, 
            selectedDeviceConfigDevName,
            selectedDeviceConfigModified,
            selectedDeviceConfigUUID,
            selectedDeviceConfigUsername,
            selectedDeviceConfigPassword,
            selectedDeviceConfigBasename,
            selectedDeviceConfigIDXEnabled,
            selectedDeviceConfigPTXEnabled
          } = this.state

    let selectedDeviceStatus = null
        
    let detectedDeviceUUIDsForListBox = []
    let detectedDeviceDevNamesForListBox = []
    const needs_update = this.state.needs_upadate
    const app_connected = this.state.app_connected
    const onvifDeviceStatuses = this.onvifDeviceStatuses
    if (app_connected === false || onvifDeviceStatuses.length == 0){
      detectedDeviceDevNamesForListBox.push("Application Loading")
    }
    else if (onvifDeviceStatuses.length > 0) {
      const searching = (onvifDeviceStatuses[0].uuid === "")
      if (searching === true){
        detectedDeviceDevNamesForListBox.push("Searching for Devices")
      }
      else{
        for (let i = 0; i < onvifDeviceStatuses.length; i++) {
          const status = onvifDeviceStatuses[i]
          const uuid = status.uuid
          const devname = status.device_name
          detectedDeviceUUIDsForListBox.push(uuid)
          detectedDeviceDevNamesForListBox.push(devname)
          if ((selectedDeviceUUID !== null) && (uuid === selectedDeviceUUID)) {
            selectedDeviceStatus = status
          }
        }
      }
    }

    let configuredDevicesUUIDsForListBox = []
    let configuredDevicesDevNamesForListBox = []
    if (this.onvifDeviceConfigs !== null) {
      for (let i = 0; i < this.onvifDeviceConfigs.length; i++) {
        const config = this.onvifDeviceConfigs[i]
        const uuid = config.uuid
        const devname = config.device_name
        configuredDevicesUUIDsForListBox.push(uuid)
        configuredDevicesDevNamesForListBox.push(devname)
      }
    }

    let config_text_color = (selectedDeviceConfigModified)? Styles.vars.colors.red : Styles.vars.colors.black
    let config_text_weight = (selectedDeviceConfigModified)? "bold" : "normal"

    let uuid_for_config_text_field = selectedDeviceUUID
     
    
    return (



      <Columns>
        <Column>
          <Section title={"Detected Devices"}>

            <ListBox 
              id="detectedDevicesListBox" 
              items={detectedDeviceDevNamesForListBox} 
              selectedItem={this.state.selectedDeviceConfigDevicename} 
              onSelect={this.handleUUIDSelection} 
              style={{ color: 'black', backgroundColor: 'white' }}
            />
          </Section>
        </Column>
        <Column>
          <Section title={"Configured Devices"}>
            <ListBox 
              id="runningScriptsListBox" 
              items={configuredDevicesDevNamesForListBox} 
              selectedItem={this.state.selectedDeviceConfigDevicename}
              onSelect={this.handleUUIDSelection} 
              style={{ color: 'black', backgroundColor: 'white' }} 
            />
          </Section>
        </Column>
        <Column equalWidth={false}>
          <Section title={selectedDeviceConfigDevName? selectedDeviceConfigDevName : ''}>
            <label style={{fontWeight: 'bold'}}>
              {"Status"}
            </label>
            <Columns>
              <Column>
                <Label title={"Host/IP"}>
                  <Input 
                    disabled 
                    value={selectedDeviceStatus? selectedDeviceStatus.host : ''} 
                    style={{width: '100%'}} 
                  />
                </Label>

                <Label title={"Hardware ID"}>
                  <Input 
                    disabled 
                    value={selectedDeviceStatus? selectedDeviceStatus.hardware_id : ''} 
                    style={{width: '100%'}} 
                  />
                </Label>

                <Label title={"Device Connected"}>
                  <BooleanIndicator value={selectedDeviceStatus? selectedDeviceStatus.connectable : ''} />
                </Label>

                <Label title={"Camera Connected"}>
                  <BooleanIndicator value={selectedDeviceStatus? selectedDeviceStatus.idx_node_running : ''} />
                </Label>

                <Label title={"PanTilt Connected"}>
                  <BooleanIndicator value={selectedDeviceStatus? selectedDeviceStatus.ptx_node_running : ''} />
                </Label>

              </Column>
              <Column>
                <Label title={"Port"}>
                  <Input 
                    disabled 
                    value={selectedDeviceStatus? selectedDeviceStatus.port : ''} 
                    style={{width: '100%'}} 
                  />
                </Label>

                <Label title={"Firmware"}>
                  <Input 
                    disabled 
                    value={selectedDeviceStatus? selectedDeviceStatus.firmware_version : ''} 
                    style={{width: '100%'}} 
                  />
                </Label>

              </Column>
            </Columns>
            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
            <label style={{fontWeight: 'bold'}}>
              {"Configuration"}
            </label>
            <Label title={"UUID"}>
              <Input
                id={'onvif_uuid_textbox'} 
                value={uuid_for_config_text_field} 
                style={{width: '100%', color: config_text_color, fontWeight: config_text_weight}}
                onChange={this.onChangeTextField}
                disabled={(detectedDeviceUUIDsForListBox.includes(uuid_for_config_text_field))} 
              />
            </Label>
            <Label title={"Device Name (User Defined)"}>
              <Input
                id={'node_base_name_texbox'} 
                value={selectedDeviceConfigBasename} 
                style={{width: '100%', color: config_text_color, fontWeight: config_text_weight}}
                onChange={this.onChangeTextField}
              />
            </Label>
            <Label title={"Login: Username"}>
              <Input
                id={'onvif_username_textbox'} 
                value={selectedDeviceConfigUsername} 
                style={{width: '100%', color: config_text_color, fontWeight: config_text_weight}}
                onChange={this.onChangeTextField}
              />
            </Label>
            <Label title={"Login: Password"}>
              <Input
                id={'onvif_password_textbox'} 
                value={selectedDeviceConfigPassword} 
                style={{width: '100%', color: config_text_color, fontWeight: config_text_weight}}
                onChange={this.onChangeTextField}
              />
            </Label>
            <Columns>
              <Column>
                <Label title={"Enable Camera"}>
                  <Toggle
                    id={'onvif_idx_enabled_toggle'} 
                    checked={selectedDeviceConfigIDXEnabled} 
                    onClick={() => { this.setState({selectedDeviceConfigIDXEnabled : !this.state.selectedDeviceConfigIDXEnabled,
                                                    selectedDeviceConfigModified : true}); }
                            }
                  />
                </Label>
                <Label title={"Enable PanTilt"}>
                  <Toggle
                    id={'onvif_ptx_enabled_toggle'} 
                    checked={selectedDeviceConfigPTXEnabled} 
                    onClick={() => { this.setState({selectedDeviceConfigPTXEnabled : !this.state.selectedDeviceConfigPTXEnabled,
                                                    selectedDeviceConfigModified : true}); }
                            } 
                  />
                </Label>
              </Column>
              <Column>
                <Label title={"Camera Driver"}>
                  <Select
                    onChange={this.onIDXDriverSelected}
                    value={this.state.selectedDeviceConfigIDXDriver}
                  >
                    {this.createIDXDriverOptions()}
                  </Select>
                </Label>
                <Label title={"PanTilt Driver"}>
                  <Select
                    onChange={this.onPTXDriverSelected}
                    value={this.state.selectedDeviceConfigPTXDriver}
                  >
                    {this.createPTXDriverOptions()}
                  </Select>
                </Label>                
              </Column>
            </Columns>


            <ButtonMenu>

            {/*
              <Button
                id="new_config_button"
                onClick={this.handleNewConfigClick}
              >
                {"New Config"}
              </Button>
            */}

              <Button
                id="delete_config_button"
                onClick={this.handleDeleteConfigClick}
                hidden={!(configuredDevicesUUIDsForListBox.includes(selectedDeviceUUID))}
              >
                {"Delete Config"}
              </Button>
              <Button
                id="create_config_button"
                onClick={() => {this.handleUpdateConfigClick(uuid_for_config_text_field)}} 
                style={{color: config_text_color}}
                hidden={!selectedDeviceConfigModified}
              >
                {"Apply Changes"}
              </Button>
            </ButtonMenu>
          </Section>
        </Column>
      </Columns>
    )
  }
};

export default OnvifMgr
