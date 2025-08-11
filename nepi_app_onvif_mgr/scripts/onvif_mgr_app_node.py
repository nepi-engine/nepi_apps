#!/usr/bin/env python
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
import os
import rospy
import sys
import subprocess
import time
from wsdiscovery.discovery import ThreadedWSDiscovery as WSDiscovery
import urllib.parse

import datetime
from datetime import timezone
import requests
from xml.etree import ElementTree as ET

# DON'T USE SaveCfgIF IN THIS CLASS -- SEE WARNING BELOW
#from nepi_api.sys_if_save_cfg import SaveCfgIF

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_drvs

from std_msgs.msg import String, Bool
from std_srvs.srv import Empty, EmptyResponse
from nepi_interfaces.msg import MgrDriversStatus
from nepi_app_onvif_mgr.msg import OnvifStatus, OnvifDeviceCfg, OnvifDeviceStatus
from nepi_app_onvif_mgr.srv import OnvifDeviceListQuery, OnvifDeviceListQueryRequest, OnvifDeviceListQueryResponse
from nepi_app_onvif_mgr.srv import OnvifDeviceCfgUpdate, OnvifDeviceCfgUpdateRequest, OnvifDeviceCfgUpdateResponse
from nepi_app_onvif_mgr.srv import OnvifDeviceCfgDelete, OnvifDeviceCfgDeleteRequest, OnvifDeviceCfgDeleteResponse
from nepi_app_onvif_mgr.srv import OnvifDriverListQuery, OnvifDriverListQueryRequest, OnvifDriverListQueryResponse

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF

MGR_NAME = 'ONVIF Manager' # Use in display menus
FILE_TYPE = 'MANAGER'

class ONVIFMgr:

  NODE_LAUNCH_TIME_SEC = 20  # Will not check node status before

  DEFAULT_NEPI_CONFIG_PATH = "/opt/nepi/engine/etc"
  WSDL_FOLDER = os.path.join(DEFAULT_NEPI_CONFIG_PATH, "onvif/wsdl/")

 
  DEFAULT_DISCOVERY_INTERVAL_SEC = 5.0
  
  ONVIF_SCOPE_NVT_ID = 'Network_Video_Transmitter'
  ONVIF_SCOPE_NVT_ALT_ID = 'NetworkVideoTransmitter' # ONVIF spec. says this name is legal for NVT, too
  ONVIF_SCOPE_PTZ_ID = 'ptz'

  node_if = None
  status_msg = OnvifStatus()

  discovery_interval = DEFAULT_DISCOVERY_INTERVAL_SEC
  autosave_cfg_changes = True
  clear_discovery = False
  configured_onvifs = dict()

  device_name_dict = dict()
  drvs_dict = dict()
  drivers_files = []
  active_drivers_list = []
  idx_drivers_dict = []
  ptx_drivers_dict = []

  
  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "onvif_app" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
    self.class_name = type(self).__name__
    self.base_namespace = nepi_sdk.get_base_namespace()
    self.node_name = nepi_sdk.get_node_name()
    self.node_namespace = nepi_sdk.get_node_namespace()

    ##############################  
    # Create Msg Class
    self.msg_if = MsgIF(log_name = self.class_name)
    self.msg_if.pub_info("Starting IF Initialization Processes")


    ##############################
    # Get for System Folders
    self.msg_if.pub_info("Waiting for system folders")
    system_folders = nepi_system.get_system_folders(log_name_list = [self.node_name])
    #self.msg_if.pub_warn("Got system folders: " + str(system_folders))

    self.drivers_folder = system_folders['drivers_lib']
    self.msg_if.pub_info("Using Drivers Folder: " + str(self.drivers_folder))

    self.drvs_dict = nepi_drvs.getDriversDict(self.drivers_folder)
    self.msg_if.pub_info("Init Drivers Dict keys " + str(self.drvs_dict.keys()))
    self.drivers_files = nepi_drvs.getDriverFilesList(self.drivers_folder)
    
    ############
    # start onvif discovery process
    self.wsd = WSDiscovery()
    self.wsd.start()
    detected_services = self.wsd.searchServices(timeout=1)
    
        
    ##############################
    # Initialize Class Variables
    drivers_mgr_status = nepi_sdk.create_namespace(self.base_namespace,'drivers_mgr/status')

    self.detected_onvifs = {}
    
    self.msg_if.pub_info("Starting device dict from param server: " + str(self.configured_onvifs) )
    # Get drv drivers database
    drvs_dict = nepi_drvs.getDriversDict(self.drivers_folder)
    self.drivers_files = nepi_drvs.getDriverFilesList(self.drivers_folder)



    self.initCb(do_updates = False)
    ##############################
    ### Setup Node


    #### WARNING ####
    # Do not use topics in this class, only services... somehow just the execution of any subscriber
    # callback breaks the ONVIF discovery mechanism, even if the callback does nothing at all and 
    # even if the WSDiscovery object is destroyed and recreated afterwards. Very weird, but very repeatable.
    
    # This WARNING extends to topics that are part of an included interface (e.g., SaveCfgIF) -- 
    # can't use those interfaces, instead must manage config. file saving ourselves!
    # self.save_cfg_if = SaveCfgIF(initCb=self.setCurrentSettingsAsDefault, resetCb=self.resetCb)
    #### END WARNING ####


    # Configs Config Dict ####################
    self.CFGS_DICT = {
        'init_callback': self.initCb,
        'reset_callback': self.resetCb,
        'factory_reset_callback': self.factoryResetCb,
        'init_configs': True,
        'namespace': self.node_namespace
    }

    # Params Config Dict ####################
    self.PARAMS_DICT = {
        'discovery_interval': {
            'namespace': self.node_namespace,
            'factory_val':  self.discovery_interval
        },
        'autosave_cfg_changes': {
            'namespace': self.node_namespace,
            'factory_val':  self.autosave_cfg_changes
        },
        'clear_discovery': {
            'namespace': self.node_namespace,
            'factory_val':  self.clear_discovery
        },
        'configured_onvifs': {
            'namespace': self.node_namespace,
            'factory_val':  self.configured_onvifs
        },

    }



    # Services Config Dict ####################
    self.SRVS_DICT = {
        'set_device_cfg': {
            'namespace': self.node_namespace,
            'topic': 'set_device_cfg',
            'srv': OnvifDeviceCfgUpdate,
            'req': OnvifDeviceCfgUpdateRequest(),
            'resp': OnvifDeviceCfgUpdateResponse(),
            'callback': self.updateDeviceCfgHandler
        },
        'delete_device_cfg': {
            'namespace': self.node_namespace,
            'topic': 'delete_device_cfg',
            'srv': OnvifDeviceCfgDelete,
            'req': OnvifDeviceCfgDeleteRequest(),
            'resp': OnvifDeviceCfgDeleteResponse(),
            'callback': self.deleteDeviceCfgHandler
        },
        'device_list_query': {
            'namespace': self.node_namespace,
            'topic': 'device_list_query',
            'srv': OnvifDeviceListQuery,
            'req': OnvifDeviceListQueryRequest(),
            'resp': OnvifDeviceListQueryResponse(),
            'callback': self.provideDeviceList
        },
        'resync_onvif_device_clocks': {
            'namespace': self.node_namespace,
            'topic': 'resync_onvif_device_clocks',
            'srv': Empty,
            'req': Empty(),
            'resp': Empty(),
            'callback': self.resyncOnvifDeviceClocks
        },
        'driver_list_query': {
            'namespace': self.node_namespace,
            'topic': 'driver_list_query',
            'srv': OnvifDriverListQuery,
            'req': OnvifDriverListQueryRequest(),
            'resp': OnvifDriverListQueryResponse(),
            'callback': self.provideDeviceDriverList
        }
    }

    # Publishers Config Dict ####################
    self.PUBS_DICT = {
        'status_pub': {
            'namespace': self.node_namespace,
            'topic': 'status',
            'msg': OnvifStatus,
            'qsize': 1,
            'latch': True
        }
    }  


    # Subscribers Config Dict ####################
    self.SUBS_DICT = {
        'allow_discovery_clearing': {
            'namespace': self.node_namespace,
            'topic': 'allow_discovery_clearing',
            'msg': Bool,
            'qsize': 10,
            'callback': self.clearDiscCb, 
            'callback_args': ()
        },
        'drivers_status': {
            'namespace': drivers_mgr_status,
            'topic': '',
            'msg': MgrDriversStatus,
            'qsize': 10,
            'callback': self.driversStatusCb, 
            'callback_args': ()
        }
    }




    # Create Node Class ####################
    self.node_if = NodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                      msg_if = self.msg_if
    )

    nepi_sdk.wait()

    ###########################
    # Initialize Params

    self.initCb(do_updates = True)

    ###########################
        

    ########################
    # Complete init processes

    # Iterate through these configured devices fixing invalid characters in the node base name
    for uuid in self.configured_onvifs:
      self.configured_onvifs[uuid]['node_base_name'] = self.configured_onvifs[uuid]['node_base_name'].replace(' ','_').replace('-','_')

    # Must handle our own store params rather than offloading to SaveCfgIF per WARNING above
    self.store_params_publisher = rospy.Publisher('store_params', String, queue_size=1)

    nepi_sdk.start_timer_process(self.discovery_interval, self.runDiscovery, oneshot=True)
    nepi_sdk.start_timer_process(1, self.statusPublishCb)
    #########################################################
    ## Initiation Complete
    self.msg_if.pub_info("Initialization Complete")
    #Set up node shutdown
    nepi_sdk.on_shutdown(self.cleanup_actions)
    # Spin forever (until object is detected)
    nepi_sdk.spin()
    #########################################################


  def initCb(self,do_updates = False):
      if self.node_if is not None:
        self.discovery_interval = self.node_if.get_param('discovery_interval')
        self.autosave_cfg_changes = self.node_if.get_param('autosave_cfg_changes')
        self.clear_discovery = self.node_if.get_param('clear_discovery')
        self.configured_onvifs = self.node_if.get_param('configured_onvifs')
      if do_updates == True:
        pass
      self.publish_status()
        

  def resetCb(self,do_updates = True):
      self.msg_if.pub_warn("Reseting")
      if self.node_if is not None:
        pass
      if do_updates == True:
        pass
      self.initCb(do_updates = do_updates)


  def factoryResetCb(self,do_updates = True):
      self.msg_if.pub_warn("Factory Reseting")
      if self.node_if is not None:
        pass
      if do_updates == True:
        pass
      self.initCb(do_updates = do_updates)


  def statusPublishCb(self,timer):
      self.publish_status()

  def publish_status(self):
      if self.node_if is not None:
        self.node_if.publish_pub('status_pub',self.status_msg)


  def clearDiscCb(self,msg):
    clear = msg.data
    self.clear_discovery = clear
    if self.node_if is not None:
          self.node_if.set_param('clear_discovery',clear)
    if self.autosave_cfg_changes is True:
      self.msg_if.pub_info('Auto-saving updated config')
      self.setCurrentSettingsAsDefault()
      self.store_params_publisher.publish(self.node_namespace)

  def driversStatusCb(self,msg):
    # First check if drv driver database needs updating
    drvs_dict = self.drvs_dict
    drivers_files = nepi_drvs.getDriverFilesList(self.drivers_folder)
    need_update = self.drivers_files != drivers_files
    if need_update:
      self.msg_if.pub_info("Need to Update Drv Database")
      drvs_dict = nepi_drvs.refreshDriversDict(self.drivers_folder,drvs_dict)
    #self.msg_if.pub_warn("Drivers Dict Keys: " + str(drvs_dict.keys()))
    #ln = sys._getframe().f_lineno ; self.printND('Info',ln)
    self.drivers_files = drivers_files
    # Next update available drivers base on active drivers
    active_drivers_list = msg.active_pkg_list
    if self.active_drivers_list != active_drivers_list:
      self.msg_if.pub_warn("Got Active Drivers Update list: " + str(active_drivers_list))
    for drv_name in drvs_dict.keys():
      active = False
      if drv_name in active_drivers_list:
        active = True
      drvs_dict[drv_name]['active'] = active
    self.drvs_dict = drvs_dict
    active_drvs_dict = nepi_drvs.getDriversByActive(self.drvs_dict)
    #self.msg_if.pub_warn("Active Drivers Dict Keys: " + str(active_drvs_dict.keys()))
    idx_drivers_dict = dict()
    ptx_drivers_dict = dict()
    for drv_name in active_drvs_dict.keys():
      drv_dict = active_drvs_dict[drv_name]
      type = drv_dict['type']
      group_id = drv_dict['group_id']
      if type == "IDX" and group_id == "ONVIF":
        idx_drivers_dict[drv_name] = drv_dict
      elif type == "PTX" and group_id == "ONVIF":
        ptx_drivers_dict[drv_name] = drv_dict
    self.idx_drivers_dict = idx_drivers_dict
    self.ptx_drivers_dict = ptx_drivers_dict
    self.active_drivers_list = active_drivers_list
    #self.msg_if.pub_warn("IDX Drivers Dict: " + str(idx_drivers_dict.keys()))
    #self.msg_if.pub_warn("PTX Drivers Dict: " + str(ptx_drivers_dict.keys()))



  def updateDeviceCfgHandler(self, req):
    self.msg_if.pub_info('Received device update req ' + str(req))
    device_cfg = req.cfg
    uuid = device_cfg.uuid.upper()
    
    if uuid in self.detected_onvifs:
      detected_device = self.detected_onvifs[uuid]
    else:
      self.msg_if.pub_info('Setting configuration for as-yet undetected device ' + str(uuid))
      detected_device = None

    # Make sure the node_base_name is legal ROS
    legal_base_name = device_cfg.node_base_name.replace(' ', '_').replace('-','_')

    # Make sure the specified drivers are known
    if device_cfg.idx_driver == "":
        idx_drv_dict = dict()
    else:
      if device_cfg.idx_driver not in self.idx_drivers_dict.keys():
        self.msg_if.pub_warn('Unknown idx driver(s) specified ( ' + str(device_cfg.idx_driver) + ' )... will not update config')
        return OnvifDeviceCfgUpdateResponse(success = False)
      else:
        idx_drv_dict = self.idx_drivers_dict[device_cfg.idx_driver]

    if device_cfg.ptx_driver == "":
        ptx_drv_dict = dict()
    else:
      if device_cfg.ptx_driver not in self.ptx_drivers_dict.keys():
        self.msg_if.pub_warn('Unknown ptx driver(s) specified ( ' + str(device_cfg.ptx_driver) + ' )... will not update config')
        return OnvifDeviceCfgUpdateResponse(success = False)
      else:
        ptx_drv_dict = self.ptx_drivers_dict[device_cfg.ptx_driver]        

    updated_cfg = {
      'device_name' : device_cfg.device_name,
      'username' : device_cfg.username,
      'password' : device_cfg.password,
      'node_base_name' : legal_base_name,
      'idx_enabled' : device_cfg.idx_enabled,
      'ptx_enabled' : device_cfg.ptx_enabled,
      'idx_driver' : device_cfg.idx_driver,
      'idx_drv_dict': idx_drv_dict,
      'ptx_driver' : device_cfg.ptx_driver,
      'ptx_drv_dict': ptx_drv_dict
    }
    self.configured_onvifs[uuid] = updated_cfg
    if self.node_if is not None:
      self.node_if.set_param('configured_onvifs', self.configured_onvifs)
    # Now remove the device from the list of detected devices so that it can be rediscovered and properly connected and nodes launched
    # using this updated config. If nodes are already running, stop them (to restart as newly-configured) and warn user
    if detected_device:
      if (detected_device['idx_subproc'] is not None) or (detected_device['ptx_subproc'] is not None):
          self.msg_if.pub_warn('Config. for ' + str(uuid) + ' is updated, will restart any already-running nodes')
          self.stopAndPurgeNodes(uuid)
      
      self.detected_onvifs.pop(uuid)

      # Must handle our own saving since topics don't work in this class (see WARNING above)
      if self.autosave_cfg_changes is True:
        self.msg_if.pub_info('Auto-saving updated config')
        self.setCurrentSettingsAsDefault()
        self.store_params_publisher.publish(nepi_sdk.get_node_namespace())

    return OnvifDeviceCfgUpdateResponse(success = True)

  def deleteDeviceCfgHandler(self, req):
    self.msg_if.pub_info('Received device delete req ' + str(req))
    uuid = req.device_uuid.upper()
    self.msg_if.pub_info('Checking for uuid: ' + str(uuid) + ' in keys ' + str(self.configured_onvifs.keys()))
    if uuid not in self.configured_onvifs:
      self.msg_if.pub_warn("Device is not configured... ignoring request to delete configuration")
      return OnvifDeviceCfgDeleteResponse(success = False)
    
    self.configured_onvifs.pop(uuid)
    if self.node_if is not None:
      self.node_if.set_param('configured_onvifs', self.configured_onvifs)
    # And clean it up if this device is currently detected and running
    if uuid in self.detected_onvifs:
      self.stopAndPurgeNodes(uuid)
      self.detected_onvifs.pop(uuid)
      self.device_name_dict.pop(uuid)

    # Must handle our own saving since topics don't work in this class (see WARNING above)
    if self.autosave_cfg_changes is True:
      self.msg_if.pub_info('Auto-saving deleted config')
      self.setCurrentSettingsAsDefault()
      self.store_params_publisher.publish(self.node_namespace)

    return True

  def provideDeviceList(self, req):
    #self.msg_if.pub_warn('Got Device List Request: ' + str(req))
    resp = OnvifDeviceListQueryResponse()
    # Statuses of detected devices
    device_name = ''
    if len(self.detected_onvifs.keys()) == 0:
      resp_status_for_device = OnvifDeviceStatus()
      resp_status_for_device.uuid = ""
      resp_status_for_device.device_name = device_name
      resp_status_for_device.manufacturer = ""
      resp_status_for_device.model = ""
      resp_status_for_device.firmware_version = ""
      resp_status_for_device.hardware_id = ""
      resp_status_for_device.host = ""
      resp_status_for_device.port = 0
      resp_status_for_device.connectable = False
      resp_status_for_device.idx_node_running = False
      resp_status_for_device.ptx_node_running = False
      resp.device_statuses.append(resp_status_for_device)
    else:
      for uuid in self.detected_onvifs:
        device = self.detected_onvifs[uuid]
        if uuid in self.device_name_dict.keys():
            device_name = self.device_name_dict[uuid]
        else:
            device_name = str(device['host']) + ":" + str(device['port'])
        resp_status_for_device = OnvifDeviceStatus()
        resp_status_for_device.uuid = uuid
        resp_status_for_device.device_name = device_name

        
        # Manufacturer settings
        resp_status_for_device.manufacturer = device['manufacturer']
        resp_status_for_device.model = device['model']
        resp_status_for_device.firmware_version = device['firmware_version']
        resp_status_for_device.hardware_id = device['hardware_id']

        # Network settings
        resp_status_for_device.host = device['host']
        resp_status_for_device.port = device['port']
        resp_status_for_device.connectable = device['connectable']
        
        idx_running = False
        if device['idx_subproc'] is not None and device['idx_node_name'] is not None:
          idx_running = self.nodeIsRunning(device['idx_node_name'])
        resp_status_for_device.idx_node_running = idx_running 
        ptx_running = False
        if device['ptx_subproc'] is not None and device['ptx_node_name'] is not None:
          ptx_running = self.nodeIsRunning(device['ptx_node_name'])
        resp_status_for_device.ptx_node_running = ptx_running 
        resp.device_statuses.append(resp_status_for_device)
      # Known configurations

      #self.msg_if.pub_warn("Detected ONVIF devices: " + str(self.detected_onvifs))
      #self.msg_if.pub_warn("Connected ONVIF device at: " + str(self.configured_onvifs))
      for uuid in self.configured_onvifs:
        if uuid in self.detected_onvifs.keys():
          device = self.detected_onvifs[uuid]
          if uuid in self.device_name_dict.keys():
              device_name = self.device_name_dict[uuid]
          else:
              device_name = str(device['host']) + ":" + str(device['port'])
          resp_cfg_for_device = OnvifDeviceCfg()    
          resp_cfg_for_device.uuid = uuid
          resp_cfg_for_device.device_name = device_name
          config = self.configured_onvifs[uuid]
          resp_cfg_for_device.username = config['username']
          resp_cfg_for_device.password = config['password']
          resp_cfg_for_device.node_base_name = config['node_base_name']
          resp_cfg_for_device.idx_enabled = config['idx_enabled']
          resp_cfg_for_device.ptx_enabled = config['ptx_enabled']
          resp_cfg_for_device.idx_driver = config['idx_driver']
          resp_cfg_for_device.ptx_driver = config['ptx_driver']      
          resp.device_cfgs.append(resp_cfg_for_device)
    
    return resp

  def resyncOnvifDeviceClocks(self, _):
    for uuid in self.detected_onvifs:
      device = self.detected_onvifs[uuid]
      if (device['connectable'] is False) or  (uuid not in self.configured_onvifs):
        continue
      
      config = self.configured_onvifs[uuid]
      basename = config['node_base_name']
      soapSyncSystemDateAndTime(device['host'], device['port'], config['username'], config['password'])

    return EmptyResponse()
  
  def provideDeviceDriverList(self, _):
    resp = OnvifDriverListQueryResponse()

    idx_drivers_ordered = []
    for driver in self.idx_drivers_dict.keys():
      if driver.find("GENERIC") != -1:
        idx_drivers_ordered.append(driver)
    for driver in self.idx_drivers_dict.keys():
      if driver.find("GENERIC") == -1:
        idx_drivers_ordered.append(driver)
    resp.idx_drivers = idx_drivers_ordered

    ptx_drivers_ordered = []
    for driver in self.ptx_drivers_dict.keys():
      if driver.find("GENERIC") != -1:
         ptx_drivers_ordered.append(driver)
    for driver in self.ptx_drivers_dict.keys():
      if driver.find("GENERIC") == -1:
         ptx_drivers_ordered.append(driver)
    resp.ptx_drivers =  ptx_drivers_ordered
    return resp
  
  def runDiscovery(self, timer):
    #self.msg_if.pub_warn('Debug: running discovery')

    # Some devices only respond once to discovery
    clear_disc = self.clear_discovery
    if clear_disc:
      self.wsd.clearRemoteServices()
    detected_services = self.wsd.searchServices(timeout=1)


    #self.msg_if.pub_warn('Detected services ' + str(detected_services))
    detected_uuids = []
    for service in detected_services:
      
      endpoint_ref = service.getEPR()
      #self.msg_if.pub_info('Detected endpoint' + str(endpoint_ref))
      endpoint_ref_tokens = endpoint_ref.split(':')
      if len(endpoint_ref_tokens) < 3:
        #self.msg_if.pub_warn('Detected ill-formed endpoint reference ' + str(endpoint_ref) + ' skipping')
        continue # Ill-formed
      uuid = endpoint_ref_tokens[2]
      # Some devices randomize the first part of their UUID on each reboot, so truncate that off
      if '-' in uuid:
        uuid_tokens = uuid.split('-')[1:]
        uuid = "-".join(uuid_tokens)
      uuid = uuid.upper()
      detected_uuids.append(uuid)
            
      # Query this device and add to our tracked list if not previously done
      if uuid not in self.detected_onvifs:
        xaddr = service.getXAddrs()[0]
        parsed_xaddr = urllib.parse.urlparse(xaddr)
        hostname = parsed_xaddr.hostname
        port = parsed_xaddr.port if parsed_xaddr.port is not None else 80
        self.msg_if.pub_warn("Detected new ONVIF device at: " + str(hostname) + ":" + str(port))
        # Haven't found any universally valid way to determine if a device supports video and/or pan/tilt, so 
        # just hardcoding these as true for now -- up to the user to enable IDX or PTX as applicable
        #is_nvt = False
        #is_ptz = False
        is_nvt = True
        is_ptz = True
        
        for scope in service.getScopes():
          scope_val = scope.getValue()

          if not scope_val.startswith('onvif'):
            # Skip any WSDiscovery device that is not ONVIF
            continue

          # Check for video streaming
          # Commenting out -- see is_nvt note above
          #if scope_val.endswith(self.ONVIF_SCOPE_NVT_ID) or scope_val.endswith(self.ONVIF_SCOPE_NVT_ALT_ID):
          #  is_nvt = True
                  
          # Check for PTZ
          # Commenting out -- see is_ptz note above
          #if scope_val.endswith(self.ONVIF_SCOPE_PTZ_ID):
          #  is_ptz = True

        # Just skip any WSDiscovery device that is not identified as NVT or PTZ
        if (not is_nvt) and (not is_ptz):
          continue


        self.detected_onvifs[uuid] = {
          'manufacturer' : '',
          'model' : '',
          'firmware_version' : '',
          'hardware_id' : '',
          'host': hostname, 
          'port': port, 
          'video': is_nvt, 
          'ptz': is_ptz, 
          'idx_subproc' : None, 
          'idx_node_name': None,
          'ptx_subproc' : None,
          'ptx_node_name': None,
          'connectable' : False,
          'launch_time' : nepi_sdk.get_time()
        }
        # Now determine if it has a config struct
        self.detected_onvifs[uuid]['config'] = self.configured_onvifs[uuid] if uuid in self.configured_onvifs else None
        if self.detected_onvifs[uuid]['config'] is not None:
          # Check if this device can be reached
          connected = self.attemptONVIFConnection(uuid)
          if connected:
            self.detected_onvifs[uuid]['connectable'] = True
            self.msg_if.pub_info('Connected to device '  + str(uuid) + ' at ' + hostname + ':' + str(port) + ' via configured credentials')

    lost_onvifs = []
    for uuid in self.detected_onvifs:
      detected_onvif = self.detected_onvifs[uuid]
      # Now look for services we've previously detected but are now lost
      lost_connection = False
      #if self.detected_onvifs[uuid]['connectable']:
        #lost_connection = self.attemptONVIFConnection(uuid) == False
      

      if uuid not in detected_uuids or lost_connection:
        self.msg_if.pub_warn('detected uuids: ' + str(detected_uuids)) 
        self.stopAndPurgeNodes(uuid)
        lost_onvifs.append(uuid)
        continue
      

      if detected_onvif['config'] is None:
        continue

      # If not yet connectable, try again here, but don't proceed if still not connectable
      if detected_onvif['connectable'] is False:
        self.detected_onvifs[uuid]['connectable'] = self.attemptONVIFConnection(uuid)
      if not self.detected_onvifs[uuid]['connectable']:
        # Warning logged upstream
        continue

      needs_restart = False

      needs_idx_start = (detected_onvif['video'] is True) and (detected_onvif['idx_subproc'] is None) and \
                        (detected_onvif['config'] is not None) and ('idx_enabled' in detected_onvif['config']) and \
                        (detected_onvif['config']['idx_enabled'] is True)
      if needs_idx_start:
        self.msg_if.pub_info("IDX node needs start " + str(needs_idx_start) )
      # Check for restarts
      '''
      elif (detected_onvif['idx_node_name'] is not None) and (self.nodeIsRunning(detected_onvif['idx_node_name']) is False):
        self.msg_if.pub_warn('IDX node for ' + str(uuid) + ' Not running... will force restart')
        needs_restart = True

      self.msg_if.pub_warn("*******************************************")
      self.msg_if.pub_warn("IDX entry" + str(uuid) )
      self.msg_if.pub_warn("video: " + str(detected_onvif['video']))
      self.msg_if.pub_warn("subproc: " + str(detected_onvif['idx_subproc']))
      self.msg_if.pub_warn("idx_enabled: " + str((detected_onvif['config']['idx_enabled'] is True)))
      self.msg_if.pub_warn(str(detected_onvif['config']))
      '''
      
      needs_ptx_start = (detected_onvif['ptz'] is True) and (detected_onvif['ptx_subproc'] is None) and \
                        (detected_onvif['config'] is not None) and ('ptx_enabled' in detected_onvif['config']) and \
                        (detected_onvif['config']['ptx_enabled'] is True)

      # Do start and restart checks
      launch_time = self.detected_onvifs[uuid]['launch_time']       
      cur_time = nepi_sdk.get_time()             
      if needs_ptx_start:
        self.msg_if.pub_info("PTX needs start " + str(needs_ptx_start) )
      # Check for restarts
      elif (cur_time - launch_time) > self.NODE_LAUNCH_TIME_SEC: #Give nodes time to load
        if (detected_onvif['ptx_node_name'] is not None) and (self.nodeIsRunning(detected_onvif['ptx_node_name']) is False):
          self.msg_if.pub_warn('PTX node for ' + str(uuid) + ' Not running... will force restart')
          needs_restart = True
      '''
      self.msg_if.pub_warn("*******************************************")
      self.msg_if.pub_warn("PTX entry" + str(uuid) )
      self.msg_if.pub_warn("video: " + str(detected_onvif['video']))
      self.msg_if.pub_warn("subproc: " + str(detected_onvif['ptx_subproc']))
      self.msg_if.pub_warn("ptx_enabled: " + str((detected_onvif['config']['ptx_enabled'] is True)))
      self.msg_if.pub_warn(str(detected_onvif['config']))
      '''
      
      needs_start = needs_idx_start or needs_ptx_start
      # Now ensure that the device is connectable via the known/configured credentials
      if needs_start:
        # Device is connectable, so attempt to start the node(s), 
        self.startNodesForDevice(uuid=uuid, start_idx = needs_idx_start, start_ptx = needs_ptx_start)
      if needs_restart is True:
        self.stopAndPurgeNodes(uuid)
    # Finally, purge lost device from our set... can't do it in the detection loop above because it would modify the object 
    # that is being iterated over; throws exception.
    for uuid in lost_onvifs:
      self.detected_onvifs.pop(uuid)
    
    #self.msg_if.pub_warn('Detected Onvif List ' + str(self.detected_onvifs))

    self.status_msg.ready = True

    

    # And now that we are finished, start a timer for the drvt runDiscovery()
    nepi_sdk.start_timer_process(self.discovery_interval, self.runDiscovery, oneshot=True)

  def attemptONVIFConnection(self, uuid):
    if uuid not in self.detected_onvifs:
      self.msg_if.pub_warn("Can't attempt ONVIF connection for undetected device... ignoring")
      return False 
    
    config = self.detected_onvifs[uuid]['config']
    if (config is None) or ('username' not in config) or ('password' not in config):
      self.msg_if.pub_warn('Incomplete ONVIF configuration for ' + str(uuid) + ' ... cannot proceed')
      return False
    
    hostname = self.detected_onvifs[uuid]['host']
    port = self.detected_onvifs[uuid]['port']
    host_reachable = (os.system(f"ping -c 1 {hostname}") == 0)
    if host_reachable is False:
      return False
    
    username = config['username']
    password = config['password']

    try:
      dev_info = soapGetDeviceInformation(hostname, str(port), username, password)
      soapSyncSystemDateAndTime(hostname, str(port), username, password)
      self.msg_if.pub_info('Connected to device '  + str(uuid) + ' at ' + hostname + ':' + str(port) + ' via configured credentials')
    except Exception as e:
      return False
   
    self.detected_onvifs[uuid]['manufacturer'] = dev_info["Manufacturer"]
    self.detected_onvifs[uuid]['model'] = dev_info["Model"]
    self.detected_onvifs[uuid]['firmware_version'] = dev_info["FirmwareVersion"]
    self.detected_onvifs[uuid]['hardware_id'] = dev_info["HardwareId"]
    self.detected_onvifs[uuid]['serial_num'] = dev_info["SerialNumber"]


    return True    

  def startNodesForDevice(self, uuid, start_idx, start_ptx):
    self.msg_if.pub_warn('Starting node launch for ' + str(uuid))
    if uuid not in self.detected_onvifs:
      self.msg_if.pub_warn("Can't start nodes for undetected device... ignoring")
      return False 
    
    config = self.detected_onvifs[uuid]['config']
    if (config is None) or ('node_base_name' not in config) or ('username' not in config) or ('password' not in config):
      self.msg_if.pub_warn('Incomplete configuration for ' + str(uuid) + ' ... cannot proceed')
      return False
      
    serial_num = self.detected_onvifs[uuid]['serial_num'] 
    

    # Should have already ensured all of these exist in attemptONVIFConnection
    username = config['username']
    password = config['password']
    hostname = self.detected_onvifs[uuid]['host']
    port = self.detected_onvifs[uuid]['port']
    
    identifier = hostname.replace(".","")
    device_name = config['node_base_name'] + '_' + identifier
    self.msg_if.pub_info(device_name)
    self.device_name_dict[uuid] = device_name
    ros_node_name = config['node_base_name'] + '_camera_' + identifier
    if start_idx is True:
      self.msg_if.pub_warn('Starting IDX Onvif Node for host' + str(hostname))
      driver_name = self.configured_onvifs[uuid]['idx_driver']
      if driver_name not in self.drvs_dict.keys():
        # self.msg_if.pub_warn('Cant find driver for IDX Onvif Node for host' + str(hostname) + " " + str( driver_name) + " in dict " + str(self.drivers_dict))
        self.msg_if.pub_warn('Failed to find driver ' + driver_name + ' for launch driver node ' + ros_node_name )
        self.msg_if.pub_warn('Driver Dict keys: ' + str(self.drvs_dict.keys()) ) 
      else:
        self.msg_if.pub_warn('Found Driver for IDX Onvif Node host' + str(hostname))
        file_name = self.drvs_dict[driver_name]['NODE_DICT']['file_name']
        connect_namespace = nepi_sdk.create_namespace(self.base_namespace,ros_node_name)
        self.checkLoadConfigFile(node_namespace=connect_namespace)
        drv_dict = self.configured_onvifs[uuid]['idx_drv_dict']
        driver_param_name = ros_node_name + "/drv_dict"
        nepi_sdk.set_param(driver_param_name,drv_dict)
        self.overrideConnectionParams(connect_namespace, username, password, hostname, port, config['idx_driver'])
        # And try to launch the node
        self.msg_if.pub_info('Launching node ' + ros_node_name + ' with file ' + file_name + ' for uuid ' + str(uuid))
        [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, ros_node_name)
        if success == True:
          self.msg_if.pub_info('Launched driver node ' + ros_node_name )
          self.detected_onvifs[uuid]['idx_subproc'] = sub_process
          self.detected_onvifs[uuid]['idx_node_name'] = ros_node_name
        else:
          self.msg_if.pub_warn('Failed to launch driver node ' + ros_node_name )

    if start_ptx is True:
      driver_name = self.configured_onvifs[uuid]['ptx_driver']
      if driver_name in self.drvs_dict.keys():
        file_name = self.drvs_dict[driver_name]['NODE_DICT']['file_name']
        ros_node_name = config['node_base_name'] + '_pan_tilt_' + identifier
        connect_namespace = nepi_sdk.create_namespace(self.base_namespace,ros_node_name)
        self.checkLoadConfigFile(node_namespace=connect_namespace)
        drv_dict = self.configured_onvifs[uuid]['ptx_drv_dict']
        driver_param_name = '/' + ros_node_name + "/drv_dict"
        ns = nepi_sdk.get_full_namespace(self.node_namespace)
        nepi_sdk.set_param(ns + driver_param_name,drv_dict)
        self.overrideConnectionParams(connect_namespace, username, password, hostname, port, config['ptx_driver'])
        # And try to launch the node
        self.detected_onvifs[uuid]['launch_time'] = nepi_sdk.get_time()
        self.msg_if.pub_info('Launching node ' + ros_node_name + ' with file ' + file_name + ' for uuid ' + str(uuid))
        [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, ros_node_name)
        if success == True:
          self.msg_if.pub_info('Launched driver node ' + ros_node_name )
          self.detected_onvifs[uuid]['ptx_subproc'] = sub_process
          self.detected_onvifs[uuid]['ptx_node_name'] = ros_node_name
        else:
          self.msg_if.pub_warn('Failed to launch driver node ' + ros_node_name + ' with msg: ' + str(msg))
      else:
        self.msg_if.pub_warn('Failed to find driver ' + driver_name + ' for launch driver node ' + ros_node_name )

  def overrideConnectionParams(self, connect_namespace, username, password, hostname, port, driver_id):
    ns = nepi_sdk.get_full_namespace(connect_namespace)
    nepi_sdk.set_param(ns + '/credentials/username', username)
    nepi_sdk.set_param(ns + '/credentials/password', password)
    nepi_sdk.set_param(ns + '/network/host', hostname)
    nepi_sdk.set_param(ns + '/network/port', port)
    nepi_sdk.set_param(ns + '/driver_id', str(driver_id))
  
    
  def stopAndPurgeNodes(self, uuid):
    device = self.detected_onvifs[uuid]
    subprocs = [device['idx_subproc'], device['ptx_subproc']]
                 
    for p in subprocs:
      if (p is not None) and (p.poll() is None):
        p.terminate()
        terminate_timeout = 3
        node_dead = False
        while (terminate_timeout > 0):
          time.sleep(1)
          if (p.poll() is None):
            terminate_timeout -= 1
          else:
            node_dead = True
            break
        if not node_dead:
          p.kill()
          time.sleep(1)
        
    if uuid in self.detected_onvifs.keys():
      self.detected_onvifs[uuid]['idx_subproc'] = None
      self.detected_onvifs[uuid]['ptx_subproc'] = None       

  def subprocessIsRunning(self, subproc):
    if subproc.poll() is not None:
      return True
    return False
  
  def nodeIsRunning(self, node_name):
    running = False
    node_list = nepi_sdk.get_node_list()
    for node in node_list:
      if node.find(node_name) != -1:
        running = True
    #if running == False:
      #self.msg_if.pub_warn("Failed to find node_name: " + str(node_name) ) #+ " in node list: " + str(node_list))
    return running
  
  def checkLoadConfigFile(self, node_namespace):
    ros_node_name = node_namespace.split('/')[-1]
    folder_name = "drivers/" + ros_node_name
    node_config_file_folder = os.path.join(self.DEFAULT_NEPI_CONFIG_PATH, folder_name)
    self.msg_if.pub_info(node_config_file_folder)
    # Just make the folder if necessary 
    os.makedirs(node_config_file_folder, exist_ok=True)

    full_path_config_file = os.path.join(node_config_file_folder, ros_node_name + ".yaml")
    full_path_config_file_factory = full_path_config_file + ".num_factory"
    if not os.path.exists(full_path_config_file_factory):
      self.msg_if.pub_warn('No existing config. infrastructure for ' + ros_node_name + ' ... creating folders and empty file')
      with open(full_path_config_file_factory, 'w') as f:
        f.write('# This factory config file was auto-generated by NEPI onvif manager')
      os.symlink(full_path_config_file_factory, full_path_config_file)
    
    self.msg_if.pub_info(self.node_name + ": Loading parameters from " + full_path_config_file + " for " + node_namespace)
    rosparam_load_cmd = ['rosparam', 'load', full_path_config_file, node_namespace]
    subprocess.run(rosparam_load_cmd)
        
    # This doesn't work -- returns a dictionary, but you must still use rosparam.upload_params() to get them to the server!!!
    #rosparam.load_file(filename = full_path_config_file, default_namespace = node_namespace)

  def setCurrentSettingsAsDefault(self):
    if self.node_if is not None:
      self.node_if.set_param('discovery_interval', self.discovery_interval)
      self.node_if.set_param('autosave_cfg_changes', self.autosave_cfg_changes)
      self.node_if.set_param('configured_onvifs', self.configured_onvifs)
  


  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")

# Direct SOAP calls
DEVICE_SERVICE_PATH = "/onvif/device_service"

def soapGetDeviceInformation(globalip, globalport, username, password):
    soap_env = f"""
        <s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope"
                    xmlns:t="http://www.onvif.org/ver10/device/wsdl">
            <s:Header>
                <Security s:mustUnderstand="false" xmlns="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd">
                    <UsernameToken>
                        <Username>{username}</Username>
                        <Password Type="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordText">{password}</Password>
                    </UsernameToken>
                </Security>
            </s:Header>
            <s:Body>
                <t:GetDeviceInformation/>
            </s:Body>
        </s:Envelope>
        """
    headers = {
            "Content-Type": "application/soap+xml",
            "charset": "utf-8",
            "Content-Length": str(len(soap_env)),
        }

    # Send HTTP request
    url = f"http://{globalip}:{globalport}{DEVICE_SERVICE_PATH}"
    soap_resp = requests.post(url, data=soap_env, headers=headers, timeout=30)


    # Initialize to unknown
    device_info = {
        "Manufacturer" : "Unknown",
        "Model" : "Unknown",
        "FirmwareVersion" : "Unknown",
        "SerialNumber" : "Unknown",
        "HardwareId" : "Unknown"
    }

    if soap_resp.status_code != 200:
      self.msg_if.pub_warn('Got error response from onvif device at ' + globalip + ' when querying device info')
      return device_info
    
    root = ET.fromstring(soap_resp.content)

    namespaces = {
        'tds': 'http://www.onvif.org/ver10/device/wsdl',
    }

    dev_info_xml = root.find('.//tds:GetDeviceInformationResponse', namespaces=namespaces)
    device_info["Manufacturer"] = dev_info_xml.find('.//tds:Manufacturer', namespaces=namespaces).text
    device_info["Model"] = dev_info_xml.find('.//tds:Model', namespaces=namespaces).text
    device_info["FirmwareVersion"] = dev_info_xml.find('.//tds:FirmwareVersion', namespaces=namespaces).text
    device_info["SerialNumber"] = dev_info_xml.find('.//tds:SerialNumber', namespaces=namespaces).text
    device_info["HardwareId"] = dev_info_xml.find('.//tds:HardwareId', namespaces=namespaces).text

    return device_info

def soapSyncSystemDateAndTime(globalip, globalport, username, password):
    now = datetime.datetime.now(timezone.utc)
    soap_env = f"""
        <s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope"
                    xmlns:tds="http://www.onvif.org/ver10/device/wsdl"
                    xmlns:tt="http://www.onvif.org/ver10/schema">
            <s:Header>
                <Security s:mustUnderstand="false" xmlns="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd">
                    <UsernameToken>
                        <Username>{username}</Username>
                        <Password Type="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordText">{password}</Password>
                    </UsernameToken>
                </Security>
            </s:Header>
            <s:Body>
              <tds:SetSystemDateAndTime>
                <tds:DateTimeType>Manual</tds:DateTimeType>
                <tds:DaylightSavings>false</tds:DaylightSavings>
                <tds:UTCDateTime>
                  <tt:Time>
                    <tt:Hour>{now.hour}</tt:Hour>
                    <tt:Minute>{now.minute}</tt:Minute>
                    <tt:Second>{now.second}</tt:Second>
                  </tt:Time>
                  <tt:Date>
                    <tt:Year>{now.year}</tt:Year>
                    <tt:Month>{now.month}</tt:Month>
                    <tt:Day>{now.day}</tt:Day>
                  </tt:Date>
                </tds:UTCDateTime>
              </tds:SetSystemDateAndTime>
            </s:Body>
        </s:Envelope>
        """
    headers = {
            "Content-Type": "application/soap+xml",
            "charset": "utf-8",
            "Content-Length": str(len(soap_env)),
        }

    #self.msg_if.pub_warn('Debug: Sending SetSystemTimeAndDate soap_env ' + str(soap_env))
    # Send HTTP request
    url = f"http://{globalip}:{globalport}{DEVICE_SERVICE_PATH}"
    soap_resp = requests.post(url, data=soap_env, headers=headers, timeout=30)
    #self.msg_if.pub_warn(Debug: Got back ' + str(soap_resp.text))





if __name__ == '__main__':
  node = ONVIFMgr()



