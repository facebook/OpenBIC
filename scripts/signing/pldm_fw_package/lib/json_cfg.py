#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 25 16:06:03 2020

@author: mouchen
"""
import json, sys, os
from wsgiref.util import request_uri
sys.path.append(os.path.abspath("./lib"))
from common_lib import Common_time

# import common library
comm_time = Common_time()
get_time = comm_time.get_time

APP_NAME = "json config lib"
APP_RELEASE_VER = "v1.0.0"
APP_RELEASE_DATE = "2023/01/11"

PKG_RELEASE_DATETIME = get_time(1)
PKG_VER_STR = APP_NAME + " - " + APP_RELEASE_VER + " - " + APP_RELEASE_DATE

PKG_HDR_INFO = {
    "pkg_hdr_id" : "1244D2648D7D4718A030FC8A56587D5A",
    "pkg_hdr_format_ver" : 2,
    "pkg_release_datetime" : PKG_RELEASE_DATETIME,
    "pkg_ver_str" : PKG_VER_STR
}

OEM_KEY = 65535 #0xFFFF
COMP_IMG_SET_VER_STR = "PLDM_UPDATE_SUPPORTED_DEVICE"
DESC_COM_IANA = "0000A015"
DESC_OEM_PLATFORM = "GrandTeton"
DESC_OEM_BOARDID = "SwitchBoard"
DESC_OEM_STAGE = "DVT"

DESC_INFO = [
    {
        "DescriptorType" : 1,
        "DescriptorData" : DESC_COM_IANA
    },
    {
        "DescriptorType" : OEM_KEY,
        "VendorDefinedDescriptorTitleString" : "Platform",
        "VendorDefinedDescriptorData" : DESC_OEM_PLATFORM

    },
    {
        "DescriptorType" : OEM_KEY,
        "VendorDefinedDescriptorTitleString" : "BoardID",
        "VendorDefinedDescriptorData" : DESC_OEM_BOARDID
    },
    {
        "DescriptorType" : OEM_KEY,
        "VendorDefinedDescriptorTitleString" : "Stage",
        "VendorDefinedDescriptorData" : DESC_OEM_STAGE
    }
]

COMP_INFO = [
    {
        "ComponentClassification" : OEM_KEY,
        "ComponentIdentifier" : 0,
        "ComponentOptions" : [1],
        "RequestedComponentActivationMethod" : [0],
        "ComponentVersionString" : "version"
    },
]

PACKAGE_CONFIG = [
    DESC_INFO,
    COMP_INFO
]

def TOOL_pldm_json_WR(mode, file, PC=[]):
    data = {}
    # Write File
    if mode == "w" and PC:
        DESC_INFO = PC[0]
        COMP_INFO = PC[1]

        # PackageHeaderInformation
        data['PackageHeaderInformation'] = {
            'PackageHeaderIdentifier': PKG_HDR_INFO["pkg_hdr_id"],
            'PackageHeaderFormatVersion' : PKG_HDR_INFO["pkg_hdr_format_ver"],
            'PackageReleaseDateTime': PKG_HDR_INFO["pkg_release_datetime"],
            'PackageVersionString': PKG_HDR_INFO["pkg_ver_str"]
        }

        # FirmwareDeviceIdentificationArea
        data['FirmwareDeviceIdentificationArea'] = []
        data["FirmwareDeviceIdentificationArea"].append(
            {
                "DeviceUpdateOptionFlags" : [0],
                'ComponentImageSetVersionString' : COMP_IMG_SET_VER_STR,
                "ApplicableComponents" : [100],
                "Descriptors" : DESC_INFO
            }
        )

        # ComponentImageInformationArea
        data['ComponentImageInformationArea'] = COMP_INFO

        with open(file, 'w') as outfile:
            json.dump(data, outfile, indent=4)
    
    # Read File
    elif mode == "r":
        with open(file) as json_file:
            data = json.load(json_file)
            
            # Colect descriptor info            
            desc_info = [
                data['FirmwareDeviceIdentificationArea'][0]['Descriptors']
            ]
            
            # Colect component info            
            comp_info = data['ComponentImageInformationArea']
            
            #print("* ACCOUNT_INFO:\n", desc_info)
            #print("* BRANCH_INFO:\n", comp_info)

            PACKAGE_CONFIG = [
                desc_info,
                comp_info
            ]

            return PACKAGE_CONFIG
            
def TOOL_pldm_plat_cfg_R(file):
    with open(file) as json_file:
        data = json.load(json_file)

    PLATFORM_CONFIG = [
        data['PlatformDescriptor'],
        data['SupportDevice']
    ]

    return PLATFORM_CONFIG

if __name__ == '__main__':
    CONFIG_FILE = "./test.json"
    TOOL_pldm_json_WR('w', CONFIG_FILE, PACKAGE_CONFIG)
    output = TOOL_pldm_json_WR('r', CONFIG_FILE, PACKAGE_CONFIG)
    print(output)
