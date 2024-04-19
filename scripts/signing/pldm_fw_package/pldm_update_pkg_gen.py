#!/usr/bin/env python3

import sys, subprocess, argparse

import lib.json_cfg as json_lib
from lib.common_lib import Common_msg, Common_file, System_ctrl

APP_NAME = "PLDM UPDATE PACKAGE GENERATOR"
APP_RELEASE_VER = "1.8"
APP_RELEASE_DATE = "2024/04/19"

PLATFORM_PATH = "./platform"
CONFIG_FILE = "pldm_cfg.json"
DEF_PKG_FILE = "default_package.pldm"

OEM_KEY = 65535 #0xFFFF

STAGE_CFG = {
    "poc" : "POC",
    "evt" : "EVT",
    "dvt" : "DVT",
    "pvt" : "PVT",
    "mp" : "MP",
}

DBG_EN = True # Enable if there's no Sign EXE file

# import common library
comm_msg = Common_msg()
msg_hdr_print = comm_msg.msg_hdr_print
comm_file = Common_file()
is_file_exist = comm_file.is_file_exist
resource_path = comm_file.resource_path
get_md5_str_from_file = comm_file.get_md5_str_from_file_list
comm_sys = System_ctrl()
platform_os = comm_sys.os_name.lower()

# EXE file
EXE_WIN_FILE = resource_path("pldm_fwup_pkg_creator.exe")
EXE_LINUX_FILE = resource_path("./pldm_fwup_pkg_creator")

if DBG_EN == True:
    command_prefix = resource_path("pldm_fwup_pkg_creator.py")
else:
    if platform_os == "windows":
        command_prefix = EXE_WIN_FILE
        if not is_file_exist(EXE_WIN_FILE):
            print("Can't find exe file " + EXE_WIN_FILE)
            sys.exit(1)
    elif platform_os == "linux":
        command_prefix = EXE_LINUX_FILE
        if not is_file_exist(EXE_LINUX_FILE):
            print("Can't find exe file " + EXE_LINUX_FILE)
            sys.exit(1)
    else:
        print("Current os " + platform_os + " is not supported!")
        sys.exit(1)

def APP_HEADER():
    msg_hdr_print("n", "========================================================")
    msg_hdr_print("n", "* APP name:    "+APP_NAME)
    msg_hdr_print("n", "* APP version: "+APP_RELEASE_VER)
    msg_hdr_print("n", "* APP date:    "+APP_RELEASE_DATE)
    msg_hdr_print("n", "* NOTE: This APP is based on pldm_fwup_pkg_creator.py")
    msg_hdr_print("n", "========================================================")

def get_parser():
    parser = argparse.ArgumentParser(description="HELP:")
    parser.add_argument("-p", "--platform", type=str, required=True, help="Platform select.")
    parser.add_argument("-b", "--board", type=str, required=True, help="Board select.")
    parser.add_argument("-s", "--stage", type=str, choices=['poc', 'evt', 'dvt', 'pvt', 'mp'], required=True, help="Stage select.")
    parser.add_argument("-c", "--compid", type=int, nargs='*', required=True, help="Component id select. Please follow spec, could be list(-c 1 2).")
    parser.add_argument("-v", "--version", type=str, nargs='*', required=True, help="Component version string select. Please follow spec, could be list(-v 'dev1 ver1' 'dev2 ver2').")
    parser.add_argument("-i", "--image", type=str, nargs='*', required=True, help="Component image select. Please follow spec, could be list(-i ./img1 ./img2).")
    return parser

def PLAT_CheckVRChecksum(str, byte_num):
    if len(str) < byte_num*2:
        return False

    focus_str = str[0:byte_num*2]

    for c in focus_str:
        try:
            int(c, 16)
        except:
            return False

    return True

if __name__ == '__main__':
    APP_HEADER()

    parser = get_parser()
    args = parser.parse_args()

    select_platform = args.platform
    select_board = args.board
    select_stage = args.stage
    select_comp_id_lst = args.compid
    select_comp_version_lst = args.version
    select_comp_img_lst = args.image

    if len(select_comp_id_lst) != len(select_comp_version_lst) or len(select_comp_version_lst) != len(select_comp_img_lst):
        msg_hdr_print('e', "Component id, version, image count should be the same")
        sys.exit(0)

    for i in range(len(select_comp_version_lst)):
        if (len(select_comp_version_lst[i].split(' ')) != 2):
            msg_hdr_print('e', "Component #" + str(i+1) + " got invalid version format, should be '<device> <ver>'")
            sys.exit(0)

    PACKAGE_CONFIG = []
    DESC_INFO = []
    COMP_INFO = []

    cfg_file = resource_path(PLATFORM_PATH + "/cfg_" + str(select_platform) + "_" + str(select_board) + ".json")
    if not is_file_exist(cfg_file):
        msg_hdr_print('e', "Can't find config file " + cfg_file.split('./')[1] + " by given platform[" + str(select_platform) + "] and board[" + str(select_board) + "]")
        sys.exit(0)

    PLAT_INFO = json_lib.TOOL_pldm_plat_cfg_R(cfg_file)
    DESC_INFO = PLAT_INFO[0]

    try:
        DESC_INFO[3]["VendorDefinedDescriptorData"] = STAGE_CFG[select_stage]
    except:
        msg_hdr_print('e', "Invalid given stage [" + str(select_stage) + "]")
        sys.exit(1)

    package_name_lst = []
    found_verify_flag = 0
    for i in range(len(select_comp_id_lst)):
        for comp in PLAT_INFO[1]:
            if select_comp_id_lst[i] in comp["CompID"]:
                version_prefix = comp["Device"]
                given_prefix = select_comp_version_lst[i].split(' ')[0]
                given_subfix = select_comp_version_lst[i].split(' ')[1]
                if given_prefix == version_prefix:
                    found_verify_flag = 1

                    if comp["CheckSum"] == "y":
                        img_checksum = given_subfix.split('_')[0]
                        # checksum should be even
                        if len(img_checksum) % 2 != 0:
                            msg_hdr_print('e', "Component #" + str(select_comp_id_lst[i]) + " checksum [" + img_checksum + "] byte count should be even!")
                            sys.exit(1)
                        checksum_byte_cnt = int(len(img_checksum)/2)
                        if PLAT_CheckVRChecksum(img_checksum, checksum_byte_cnt) == False:
                            msg_hdr_print('e', "Component #" + str(select_comp_id_lst[i]) + " sub version string [" + given_subfix + "] need CheckSum in front with format <checksum_version>!")
                            sys.exit(1)
                    break
        
        if not found_verify_flag:
            msg_hdr_print('e', "Can't find support device by given component version [" + str(select_comp_version_lst[i]) + "]")
            sys.exit(1)

        package_name_lst.append(str(select_platform) + "_" + str(select_board) + "_" + str(select_comp_version_lst[i]).replace(" ", "_") + comp["pkg_suffix"])

        COMP_INFO.append({
            "ComponentClassification" : OEM_KEY,
            "ComponentIdentifier" : select_comp_id_lst[i],
            "ComponentOptions" : [1],
            "RequestedComponentActivationMethod" : [0],
            "ComponentVersionString" : select_comp_version_lst[i]
        })

    msg_hdr_print("n", "[STEP0] Calculate image(s) MD5")

    comp_md5 = ""
    for img in select_comp_img_lst:
        if not is_file_exist(img):
            msg_hdr_print('e', "Invalid given image [" + str(img) + "]")
            sys.exit(1)

    comp_md5 = get_md5_str_from_file(select_comp_img_lst)

    DESC_INFO.append({
        "DescriptorType" : 65535,
        "VendorDefinedDescriptorTitleString" : "Image MD5",
        "VendorDefinedDescriptorData" : comp_md5
    })
    msg_hdr_print("n", "Get MD5: " + comp_md5)
    msg_hdr_print("n", "--> SUCCESS!")

    PACKAGE_CONFIG = [
        DESC_INFO,
        COMP_INFO
    ]

    msg_hdr_print("n", "\n[STEP1] Generate pldm config file")
    json_lib.TOOL_pldm_json_WR('w', CONFIG_FILE, PACKAGE_CONFIG)
    msg_hdr_print("n", "PLDM json file [" + CONFIG_FILE + "] has been created!")
    msg_hdr_print("n", "--> SUCCESS!")

    msg_hdr_print("n", "\n[STEP2] Generate pldm package file")

    if len(select_comp_id_lst) != 1:
        pkg_file_name = DEF_PKG_FILE
        msg_hdr_print("n", "Using default package file name [" + pkg_file_name + "], cause of muti-comp case.")
    else:
        pkg_file_name = package_name_lst[0]
        msg_hdr_print("n", "Using package file name [" + pkg_file_name + "].")

    if DBG_EN == True:
        cmd_line = ["python3", command_prefix, pkg_file_name, CONFIG_FILE]
    else:
        cmd_line = [command_prefix, pkg_file_name, CONFIG_FILE]

    for img in select_comp_img_lst:
        cmd_line.append(img)

    subprocess.run(cmd_line)
