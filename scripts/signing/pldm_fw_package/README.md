# PROJ_Q_PLDM_FW_UPDATE_TOOL
Package generator for pldm fw update.

### Purpose:
    Tools that used to generate package image for pldm fw update.

### Latest rlease:
    * pldm_pkg_gen_app: NON-SUPPORT
    * pldm_update_pkg_gen: v1.7.0 - 2023/07/27
    * pldm_fwup_pkg_creator: v1.1.0 - 2023/03/21

### Version:
**[pldm_pkg_gen_app]**
- NON-SUPPORT

**[pldm_update_pkg_gen]**
- 1.7.0 - Modify arg-parsing method - 2023/07/27
  - Feature:
  	- Follow standard arg-parsing method, command format not change.
  - Bug:
  	- none
- 1.6.0 - Fix bug - 2023/07/24
  - Feature:
  	- Default debug mode enable and upload first version to OpenBIC.
  - Bug:
  	- Fix debug mode can't execute issue.
- 1.5.1 - Fix bug - 2023/03/30
  - Feature:
  	- Modify VR CheckSum position from the end of version to second part.
  - Bug:
  	- Fix EXE file can't recognize some hidden import issue by changing pldm_fwup_pkg_creator from 'py' to 'exe'.
- 1.5.0 - Modify version parsing - 2023/03/28
  - Feature:
  	- Modify version parsing and VR version should add CheckSum.
  - Bug:
  	- none
- 1.4.0 - Modify MD5 feature - 2023/03/21
  - Feature:
  	- Modify MD5 feature from Whole package to images only, and set it to Descriptor area.
  - Bug:
  	- none
- 1.3.0 - Modify output package nameing format - 2023/03/07
  - Feature:
  	- Modify output package nameing format to **<project_name>** _ **<board_name>** _ **<component_version>** .pldm
	- Add Board name option **-b**.
  - Bug:
  	- none
- 1.2.0 - Support EXE(user do not need to download python) - 2023/01/31
  - Feature:
  	- Change UUID from v1.0.x to v1.1.0
	- Given release package only contains EXE file **pldm_update_pkg_gen**.
  - Bug:
  	- none
- 1.1.0 - Support Auto-GEN - 2023/01/17
  - Feature:
  	- Move platform config outside from code by creating folder 'platform'.
	- Add key '-i' for image path list
	- Support AUTO-GEN which could link with pldm_fwup_pkg_creator after json created.
  - Bug:
  	- Fix list element could not include '-' bug.
- 1.0.0 - First commit - 2023/01/11
  - Feature:
  	- none
  - Bug:
  	- none

**[pldm_fwup_pkg_creator]**
- 1.1.0 - Remove MD5 - 2023/03/21
  - Feature:
  	- Follow pldm_update_pkg_gen v1.4.0.
  - Bug: none
- 1.0.0 - First commit - 2022/12/14
  - Feature:
  	- Add MD5 at the end of package with 16bytes.
  - Bug: none

### Requirement:
- OS
  - Linux: support
  - Windows: support
- Enviroment
  - python3
  - Other library: Only required if DEBUG mode enable, not sure yet.

### Usage
  - **STEP0. Create project config file(only do once)**\
  Create cfg_<project_name>_<board_name>.json to folder **platform**, which could refer to cfg_gt_swb.json.
  - **STEP1. Generate pldm signed image**\
  Format: python3 pldm_update_pkg_gen.py -p **<project_name>** -b **<board_name>** -s **<project_stage>** -c **<component_id>** -v **<component_version>** -i **<component_image>**
    - project_name: Project name
    - board_name: Board name
    - project_stage: poc/evt/dvt/pvt/mp
    - component_id: should follow project spec
    - component_version: should follow project spec
    - component_image: binary image
```
mouchen@mouchen-System-Product-Name:~/$ python3 pldm_update_pkg_gen.py -p gt -b swb -s pvt -c 2 -v "ast1030 ???" -i bic_image/xxxxx.bin 
========================================================
* APP name:    PLDM UPDATE PACKAGE GENERATOR
* APP auth:    Mouchen
* APP version: 1.6
* APP date:    2023/07/24
* NOTE: This APP is based on pldm_fwup_pkg_creator.py
========================================================
[STEP0] Calculate image(s) MD5
Get MD5: 592d20ee05ad09ab90aa2a5a0b62ac3a
--> SUCCESS!

[STEP1] Generate pldm config file
PLDM json file [pldm_cfg.json] has been created!
--> SUCCESS!

[STEP2] Generate pldm package file
Using package file name [gt_swb_ast1030_???.pldm].
============================================================================================
* APP name:     PLDM FWUPDATE PACKAGE CREATOR
* APP auth:     opensource
* APP version:  1.1.0
* APP date:     2023/03/21
* OEM features:
  1. Support string type for data in vendor defined descriptors.
* NOTE: 
* 1. More detail, please check [https://github.com/openbmc/pldm/tree/master/tools/fw-update]
============================================================================================
Please look for pldm package file [gt_swb_ast1030_???.pldm]
--> SUCCESS!
```

### Note
- More command examples, please look at CMD_EX.
- Knowing how to generate EXE file, please look at CMD_EXE.

