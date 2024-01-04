/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SDR_H
#define SDR_H

#include <stdbool.h>
#include <stdint.h>

/* SDR definition follow IPMI specification V2 */

#define IPMI_SDR_VER_15 0x51

#define IPMI_SDR_FULL_SENSOR 0x01
#define IPMI_SDR_COMPACT_SENSOR 0x02
#define IPMI_SDR_EVENT_ONLY 0x03

#define IPMI_SDR_HEADER_LEN 5
#define IPMI_SDR_FULL_SENSOR_MIN_LEN 43
#define IPMI_SDR_COMPACT_SENSOR_MIN_LEN 27
#define IPMI_SDR_EVENT_SENSOR_MIN_LEN 12
#define IPMI_SDR_FRU_SENSOR_MIN_LEN 11
#define IPMI_SDR_MC_SENSOR_MIN_LEN 11

/* Entity IDs, see IPMI spec 43.14, table 43-13 */
#define IPMI_SDR_ENTITY_ID_OTHER 0x01
#define IPMI_SDR_ENTITY_ID_UNKNOWN 0x02
#define IPMI_SDR_ENTITY_ID_PROCESSOR 0x03
#define IPMI_SDR_ENTITY_ID_DISK 0x04
#define IPMI_SDR_ENTITY_ID_SYS_MGT_MOD 0x06
#define IPMI_SDR_ENTITY_ID_SYS_BOARD 0x07
#define IPMI_SDR_ENTITY_ID_MEM_MODULE 0x08
#define IPMI_SDR_ENTITY_ID_POWER_SUPPLY 0x0A
#define IPMI_SDR_ENTITY_ID_ADDIN_CARD 0x0B
#define IPMI_SDR_ENTITY_ID_FRONT_PANEL 0x0C
#define IPMI_SDR_ENTITY_ID_BACK_PANEL 0x0D
#define IPMI_SDR_ENTITY_ID_POWER_SYSTEM_BOARD 0x0E
#define IPMI_SDR_ENTITY_ID_BACKPLANE 0x0F
#define IPMI_SDR_ENTITY_ID_INTERNAL_EXPANSION_BOARD 0x10
#define IPMI_SDR_ENTITY_ID_OTHER_SYSTEM_BOARD 0x11
#define IPMI_SDR_ENTITY_ID_PROCESSOR_BOARD 0x12
#define IPMI_SDR_ENTITY_ID_POWER_UNIT 0x13
#define IPMI_SDR_ENTITY_ID_POWER_MODULE 0x14
#define IPMI_SDR_ENTITY_ID_PDB 0x15
#define IPMI_SDR_ENTITY_ID_CHASSIS_BACK_PANEL_BOARD 0x16
#define IPMI_SDR_ENTITY_ID_SYS_CHASSIS 0x17
#define IPMI_SDR_ENTITY_ID_FAN_DEVICE 0x1D
#define IPMI_SDR_ENTITY_ID_MEMORY 0x20
#define IPMI_SDR_ENTITY_ID_SYS_FW 0x22
#define IPMI_SDR_ENTITY_ID_OS 0x23
#define IPMI_SDR_ENTITY_ID_SYS_BUS 0x24
#define IPMI_SDR_ENTITY_ID_BATTERY 0x28
#define IPMI_SDR_ENTITY_ID_IO_MODULE 0x2C
#define IPMI_SDR_ENTITY_ID_PROCESSOR_IO 0x2D
#define IPMI_SDR_ENTITY_ID_MC_FW 0x2E
#define IPMI_SDR_ENTITY_ID_PCI_BUS 0x30
#define IPMI_SDR_ENTITY_ID_PCIE_BUS 0x31
#define IPMI_SDR_ENTITY_ID_SCSI_BUS 0x32
#define IPMI_SDR_ENTITY_ID_SATA_BUS 0x33
#define IPMI_SDR_ENTITY_ID_FSB 0x34
#define IPMI_SDR_ENTITY_ID_AIR_INLET 0x37
#define IPMI_SDR_ENTITY_ID_PROCESSOR_DCMI 0x41
#define IPMI_SDR_ENTITY_ID_SYS_BOARD_DCMI 0x42

/* sensor type, see IPMI spec 42.2, table 42-3 */
#define IPMI_SDR_SENSOR_TYPE_TEMPERATURE 0x01
#define IPMI_SDR_SENSOR_TYPE_VOLTAGE 0x02
#define IPMI_SDR_SENSOR_TYPE_CURRENT 0x03
#define IPMI_SDR_SENSOR_TYPE_FAN 0x04
#define IPMI_SDR_SENSOR_TYPE_PHY_SECURITY 0x05
#define IPMI_SDR_SENSOR_TYPE_SECURITY_VIO 0x06
#define IPMI_SDR_SENSOR_TYPE_PROCESSOR 0x07
#define IPMI_SDR_SENSOR_TYPE_POWER_SUPPLY 0x08
#define IPMI_SDR_SENSOR_TYPE_POWER_UNIT 0x09
#define IPMI_SDR_SENSOR_TYPE_OTHER_UNIT_BASE 0x0B
#define IPMI_SDR_SENSOR_TYPE_MEMORY 0x0C
#define IPMI_SDR_SENSOR_TYPE_SYS_FW 0x0F
#define IPMI_SDR_SENSOR_TYPE_EVENT_LOG 0x10
#define IPMI_SDR_SENSOR_TYPE_SYS_EVENT 0x12
#define IPMI_SDR_SENSOR_TYPE_CRITICAL_INT 0x13
#define IPMI_SDR_SENSOR_TYPE_BUTTON 0x14
#define IPMI_SDR_SENSOR_TYPE_BOOT_ERR 0x1E
#define IPMI_SDR_SENSOR_TYPE_WATCHDOG2 0x23
#define IPMI_SDR_SENSOR_TYPE_MANGE_HEALTH 0x28
#define IPMI_SDR_SENSOR_TYPE_OEM 0xC0

/* string type, see IPMI spec 43.15 */
#define IPMI_SDR_STRING_TYPE_BCD 0x40
#define IPMI_SDR_STRING_TYPE_ASCII_6 0x80
#define IPMI_SDR_STRING_TYPE_ASCII_8 0xC0

/* sensor unit type, see IPMI spec 43.17, table 43-15 */
#define IPMI_SENSOR_UNIT_UNSPECIFIED 0x00
#define IPMI_SENSOR_UNIT_DEGREE_C 0x01
#define IPMI_SENSOR_UNIT_DEGREE_F 0x02
#define IPMI_SENSOR_UNIT_DEGREE_K 0x03
#define IPMI_SENSOR_UNIT_VOL 0x04
#define IPMI_SENSOR_UNIT_AMP 0x05
#define IPMI_SENSOR_UNIT_WATT 0x06
#define IPMI_SENSOR_UNIT_RPM 0x12

/* sensor initial, see IPMI spec 43.1, Full Sensor Record, byte 11 */
#define IPMI_SDR_SENSOR_INIT_SETTABLE 0x80 /* bit 7 */
#define IPMI_SDR_SENSOR_INIT_SCAN 0x40 /* bit 6 */
#define IPMI_SDR_SENSOR_INIT_EVENT 0x20 /* bit 5 */
#define IPMI_SDR_SENSOR_INIT_THRESHOLD 0x10 /* bit 4 */
#define IPMI_SDR_SENSOR_INIT_HYSTERESIS 0x08 /* bit 3 */
#define IPMI_SDR_SENSOR_INIT_TYPE 0x04 /* bit 2 */
#define IPMI_SDR_SENSOR_INIT_DEF_EVENT 0x02 /* bit 1 */
#define IPMI_SDR_SENSOR_INIT_DEF_SCAN 0x01 /* bit 0 */

/* sensor capabilities, see IPMI spec 43.1, Full Sensor Record, byte 12 */
#define IPMI_SDR_SENSOR_CAP_IGNORE 0x80 /* bit 7 */
#define IPMI_SDR_SENSOR_CAP_AUTO_RE_ARM 0x40 /* bit 6 */
#define IPMI_SDR_SENSOR_CAP_MANUAL_REARM 0x00 /* bit 6 */
#define IPMI_SDR_SENSOR_CAP_HYSTERESIS_NO 0x00 /* bits[5:4], 00 */
#define IPMI_SDR_SENSOR_CAP_HYSTERESIS_RO 0x10 /* bits[5:4], 01 */
#define IPMI_SDR_SENSOR_CAP_HYSTERESIS_RW 0x20 /* bits[5:4], 10 */
#define IPMI_SDR_SENSOR_CAP_HYSTERESIS_FIX 0x30 /* bits[5:4], 11 */
#define IPMI_SDR_SENSOR_CAP_THRESHOLD_NO 0x00 /* bits[3:2], 00 */
#define IPMI_SDR_SENSOR_CAP_THRESHOLD_RO 0x04 /* bits[3:2], 01 */
#define IPMI_SDR_SENSOR_CAP_THRESHOLD_RW 0x08 /* bits[3:2], 10 */
#define IPMI_SDR_SENSOR_CAP_THRESHOLD_FIX 0x0C /* bits[3:2], 11 */
#define IPMI_SDR_SENSOR_CAP_EVENT_CTRL_BIT 0x00 /* bits[1:0], 00 */
#define IPMI_SDR_SENSOR_CAP_EVENT_CTRL_ENTIRE 0x01 /* bits[1:0], 01 */
#define IPMI_SDR_SENSOR_CAP_EVENT_CTRL_GLOBAL 0x02 /* bits[1:0], 10 */
#define IPMI_SDR_SENSOR_CAP_EVENT_CTRL_NO 0x03 /* bits[1:0], 11 */

/* event/reading type, see IPMI spec 42.1, table 42-1 */
#define IPMI_SDR_EVENT_TYPE_THRESHOLD 0x01
#define IPMI_SDR_EVENT_TYPE_USAGE 0x02
#define IPMI_SDR_EVENT_TYPE_DEAS_ASSE 0x03
#define IPMI_SDR_EVENT_TYPE_LIMIT_EXCEED 0x05
#define IPMI_SDR_EVENT_TYPE_PERFORMANCE 0x06
#define IPMI_SDR_EVENT_TYPE_SEVERITY 0x07
#define IPMI_SDR_EVENT_TYPE_PRESENT 0x08
#define IPMI_SDR_EVENT_TYPE_EN_DIS 0x09
#define IPMI_SDR_EVENT_TYPE_DIS_REDUNDANCY 0x0B
#define IPMI_SDR_EVENT_TYPE_SENSOR_SPEC 0x6F

/* assertion event mask / lower threshold reading mask */
#define IPMI_SDR_ASSERT_MASK_LNCT_LO 0x01 /* bit 0 */
#define IPMI_SDR_ASSERT_MASK_LNCT_HI 0x02 /* bit 1 */
#define IPMI_SDR_ASSERT_MASK_LCT_LO 0x04 /* bit 2 */
#define IPMI_SDR_ASSERT_MASK_LCT_HI 0x08 /* bit 3 */
#define IPMI_SDR_ASSERT_MASK_LNRT_LO 0x10 /* bit 4 */
#define IPMI_SDR_ASSERT_MASK_LNRT_HI 0x20 /* bit 5 */
#define IPMI_SDR_ASSERT_MASK_UNCT_LO 0x40 /* bit 6 */
#define IPMI_SDR_ASSERT_MASK_UNCT_HI 0x80 /* bit 7 */

#define IPMI_SDR_ASSERT_MASK_UCT_LO 0x01 /* bit 8 */
#define IPMI_SDR_ASSERT_MASK_UCT_HI 0x02 /* bit 9 */
#define IPMI_SDR_ASSERT_MASK_UNRT_LO 0x04 /* bit 10 */
#define IPMI_SDR_ASSERT_MASK_UNRT_HI 0x08 /* bit 11 */
#define IPMI_SDR_CMP_RETURN_LNCT 0x10 /* bit 12 */
#define IPMI_SDR_CMP_RETURN_LCT 0x20 /* bit 13 */
#define IPMI_SDR_CMP_RETURN_LNRT 0x40 /* bit 14 */

/* deassertion event mask / upper threshold reading mask */
#define IPMI_SDR_DEASSERT_MASK_LNCT_LO 0x01 /* bit 0 */
#define IPMI_SDR_DEASSERT_MASK_LNCT_HI 0x02 /* bit 1 */
#define IPMI_SDR_DEASSERT_MASK_LCT_LO 0x04 /* bit 2 */
#define IPMI_SDR_DEASSERT_MASK_LCT_HI 0x08 /* bit 3 */
#define IPMI_SDR_DEASSERT_MASK_LNRT_LO 0x10 /* bit 4 */
#define IPMI_SDR_DEASSERT_MASK_LNRT_HI 0x20 /* bit 5 */
#define IPMI_SDR_DEASSERT_MASK_UNCT_LO 0x40 /* bit 6 */
#define IPMI_SDR_DEASSERT_MASK_UNCT_HI 0x80 /* bit 7 */

#define IPMI_SDR_DEASSERT_MASK_UCT_LO 0x01 /* bit 8 */
#define IPMI_SDR_DEASSERT_MASK_UCT_HI 0x02 /* bit 9 */
#define IPMI_SDR_DEASSERT_MASK_UNRT_LO 0x04 /* bit 10 */
#define IPMI_SDR_DEASSERT_MASK_UNRT_HI 0x08 /* bit 11 */
#define IPMI_SDR_CMP_RETURN_UNCT 0x10 /* bit 12 */
#define IPMI_SDR_CMP_RETURN_UCT 0x20 /* bit 13 */
#define IPMI_SDR_CMP_RETURN_UNRT 0x40 /* bit 14 */

/* settable/readable threshold moask */
#define IPMI_SDR_LNCT_READABLE 0x01 /* bit 0 */
#define IPMI_SDR_LCT_READABLE 0x02 /* bit 1 */
#define IPMI_SDR_LNRT_READABLE 0x04 /* bit 2 */
#define IPMI_SDR_UNCT_READABLE 0x08 /* bit 3 */
#define IPMI_SDR_UCT_READABLE 0x10 /* bit 4 */
#define IPMI_SDR_UNRT_READABLE 0x20 /* bit 5 */

#define IPMI_SDR_LNCT_SETTABLE 0x01 /* bit 8 */
#define IPMI_SDR_LCT_SETTABLE 0x02 /* bit 9 */
#define IPMI_SDR_LNRT_SETTABLE 0x04 /* bit 10 */
#define IPMI_SDR_UNCT_SETTABLE 0x08 /* bit 11 */
#define IPMI_SDR_UCT_SETTABLE 0x10 /* bit 12 */
#define IPMI_SDR_UNRT_SETTABLE 0x20 /* bit 13 */

/* linearization */
#define IPMI_SDR_LINEAR_LINEAR 0x00
#define IPMI_SDR_LINEAR_LN 0x01
#define IPMI_SDR_LINEAR_LOG10 0x02
#define IPMI_SDR_LINEAR_LOG2 0x03
#define IPMI_SDR_LINEAR_EXP 0x04
#define IPMI_SDR_LINEAR_EXP10 0x05
#define IPMI_SDR_LINEAR_EXP2 0x06
#define IPMI_SDR_LINEAR_1_X 0x07
#define IPMI_SDR_LINEAR_SQR 0x08
#define IPMI_SDR_LINEAR_CUBE 0x09
#define IPMI_SDR_LINEAR_SQRT 0x0A
#define IPMI_SDR_LINEAR_CUBE_1 0x0B

#define IPMI_NOR_READING_SPEC 0x01
#define IPMI_NOR_MAX_SPEC 0x02
#define IPMI_NOR_MIN_SPEC 0x04

/* sensor direction */
#define IPMI_SDR_SENSOR_DIREC_NO 0x00
#define IPMI_SDR_SENSOR_DIREC_IN 0x40
#define IPMI_SDR_SENSOR_DIREC_OUT 0x80

/* ID string instance modifier type */
#define IPMI_ID_STR_MODIFIER_NUM 0x00
#define IPMI_ID_STR_MODIFIER_ALPHA 0x10

/* ID entity instance sharing */
#define IPMI_ENTIFY_INSTANCE_SHARE_SAME 0x00
#define IPMI_ENTIFY_INSTANCE_SHARE_INC 0x80

/* generic offset for IPMI_SDR_EVENT_TYPE_PRESENT */
#define IPMI_SDR_EVENT_TYPE_ABS_PRES_ABSENT 0x01
#define IPMI_SDR_EVENT_TYPE_ABS_PRES_PRESENT 0x02

/* generic offset for IPMI_SDR_EVENT_TYPE_DEAS_ASSE */
#define IPMI_SDR_EVENT_TYPE_STATE_DEASSERT 0x01
#define IPMI_SDR_EVENT_TYPE_STATE_ASSERT 0x02

/* generic offset for IPMI_SDR_EVENT_TYPE_DIS_REDUNDANCY */
#define IPMI_SDR_EVENT_TYPE_FULL_REDUNDANCY 0x01
#define IPMI_SDR_EVENT_TYPE_REDUNDANCY_LOST 0x02

/* generic offset for IPMI_SDR_EVENT_TYPE_EN_DIS */
#define IPMI_SDR_EVENT_TYPE_EN_DIS_DISABLE 0x01
#define IPMI_SDR_EVENT_TYPE_EN_DIS_ENABLE 0x02

/* sensor-specific offset for IPMI_SDR_SENSOR_TYPE_PROCESSOR */
#define IPMI_SDR_SENSOR_SPEC_EVENT_PROCESSOR_IERR 0x01 /* bit 0 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_PROCESSOR_THERMAL_TRIP 0x02 /* bit 1 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_PROCESSOR_FRB1 0x04 /* bit 2 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_PROCESSOR_FRB2 0x08 /* bit 3 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_PROCESSOR_FRB3 0x10 /* bit 4 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_PROCESSOR_PRESENCE 0x80 /* bit 7 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_PROCESSOR_MCERR 0x08 /* bit 11 */ // Jerry130827

/* sensor-specific offset for IPMI_SDR_SENSOR_TYPE_MEMORY */
#define IPMI_SDR_SENSOR_SPEC_EVENT_MEMORY_CORRECT_ECC 0x01 /* bit 0 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_MEMORY_UNCORRECT_ECC 0x02 /* bit 1 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_MEMORY_PARITY 0x04 /* bit 2 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_MEMORY_ECC_LOG_LIMIT_REACH 0x20 /* bit 6*/ // brenden130924

/* sensor-specific offset for IPMI_SDR_SENSOR_TYPE_POWER_SUPPLY */
#define IPMI_SDR_EVENT_POWER_SUPPLY_PRESENCE 0x01 /* bit 0 */
#define IPMI_SDR_EVENT_POWER_SUPPLY_FAILURE 0x02 /* bit 1 */
#define IPMI_SDR_EVENT_POWER_SUPPLY_PRE_FAILURE 0x04 /* bit 2 */
#define IPMI_SDR_EVENT_POWER_SUPPLY_INPUT_LOST 0x08 /* bit 3 */

/* sensor-specific offset for IPMI_SDR_SENSOR_TYPE_POWER_UNIT */
#define IPMI_SDR_SENSOR_SPEC_EVENT_POWER_UNIT_POWER_OFF 0x01 /* bit 0 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_POWER_UNIT_POWER_CYCLE 0x02 /* bit 1 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_POWER_UNIT_AC_LOST 0x10 /* bit 4 */

/* sensor-specific offset for IPMI_SDR_SENSOR_TYPE_CRITICAL_INT */
#define IPMI_SDR_SENSOR_SPEC_EVENT_CRITICAL_INT_FP_NMI 0x01 /* bit 0 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_CRITICAL_INT_SW_NMI 0x08 /* bit 3 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_CRITICAL_INT_PERR 0x10 /* bit 4 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_CRITICAL_INT_SERR 0x20 /* bit 5 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_CRITICAL_INT_NCERR 0x80 /* bit 7 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_CRITICAL_INT_NFERR 0x01 /* bit 8 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_CRITICAL_INT_FATAL_NMI 0x02 /* bit 9 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_CRITICAL_INT_FERR 0x04 /* bit 10 */

/* sensor-specific offset for IPMI_SDR_SENSOR_TYPE_BUTTON */
#define IPMI_SDR_SENSOR_SPEC_EVENT_BTN_POWER 0x01 /* bit 0 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_BTN_SELLP 0x02 /* bit 1 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_BTN_RESET 0x04 /* bit 2 */

/* sensor-specific offset for IPMI_SDR_SENSOR_TYPE_WATCHDOG2 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_WDT2_EXPIRE 0x01 /* bit 0 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_WDT2_RESET 0x02 /* bit 1 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_WDT2_DOWN 0x04 /* bit 2 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_WDT2_CYCLE 0x08 /* bit 3 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_WDT2_INT 0x01 /* bit 8 */

/* sensor-specific offset for IPMI_SDR_SENSOR_TYPE_BOOT_ERR */
#define IPMI_SDR_SENSOR_SPEC_EVENT_BOOT_ERR_NO_MEDIA 0x01 /* bit 0 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_BOOT_ERR_NON_BOOTABLE 0x02 /* bit 1 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_BOOT_ERR_PXE_NO 0x04 /* bit 2 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_BOOT_ERR_INVALID_SECTOR 0x08 /* bit 3 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_BOOT_ERR_TIMEOUT 0x10 /* bit 4 */

/* sensor-specific offset for IPMI_SDR_SENSOR_TYPE_SYS_FW */
#define IPMI_SDR_SENSOR_SPEC_EVENT_SYS_FW_POST_ERR 0x01 /* bit 0 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_SYS_FW_HANG 0x02 /* bit 1 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_SYS_FW_PROGRESS 0x04 /* bit 2 */

/* sensor-specific offset for IPMI_SDR_SENSOR_TYPE_EVENT_LOG */
#define IPMI_SDR_SENSOR_SPEC_EVENT_LOG_MEM_ERR_DIS 0x01 /* bit 0 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_LOG_TYPE_DIS 0x02 /* bit 1 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_LOG_CLEAR 0x04 /* bit 2 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_LOG_ALL_DIS 0x08 /* bit 3 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_LOG_SEL_FULL 0x10 /* bit 4 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_LOG_SEL_ALMOST_FULL 0x20 /* bit 5 */

/* sensor-specific offset for IPMI_SDR_SENSOR_TYPE_SYS_EVENT */
#define IPMI_SDR_SENSOR_SPEC_EVENT_SYS_EVENT_RECONFIG 0x01 /* bit 0 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_SYS_EVENT_OEM 0x02 /* bit 1 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_SYS_EVENT_HW_FAIL 0x04 /* bit 2 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_SYS_EVENT_ADD_AUXI 0x08 /* bit 3 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_SYS_EVENT_PEF 0x10 /* bit 4 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_SYS_EVENT_CLOCK 0x20 /* bit 5 */

/* sensor-specific offset for IPMI_SDR_SENSOR_TYPE_PHY_SECURITY */
#define IPMI_SDR_SENSOR_SPEC_EVENT_PHYSICAL_SECURITY_CHASSIS 0x01 /* bit 0 */

/* sensor-specific offset for IPMI_SDR_SENSOR_TYPE_MANAGEMENT */ // Jerry130723 SEL BMC Watchdog timeout
#define IPMI_SDR_SENSOR_SPEC_EVENT_MANAGEMENT_UNAVAILABLE 0x08 /* bit 3 */

/* sensor-specific offset for IPMI_SDR_SENSOR_TYPE_CPU_DIMM_HOT */
#define IPMI_SDR_SENSOR_SPEC_EVENT_CPU_HOT 0x01 /* bit 0 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_DIMM_HOT 0x02 /* bit 1 */

/* sensor-specific offset for IPMI_SDR_SENSOR_TYPE_CPU_DIMM_VR_HOT */
#define IPMI_SDR_SENSOR_SPEC_EVENT_CPU_VR_HOT 0x01 /* bit 0 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_DIMM_AB_VR_HOT 0x02 /* bit 1 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_DIMM_DE_VR_HOT 0x04 /* bit 2 */

/* sensor-specific offset for IPMI_SDR_SENSOR_TYPE_SYS_STA */
#define IPMI_SDR_SENSOR_SPEC_EVENT_SYS_THERMAL_TRIP 0x01 /* bit 0 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_SYS_FIVR_FAULT 0x02 /* bit 1 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_SYS_THROTTLE 0x04 /* bit 2 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_SYS_PCHHOT 0x08 /* bit 3 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_FM_THROT 0x10 /* bit 4 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_FAST_THROT 0x20 /* bit 5 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_PMBUS_ALERT 0x40 /* bit 6 */

/* sensor-specific offset for IPMI_SDR_SENSOR_TYPE_SYS_BOOT_STA */
#define IPMI_SDR_SENSOR_SPEC_EVENT_SLP_S4 0x01 /* bit 0 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_SLP_S3 0x02 /* bit 1 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_PCH_PWROK 0x04 /* bit 2 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_SYS_PWROK 0x08 /* bit 3 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_PLTRST 0x10 /* bit 4 */
#define IPMI_SDR_SENSOR_SPEC_EVENT_POST_CLT 0x20 /* bit 5 */

#define SDR_END_ID 0xFFFF
#define SDR_INVALID_ID 0xFFFE
#define MAX_SDR_SENSOR_NAME_LEN 32

enum rsv_table_index {
	RSV_TABLE_INDEX_0 = 0,
	RSV_TABLE_INDEX_1,
};

typedef struct _SDR_Full_sensor_ {
	/*** sensor record header ***/
	uint8_t record_id_h; /* Record ID high byte */
	uint8_t record_id_l; /* Record ID low byte */
	uint8_t sdr_ver; /* SDR version */
	uint8_t record_type; /* Record type */
	uint8_t record_len; /* Record length */

	/*** record key bytes ***/
	uint8_t owner_id; /* 8 bit owner ID */
	uint8_t owner_lun; /* [7:4] channel number, [3:2] LUN for write/read FRU commands, [1:0] sensor owner LUN */
	uint8_t sensor_num; /* unique sensor number */

	/*** record body bytes ***/
	uint8_t entity_id; /* physical entity */
	uint8_t entity_instance; /* instance number for entity */
	uint8_t sensor_init; /* report how this sensor coms up on deivce power up or cold reset */
	uint8_t sensor_capabilities; /* [6] auto re-arm support, [5:4] hysteresis support, [3:2] threshold access, [1:0] event message control support */
	uint8_t sensor_type; /* sensor type, e.g. temperature, voltage, etc. */
	uint8_t evt_read_type; /* event/Reading Type code */
	uint8_t assert_evt_mask; /* assertion event mask */
	uint8_t lower_thres_read_mask; /* low threshold reading mask */
	uint8_t deassert_evt_mask; /* deassertion event mask */
	uint8_t upper_thres_read_mask; /* upper threshold read mask */
	uint8_t discrete_read_mask; /* discrete sensor read mask */
	uint8_t discrete_thres_read_mask; /* [15:8] settable threshold mask, [7:0] readable threshold mask */
	uint8_t sensor_unit1;
	uint8_t sensor_unit2;
	uint8_t sensor_unit3;
	uint8_t linear;
	uint8_t M;
	uint8_t M_tolerance;
	uint8_t B;
	uint8_t B_accuracy;
	uint8_t accuracy;
	uint8_t RexpBexp; /* [7:4] R exp 4 bits, 2's completment, signed, [3:0] B exp 4 bits, 2's completment, signed */
	uint8_t analog_char_flag;
	uint8_t nominal_read;
	uint8_t normal_max;
	uint8_t normal_min;
	uint8_t sensor_max_read;
	uint8_t sensor_min_read;
	uint8_t UNRT;
	uint8_t UCT;
	uint8_t UNCT;
	uint8_t LNRT;
	uint8_t LCT;
	uint8_t LNCT;
	uint8_t positive_going_thres;
	uint8_t negative_going_thres;
	uint8_t reserved0;
	uint8_t reserved1;
	uint8_t OEM;
	uint8_t ID_len;
	uint8_t ID_str[MAX_SDR_SENSOR_NAME_LEN];
} SDR_Full_sensor;

typedef struct _SDR_INFO_ {
	uint16_t start_ID;
	uint16_t last_ID;
	uint16_t current_ID;
} SDR_INFO;

enum {
	THRESHOLD_UNR,
	THRESHOLD_UCR,
	THRESHOLD_UNC,
	THRESHOLD_LNR,
	THRESHOLD_LCR,
	THRESHOLD_LNC,
	/* Follow the section 36.3 of IPMI spec,
	 * there are three parameters for sensor reading conversion formula.
	 * M: Signed integer constant multiplier
	 * B: Signed additive "offset"
	 * R: Signed Exponent
	 */
	MBR_M,
	MBR_B,
	MBR_R,
};

extern uint8_t sdr_count;
extern bool is_sdr_not_init;
// Mapping sensor number to sdr config index
extern uint8_t sdr_index_map[];
extern SDR_Full_sensor *full_sdr_table;
extern uint8_t sensor_config_size;
extern const int negative_ten_power[16];
#define SDR_M(sensor_num)                                                                          \
	(((full_sdr_table[sdr_index_map[sensor_num]].M_tolerance & 0xC0) << 2) |                   \
	 full_sdr_table[sdr_index_map[sensor_num]].M)
#define SDR_R(sensor_num) ((full_sdr_table[sdr_index_map[sensor_num]].RexpBexp >> 4) & 0x0F)
#define SDR_Rexp(sensor_num) negative_ten_power[SDR_R(sensor_num)]

static inline uint8_t round_add(uint8_t sensor_num, int val)
{
	return (SDR_R(sensor_num) > 0) ?
		       (((negative_ten_power[((SDR_R(sensor_num) + 1) & 0xF)] * val) % 10) > 5 ?
				1 :
				0) :
		       0;
}

uint16_t SDR_get_record_ID(uint16_t current_ID);
bool SDR_check_record_ID(uint16_t current_ID);
uint16_t SDR_get_RSV_ID(uint8_t rsv_table_index);
bool SDR_RSV_ID_check(uint16_t ID, uint8_t rsv_table_index);
uint8_t sdr_init(void);
void pal_fix_full_sdr_table(void);
bool check_sdr_num_exist(uint8_t sensor_num);
void add_full_sdr_table(SDR_Full_sensor add_item);
void change_sensor_threshold(uint8_t sensor_num, uint8_t threshold_type, uint8_t change_value);
void change_sensor_mbr(uint8_t sensor_num, uint8_t mbr_type, uint16_t change_value);
uint8_t plat_get_sdr_size();
void load_sdr_table(void);

#endif
