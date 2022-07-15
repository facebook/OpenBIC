#ifndef _LIBIPMI_H_
#define _LIBIPMI_H_

/* sensor type, see IPMI spec 42.2, table 42-3 */
#define IPMI_SENSOR_TYPE_TEMPERATURE 0x01
#define IPMI_SENSOR_TYPE_VOLTAGE 0x02
#define IPMI_SENSOR_TYPE_CURRENT 0x03
#define IPMI_SENSOR_TYPE_FAN 0x04
#define IPMI_SENSOR_TYPE_PHY_SECURITY 0x05
#define IPMI_SENSOR_TYPE_SECURITY_VIO 0x06
#define IPMI_SENSOR_TYPE_PROCESSOR 0x07
#define IPMI_SENSOR_TYPE_POWER_SUPPLY 0x08
#define IPMI_SENSOR_TYPE_POWER_UNIT 0x09
#define IPMI_SENSOR_TYPE_SYS_EVENT 0x12
#define IPMI_SENSOR_TYPE_MEMORY 0x0C
#define IPMI_SENSOR_TYPE_SYS_FW 0x0F
#define IPMI_SENSOR_TYPE_EVENT_LOG 0x10
#define IPMI_SENSOR_TYPE_CRITICAL_INT 0x13
#define IPMI_SENSOR_TYPE_BUTTON 0x14
#define IPMI_SENSOR_TYPE_BOOT_ERR 0x1E
#define IPMI_SENSOR_TYPE_WATCHDOG2 0x23
#define IPMI_SENSOR_TYPE_MANAGEMENT 0x28 //Jerry130723
#define IPMI_SENSOR_TYPE_VERSION_CHANGE 0x2B //Jerry130913
#define IPMI_OEM_SENSOR_TYPE_OEM 0xC0
#define IPMI_OEM_SENSOR_TYPE_OEM_C3 0xC3
#define IPMI_OEM_SENSOR_TYPE_CPU_DIMM_VR_HOT 0xC6
#define IPMI_OEM_SENSOR_TYPE_CPU_DIMM_HOT 0xC7
#define IPMI_OEM_SENSOR_TYPE_SYS_STA 0xC9
#define IPMI_OEM_SENSOR_TYPE_SYS_BOOT_STA 0xCA

/* event/reading type, see IPMI spec 42.1, table 42-1 */
#define IPMI_EVENT_TYPE_THRESHOLD 0x01
#define IPMI_EVENT_TYPE_USAGE 0x02
#define IPMI_EVENT_TYPE_DEAS_ASSE 0x03
#define IPMI_EVENT_TYPE_LIMIT_EXCEED 0x05
#define IPMI_EVENT_TYPE_PERFORMANCE 0x06
#define IPMI_EVENT_TYPE_SEVERITY 0x07
#define IPMI_EVENT_TYPE_PRESENT 0x08
#define IPMI_EVENT_TYPE_EN_DIS 0x09
#define IPMI_EVENT_TYPE_SENSOR_SPECIFIC 0x6F
#define IPMI_OEM_EVENT_TYPE_DEASSART 0xEF

#define IPMI_EVENT_DIR_ASSERT 0x00
#define IPMI_EVENT_DIR_DEASSERT 0x80

/* generic offset for IPMI_EVENT_TYPE_DEAS_ASSE */
#define IPMI_EVENT_OFFSET_DEASSERT 0x00
#define IPMI_EVENT_OFFSET_ASSERT 0x01

/* sensor-specific offset for IPMI_SENSOR_TYPE_PROCESSOR */
#define IPMI_EVENT_OFFSET_PROCESSOR_IERR 0x00
#define IPMI_EVENT_OFFSET_PROCESSOR_THERMAL_TRIP 0x01
#define IPMI_EVENT_OFFSET_PROCESSOR_FRB1 0x02
#define IPMI_EVENT_OFFSET_PROCESSOR_FRB2 0x03
#define IPMI_EVENT_OFFSET_PROCESSOR_FRB3 0x04
#define IPMI_EVENT_OFFSET_PROCESSOR_PRESENCE 0x07
#define IPMI_EVENT_OFFSET_PROCESSOR_MCERR 0x0B
#define IPMI_OEM_EVENT_OFFSET_MEM_RMCA 0x0D

/* sensor-specific offset for IPMI_SENSOR_TYPE_MEMORY */
#define IPMI_EVENT_MEMORY_CORRECT_ECC 0x00
#define IPMI_EVENT_MEMORY_UNCORRECT_ECC 0x01
#define IPMI_EVENT_MEMORY_PARITY 0x04

/* sensor-specific offset for IPMI_SENSOR_TYPE_CRITICAL_INT */
#define IPMI_EVENT_CRITICAL_INT_FP_NMI 0x00
#define IPMI_EVENT_CRITICAL_INT_SW_NMI 0x03
#define IPMI_EVENT_CRITICAL_INT_PERR 0x04
#define IPMI_EVENT_CRITICAL_INT_SERR 0x05
#define IPMI_EVENT_CRITICAL_INT_NFERR 0x08
#define IPMI_EVENT_CRITICAL_INT_FATAL_NMI 0x09
#define IPMI_EVENT_CRITICAL_INT_FERR 0x0A

/* sensor-specific offset for IPMI_SENSOR_TYPE_EVENT_LOG */
#define IPMI_EVENT_OFFSET_SEL_FULL 0x04
#define IPMI_EVENT_OFFSET_SEL_ALMOST_FULL 0x05

/* sensor-specific offset for IPMI_SENSOR_TYPE_POWER_SUPPLY */
//#define IPMI_EVENT_POWER_SUPPLY_FAILURE				0x01
//#define IPMI_EVENT_POWER_SUPPLY_PRE_FAILURE			0x02

/* sensor-specific offset for IPMI_SENSOR_TYPE_POWER_UNIT */
#define IPMI_EVENT_POWER_UNIT_POWER_OFF 0x00
#define IPMI_EVENT_POWER_UNIT_POWER_CYCLE 0x01
#define IPMI_EVENT_POWER_UNIT_AC_LOST 0x04

/* sensor-specific offset for IPMI_SENSOR_TYPE_BUTTON */
#define IPMI_EVENT_BTN_POWER 0x00
#define IPMI_EVENT_BTN_SELLP 0x01
#define IPMI_EVENT_BTN_RESET 0x02

/* sensor-specific offset for IPMI_SENSOR_TYPE_MANAGEMENT */
#define IPMI_EVENT_OFFSET_MANAGEMENT_OFF_LINE 0x02
#define IPMI_EVENT_OFFSET_MANAGEMENT_UNAVAILABLE 0x03 //Jerry130723

/* sensor-specific offset for IPMI_SENSOR_TYPE_VERSION_CHANGE */
#define IPMI_EVENT_OFFSET_SOFTWARE_FIRMWARE_CHANGE 0x07 //Jerry130913

/* sensor-specific offset for IPMI_SENSOR_TYPE_CPU_DIMM_HOT */
#define IPMI_OEM_EVENT_OFFSET_CPU_HOT 0x00
#define IPMI_OEM_EVENT_OFFSET_DIMM_HOT 0x01

/* sensor-specific offset for IPMI_SENSOR_TYPE_CPU_DIMM_VR_HOT */
#define IPMI_OEM_EVENT_OFFSET_CPU_VR_HOT 0x00
#define IPMI_OEM_EVENT_OFFSET_IO_VR_HOT 0x01
#define IPMI_OEM_EVENT_OFFSET_DIMM_ABC_VR_HOT 0x02
#define IPMI_OEM_EVENT_OFFSET_DIMM_DEF_VR_HOT 0x03

/* sensor-specific offset for IPMI_SENSOR_TYPE_SYS_STA */
#define IPMI_OEM_EVENT_OFFSET_SYS_THERMAL_TRIP 0x00
#define IPMI_OEM_EVENT_OFFSET_SYS_FIVR_FAULT 0x01
#define IPMI_OEM_EVENT_OFFSET_SYS_THROTTLE 0x02
#define IPMI_OEM_EVENT_OFFSET_SYS_PCHHOT 0x03
#define IPMI_OEM_EVENT_OFFSET_SYS_UV 0x04
//#define IPMI_OEM_EVENT_OFFSET_SYS_FASTTHROT           0x05
#define IPMI_OEM_EVENT_OFFSET_SYS_PMBUSALERT 0x05
#define IPMI_OEM_EVENT_OFFSET_SYS_HSCTIMER 0x06
#define IPMI_OEM_EVENT_OFFSET_SYS_FIRMWAREASSERT 0x07
#define IPMI_OEM_EVENT_OFFSET_SYS_OCPFAULT 0x08
#define IPMI_OEM_EVENT_OFFSET_SYS_SEVEREOCPFAULT 0x09
#define IPMI_OEM_EVENT_OFFSET_SYS_VRWATCHDOG 0x0A
#define IPMI_OEM_EVENT_OFFSET_SYS_VPPEVT 0x0B
#define IPMI_OEM_EVENT_OFFSET_SYS_DEVPWRGOODFAULT 0x0C
#define IPMI_OEM_EVENT_OFFSET_SYS_VCCIOFAULT 0x0D
#define IPMI_OEM_EVENT_OFFSET_SYS_SMI90s 0x0E
#define IPMI_OEM_EVENT_OFFSET_SYS_VCCIOOVFAULT 0x0F
#define IPMI_OEM_EVENT_OFFSET_SYS_FMTHROTTLE 0x10
#define IPMI_OEM_EVENT_OFFSET_SYS_MEMORY_THERMALTRIP 0x11
#define IPMI_OEM_EVENT_OFFSET_SYS_X8BOARDPWRFAILEVT 0x16
#define IPMI_OEM_EVENT_OFFSET_SYS_X16BOARDPWRFAILEVT 0x17

/* sensor-specific offset for IPMI_SENSOR_TYPE_SYS_BOOT_STA */
#define IPMI_OEM_EVENT_OFFSET_SLP_S4 0x00
#define IPMI_OEM_EVENT_OFFSET_SLP_S3 0x01
#define IPMI_OEM_EVENT_OFFSET_PCH_PWROK 0x02
#define IPMI_OEM_EVENT_OFFSET_SYS_PWROK 0x03
#define IPMI_OEM_EVENT_OFFSET_PLTRST 0x04
#define IPMI_OEM_EVENT_OFFSET_POST_CLT 0x05

/* sensor-specific offset for SENSOR_NUM_POWER_ERR */
#define IPMI_OEM_EVENT_OFFSET_SYS_PWROK_FAIL 0x01
#define IPMI_OEM_EVENT_OFFSET_PCH_PWROK_FAIL 0x02
#define IPMI_OEM_EVENT_OFFSET_EXP_PWRON_FAIL 0x03
#define IPMI_OEM_EVENT_OFFSET_EXP_PWROFF_FAIL 0x04

/* sensor-specific offset for CABLE_DETECTION */
#define IPMI_OEM_EVENT_OFFSET_SLOT1_SYSTEM_PRESENT 0x01
#define IPMI_OEM_EVENT_OFFSET_SLOT3_SYSTEM_PRESENT 0x03
#define IPMI_OEM_EVENT_OFFSET_SLOT1_SYSTEM_ABSENT 0x11
#define IPMI_OEM_EVENT_OFFSET_SLOT3_SYSTEM_ABSENT 0x13
#define IPMI_OEM_EVENT_OFFSET_SLOT1_CABLE_ABSENT 0x21
#define IPMI_OEM_EVENT_OFFSET_SLOT3_CABLE_ABSENT 0x23
#define IPMI_OEM_EVENT_OFFSET_SLOT1_INSERT_SLOT3 0x31
#define IPMI_OEM_EVENT_OFFSET_SLOT3_INSERT_SLOT1 0x33

enum ipmi_chassis_control_e {
	IPMI_CHASSIS_CTRL_POWER_DOWN,
	IPMI_CHASSIS_CTRL_POWER_UP,
	IPMI_CHASSIS_CTRL_POWER_CYCLE,
	IPMI_CHASSIS_CTRL_HARD_RESET,
	IPMI_CHASSIS_CTRL_DIAGNOSTIC_IRQ,
	IPMI_CHASSIS_CTRL_SOFT_SHUTDOWN
};

enum ipmi_chassis_boot_device_e {
	IPMI_CHASSIS_BOOT_DEV_NO,
	IPMI_CHASSIS_BOOT_DEV_PXE,
	IPMI_CHASSIS_BOOT_DEV_HARD,
	IPMI_CHASSIS_BOOT_DEV_HARD_SAFE,
	IPMI_CHASSIS_BOOT_DEV_DIAG,
	IPMI_CHASSIS_BOOT_DEV_CD,
	IPMI_CHASSIS_BOOT_DEV_SETUP
};

enum ipmi_chassis_identify_state_e {
	IPMI_CHASSIS_IDENTIFY_OFF,
	IPMI_CHASSIS_IDENTIFY_TEMPO,
	IPMI_CHASSIS_IDENTIFY_ON,
};

enum ipmi_system_restart_cause_e {
	IPMI_SYS_RESTART_UNKNOWN,
	IPMI_SYS_RESTART_CMD,
	IPMI_SYS_RESTART_RESET_BUTTON,
	IPMI_SYS_RESTART_POWER_BUTTON,
	IPMI_SYS_RESTART_WDT,
	IPMI_SYS_RESTART_OEM,
	IPMI_SYS_RESTART_POLICY,
	IPMI_SYS_RESTART_RESET_PEF,
	IPMI_SYS_RESTART_CYCLE_PEF,
	IPMI_SYS_RESTART_SOFT_RESET,
	IPMI_SYS_RESTART_RTC
};

enum ipmi_power_restore_policy_e {
	IPMI_POWER_RESTORE_OFF,
	IPMI_POWER_RESTORE_RETAIN,
	IPMI_POWER_RESTORE_ON
};

#endif
