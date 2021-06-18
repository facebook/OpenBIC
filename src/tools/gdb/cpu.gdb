define dump_ssp
	printf "CM3 ctrl     = 0x%08x\n", *0x7e6e2a00
	printf "I/DMEM start = 0x%08x\n", *0x7e6e2a04
	printf "IMEM end     = 0x%08x\n", *0x7e6e2a08
	printf "DMEM end     = 0x%08x\n", *0x7e6e2a0c
	printf "cache ctrl   = 0x%08x\n", *0x7e6e2a40
	printf "cache invld  = 0x%08x\n", *0x7e6e2a44
	printf "cache func   = 0x%08x\n", *0x7e6e2a48
end

define dehalt
	set halt = 0
end

define dump_scb
	p/x *(SCB_Type *)0xe000ed00
end

define dump_nvic
	p/x *(NVIC_Type *)0xe000e100
end

define dump_coredebug
	p/x *(CoreDebug_Type *)0x0xE000EDF0
end

define dump_systick
	p/x *(SysTick_Type *)0xE000E010
end

define core_reset
	#set ((SCB_Type *)0xe000ed00)->AIRCR = 0x05fa0001
	set ((SCB_Type *)0xe000ed00)->AIRCR = 0x05fa0004
end

define clear_nvic
	set ((NVIC_Type *)0xe000e100)->ICER[0] = 0xffffffff
	set ((NVIC_Type *)0xe000e100)->ICER[1] = 0xffffffff
	set ((NVIC_Type *)0xe000e100)->ICER[2] = 0xffffffff
	set ((NVIC_Type *)0xe000e100)->ICER[3] = 0xffffffff
	set ((NVIC_Type *)0xe000e100)->ICER[4] = 0xffffffff
	set ((NVIC_Type *)0xe000e100)->ICER[5] = 0xffffffff
	set ((NVIC_Type *)0xe000e100)->ICER[6] = 0xffffffff
	set ((NVIC_Type *)0xe000e100)->ICER[7] = 0xffffffff

	set ((NVIC_Type *)0xe000e100)->ICPR[0] = 0xffffffff
	set ((NVIC_Type *)0xe000e100)->ICPR[1] = 0xffffffff
	set ((NVIC_Type *)0xe000e100)->ICPR[2] = 0xffffffff
	set ((NVIC_Type *)0xe000e100)->ICPR[3] = 0xffffffff
	set ((NVIC_Type *)0xe000e100)->ICPR[4] = 0xffffffff
	set ((NVIC_Type *)0xe000e100)->ICPR[5] = 0xffffffff
	set ((NVIC_Type *)0xe000e100)->ICPR[6] = 0xffffffff
	set ((NVIC_Type *)0xe000e100)->ICPR[7] = 0xffffffff
end