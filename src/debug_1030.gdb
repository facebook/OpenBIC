###############################################################################
## usage:
## 1. start your GDB server
## 2. run "arm-none-eabi-gdb-py -x debug.gdb"
## 3. connect to the GDB server
##    GDB server is your local PC : (gdb) connect_local
##    GDB server is a remote desktop: (gdb) connect
###############################################################################

#set disassemble-next-line on
set print pretty on
set pagination off
set output-radix 16
#source tools/PyCortexMDebug/cmdebug/svd_gdb.py
#svd_load tools/cortex-m4-scb.svd

source tools/gdb/cpu.gdb

define connect
# Please modify the remote IP to your computer
	target remote 192.168.2.216:2331
	add-symbol-file bin/ast1030.elf
	layout src
	layout reg
	focus cmd
end

define connect_local
	target remote localhost:2331
	add-symbol-file bin/ast1030.elf
	
	# enter TUI mode by default, type : <ctrl+x> then <a> to exit TUI mode
	layout src
	layout reg
	focus cmd
end

define wdt_reset
	set *0x7e78501c = 0x1
	set *0x7e785004 = 1
	set *0x7e785008 = 0x4755
	set *0x7e78500c = 0x3
end

define i3c_reset
	set *0x7e78501c = (0x7 << 17)
	set *0x7e785004 = 1
	set *0x7e785008 = 0x4755
	set *0x7e78500c = 0x3
	#set *0x7e7a3034 = 0x3f
	#x/xw 0x7e7a3034
	#set *0x7e7a2034 = 0x3f
	#x/xw 0x7e7a2034
	printf "i3c reset done\n"
end

define init_reg
	set $r0 = 0
	set $r1 = 0
	set $r2 = 0
	set $r3 = 0
	set $r4 = 0
	set $r5 = 0
	set $r6 = 0
	set $r7 = (unsigned int)(&__StackTop) - 4
	set $r8 = 0
	set $r9 = 0
	set $r10 = 0
	set $r11 = 0
	set $r12 = 0
	set $primask = 0
	set $basepri = 0
	set $faultmask = 0
	set $control = 0
	set $fpscr = 0
	set $xpsr = 0x01000000
	set $sp = (unsigned int)(&__StackTop) - 4
	set $msp = (unsigned int)(&__StackTop) - 4
	set $psp = 0
	set $lr = 0xffffffff
	set $pc = Reset_Handler + 4
	set *(0xE000E010 + 0) = 0
	set *(0xE000E010 + 4) = 0
	set *(0xE000E010 + 8) = 0
end

define reload
	restore bin/ast1030_bic.bin binary 0x0
	symbol-file
	add-symbol-file bin/ast1030.elf
	init_reg
end
