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
#svd_load tools/cortex-m3-scb.svd

source tools/gdb/cpu.gdb

define connect
# Please modify the remote IP to your computer
	target remote 192.168.2.216:2331
	add-symbol-file bin/ast2600.elf
	layout src
	layout reg
	focus cmd
end

define connect_local
	target remote localhost:2331
	add-symbol-file bin/ast2600.elf
	
	# enter TUI mode by default, type : <ctrl+x> then <a> to exit TUI mode
	layout src
	layout reg
	focus cmd
end

define reload
	restore bin/ast2600_ssp.bin binary 0x0
	symbol-file
	add-symbol-file bin/ast2600.elf
	set $sp = &__StackTop
	set $msp = &__StackTop	
	set $psp = &__StackTop
	set $r7 = 0
	set $lr = 0xffffffff
	set $pc = Reset_Handler
end