ENTRY(Reset_Handler)
MEMORY 
{   
	ram(rwx)	: ORIGIN = 0x00000000, LENGTH = 0x01000000
	ram_nc(rw)	: ORIGIN = 0x01000000, LENGTH = 0x01000000
} 


SECTIONS {	 
	.text :
	{
		_stext = .;
		KEEP(*(.vectors))
		__Vectors_End = .;
		__Vectors_Size = __Vectors_End - __Vectors;
		__end__ = .;

		KEEP(*(.sboot))

    		*(.text)

		KEEP(*(.init))
		KEEP(*(.fini))		    

		/* .ctors */
		*crtbegin.o(.ctors)
		*crtbegin?.o(.ctors)
		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
		*(SORT(.ctors.*))
		*(.ctors)

		/* .dtors */
 		*crtbegin.o(.dtors)
 		*crtbegin?.o(.dtors)
 		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
 		*(SORT(.dtors.*))
 		*(.dtors)
    		
    		*(.rodata*)
		KEEP(*(.eh_frame*))		    
		_end_text = .;
		_etext = .;
	} >ram

	.ARM.extab :
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} > ram

	__exidx_start = .;
	.ARM.exidx :
	{
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} > ram
	__exidx_end = .;
 
	/**
   	* Location counter can end up 2byte aligned with narrow Thumb code but
   	* __etext is assumed by startup code to be the LMA of a section in RAM
   	* which must be 4byte aligned 
   	*/
	__etext = ALIGN (4);

  	/* .data : AT (__etext) */
	.data :
	{
		__data_start__ = .;
		*(vtable)
		*(.data*)

		. = ALIGN(4);
		/* preinit data */
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP(*(.preinit_array))
		PROVIDE_HIDDEN (__preinit_array_end = .);

		. = ALIGN(4);
		/* init data */
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP(*(SORT(.init_array.*)))
		KEEP(*(.init_array))
		PROVIDE_HIDDEN (__init_array_end = .);


		. = ALIGN(4);
		/* finit data */
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP(*(SORT(.fini_array.*)))
		KEEP(*(.fini_array))
		PROVIDE_HIDDEN (__fini_array_end = .);

		KEEP(*(.jcr*))
		. = ALIGN(4);
		KEEP(*(.pinctrl_fun_tbl))
		KEEP(*(.pinctrl_sig_tbl))
		KEEP(*(.pinctrl_fun_desc))
		KEEP(*(.pinctrl_sig_desc))
		KEEP(*(.pinctrl_pin_desc))
		. = ALIGN(4);
		/* All data end */
		__data_end__ = .;
	} >ram

	__etext_nc = ALIGN (4);
	/* non-cached data */
	.data_nc : AT (__etext_nc)
    {
        . = ALIGN(4);
        __data_nc_start__ = .;
        *(.nocache.data*)
        . = ALIGN(4);
        __data_nc_end__ = .;
    } >ram_nc
 
	.bss (NOLOAD):
	{
    		. = ALIGN(4);
		__bss_start__ = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		__bss_end__ = .;
	} >ram

	/* non-cached bss */
	.bss_nc (NOLOAD):
	{
		. = ALIGN(4);
		__bss_nc_start__ = .;
		*(.nocache.bss*)
		. = ALIGN(4);
		__bss_nc_end__ = .;
	} >ram_nc

	/* non-cached heap */
	.heap_nc (NOLOAD):
	{
		__HeapNcBase = .;
		KEEP(*(.heap.nocache*))
		__HeapNcLimit = .;
	} > ram_nc
 
    . = ALIGN(8);
     
	.heap (NOLOAD):
	{
		__HeapBase = .;
		__end__ = .;
		end = __end__;
		KEEP(*(.heap*))
		__HeapLimit = .;
	} > ram


    	/* .stack_dummy section doesn't contains any symbols. It is only
	 * used for linker to calculate size of stack sections, and assign
	 * values to stack symbols later */
	.stack_dummy (NOLOAD):
	{
		_estack = .;
		KEEP(*(.stack*))
		_sstack = .;
	} > ram

	/* Set stack top to end of RAM, and stack limit move down by
	 * size of stack_dummy section */
	__StackTop = ORIGIN(ram) + LENGTH(ram);
	__StackLimit = __StackTop - SIZEOF(.stack_dummy);
	PROVIDE(__stack = __StackTop);

	/* Check if data + heap + stack exceeds RAM limit */
	ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")

	__ram_start = ORIGIN(ram);
	__ram_end = ORIGIN(ram) + LENGTH(ram);
	__ram_nc_start = ORIGIN(ram_nc);
	__ram_nc_end = ORIGIN(ram_nc) + LENGTH(ram_nc);
}
 
_end = .;
PROVIDE(end = .);

