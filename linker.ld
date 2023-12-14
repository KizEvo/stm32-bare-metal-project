ENTRY(Reset_Handler)

MEMORY {
 flash(rx) : ORIGIN = 0x08000000, LENGTH = 128k
 sram(rwx) : ORIGIN = 0x20000000, LENGTH = 20k
}

_estack = ORIGIN(sram) + LENGTH(sram); /* stack point is in the last address of sram */

SECTIONS {
	.vectors : { *(.vectors) } > flash	/* ARM CPU vectors table  */
	.text	 : { *(.text)    } > flash	/* Executable codes 	  */
	.rodata  : { *(.rodata)  } > flash	/* Read-only data section */
	
	.data : {
		_sdata = .;		/* Start address of sram */
		*(.data)
		_edata = .;		/* Address: *(.data) + sram */
	} > sram AT > flash /* Data section */
	
	_sizedata = LOADADDR(.data);
	
	.bss : {
		_sbss = .;
		*(.bss)
		*(COMMON)		/* All un-initialized input files */
		_ebss = .;
	} > sram			/* Zero-initialized data section */
}