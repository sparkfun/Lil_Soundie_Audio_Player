/**
   \file c-nand.s Startup module for programs booting from NAND-FLASH or UART.
	This startup does not touch stack pointer or other registers.
	This makes it possible to return to the caller.
	This is useful when the default player main loop can be used but some
	routines need to be patched. The user can perform the patches and
	return to the caller.
*/
#if 1 //def ASM
	.sect code,code
	.org 0x50
	.import _main
	jmpi _main

	.end
#endif
