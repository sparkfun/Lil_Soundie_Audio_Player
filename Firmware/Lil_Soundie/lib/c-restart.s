/**
   \file c-restart.s Startup module for flashing programs.
		 If the user program returns, execution restarts the firmware.
*/
#ifdef ASM
	.sect code,code
	.org 0x50
	.import _main
	ldc 0x1800/*__stack*/,i6
	j _main
	ldc 0x4000,lr0	//restart

	.end
#endif
