/*
  Startup at fixed address that does not touch stack pointer or other
  registers. This makes it possible to return to the caller.
  This is useful when the default player main loop can be used but some
  routines need to be patched. The code can perform the patches and
  return to the caller.

  This version is fixed to support VS1000D Usb function calling
  by setting the return address to SCSI OK return after running
  the function. main() should do return 1; 
	
*/
	.sect code,code
	.org 0x50
	.import _main
	j _main
	ldc 0x833d,lr0
	
	.end
