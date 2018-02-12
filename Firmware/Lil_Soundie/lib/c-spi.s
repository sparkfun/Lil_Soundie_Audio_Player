/**
   \file c-spi.s Startup module for programs loaded from SPI EEPROM.
		 If the user program returns, execution continues from the
		 point just after the spi boot command.
*/
#ifdef ASM
	.sect code,code
	.org 0x0050
	ldc 0x1802,i6	//__stack+2
	.import _main
	j _main
	ldc 0x46e6,lr0	//For VS1000B only!
	//Return after the spiload calls

	.end
#endif
