#include "system.h"

#include <stdio.h> //Standard io
#include <stdlib.h> // VS_DSP Standard Library
#include <vs1000.h> // VS1000B register definitions
#include <vectors.h> // VS1000B vectors (interrupts and services)
#include <minifat.h> // Read Only Fat Filesystem
#include <mapper.h> // Logical Disk
#include <string.h> // memcpy etc
#include <player.h> // VS1000B default ROM player
#include <audio.h> // DAC output
#include <codec.h> // CODEC interface
#include <vsNand.h>
#include <mappertiny.h>
#include <usb.h>

#include <dev1000.h>

#include "gpioctrl.h"

extern struct CodecServices cs;
void puthex(u_int16 a);
/*
  We currently allow two GPIO controls:
  1) GPIO_MASK for binary-encoded file number, all-zero means idle.
     The playing file is changed when the value changes to another non-zero
     value while a file is playing. The file is repeated if the number
     remains at the GPIO pins after the playback has finished.

     The number of bits used can be configured from 1 to 8 (GPIO0[0:7])
     for 255 files when SD card is used, and from 1 to 13 (GPIO0[0:12])
     for 8191 files when SD card is not used.

  2) GPIO_PRIORITIES for one GPIO per file with priorities.
     Lower number GPIO pin number has priority over a higher number GPIO pin.
     A higher number GPIO pin does not interrupt the playing of a current
     file, even if the lower number GPIO pin is no longer set.
     The file is repeated if a file is selected by the GPIO pins after the
     playback has finished.

     The number of GPIO pins used can be configured from 1 to 8 (GPIO0[0:7])
     when SD card is used, and from 1 to 13 (GPIO0[0:12]) when SD card is
     not used.

  Playing indicator:
     - 
  See system.h for GPIO_NUMBERED_FILES definition
 */

#define GPIO_MASK       0x001f // Sel. file w/ GPIO0 pins in the mask
//#define GPIO_INVERTED   //default state is HIGH, buttons bring low
//#define GPIO_PRIORITIES 0x04ff // Sel. file w/ GPIO0 pins, lower has priority

//#define GPIO0_READY_INDICATOR    GPIO0_CLE
//#define GPIO0_PLAYING_INDICATOR  GPIO0_CLE /*CLE=GPIO0_12*/
/*Note: CLE is a bad choice, because it does not come to an audio module pin.*/
#define GPIO0_PLAYING_INDICATOR  (1<<6) //GPIO0_CLE /*CLE=GPIO0_12*/


#if defined(GPIO_MASK) && defined(GPIO_PRIORITIES)
err 'Both GPIO_MASK and GPIO_PRIORITIES can not be defined at the same time'
#endif



void GPIOCtrlIdleHook(void) {

    if (uiTrigger) {
        uiTrigger = 0;

        /*
          If you want the default firmware key-controls, use KeyScan();
        */

#ifdef GPIO0_PLAYING_INDICATOR
        /* We do it this way so that the logic can be in this file. */
        if (player.currentFile != -1) {
            /* high = playing */
            PERIP(GPIO0_SET_MASK) = GPIO0_PLAYING_INDICATOR;
        } else {
            /* low = waiting for GPIO */
            PERIP(GPIO0_CLEAR_MASK) = GPIO0_PLAYING_INDICATOR;
        }
#endif

  /*TURN GPIO into GPIO mode.*/
#ifdef GPIO_MASK
        /* Direct selection through GPIO pins. */
        {
            register s_int16 mask;
#ifdef GPIO_INVERTED
            /*default state is high, button pulls low to GND */
            PERIP(GPIO0_SET_MASK)   = GPIO_MASK | GPIO0_PULLUPS;
            PERIP(GPIO0_DDR) |= GPIO_MASK;  //Drive to 1
            for (mask = 0;mask<10000;mask++)
        	USEX(0);
            PERIP(GPIO0_DDR) &= ~GPIO_MASK; //To inputs
            for (mask = 0;mask<10000;mask++)
        	USEX(0);
            mask = ((PERIP(GPIO0_IDATA) ^ GPIO0_PULLUPS) & GPIO_MASK) ^ GPIO_MASK;
#else
            /*default state is low, button pulls high to IOVDD*/
            PERIP(GPIO0_CLEAR_MASK) = GPIO_MASK;
            PERIP(GPIO0_SET_MASK)   = GPIO_MASK & GPIO0_PULLUPS;
            PERIP(GPIO0_DDR) |= GPIO_MASK;  //Drive to 0
            for (mask = 0;mask<10000;mask++)
        	USEX(0);
            PERIP(GPIO0_DDR) &= ~GPIO_MASK; //To inputs
            for (mask = 0;mask<10000;mask++)
        	USEX(0);
            mask = (PERIP(GPIO0_IDATA) ^ GPIO0_PULLUPS) & GPIO_MASK;
            
#endif
//putch(mask);

            if (mask) {
		player.currentFile = player.nextFile = mask - 1;
		player.pauseOn = 0;
            } else {
               player.currentFile = 0xffffU;
	     }
        }
#endif /*GPIO_MASK*/
#ifdef GPIO_PRIORITIES
        /* Prioritized selection through GPIO pins. */
        {
            register u_int16 mask;
            PERIP(GPIO0_CLEAR_MASK) = GPIO_PRIORITIES;
            PERIP(GPIO0_SET_MASK)   = GPIO_PRIORITIES & GPIO0_PULLUPS;
            PERIP(GPIO0_DDR) |= GPIO_PRIORITIES;  //Drive to 0
            for (mask = 0;mask<10000;mask++)
        	USEX(0);
            PERIP(GPIO0_DDR) &= ~GPIO_PRIORITIES; //To inputs
            for (mask = 0;mask<10000;mask++)
        	USEX(0);
            mask = (PERIP(GPIO0_IDATA) ^ GPIO0_PULLUPS) & GPIO_PRIORITIES;
//putch(mask);
            if (mask) {
        	register s_int16 n = 0, m = GPIO_PRIORITIES;
        	while (m) {
        	    if (m & 1) {
        		if (mask & 1) {
        		    break;
        		}
        		n++;
        	    }
        	    m >>= 1;
        	    mask >>= 1;
        	}
        	if (
#if 1 /* one pin per file, lower GPIO numbers have priority */
        	    player.currentFile == -1 || n < player.currentFile
#else /* one pin per file */
        	    n != player.currentFile
#endif
        	    ) {
        	    player.currentFile = player.nextFile = n;
        	    cs.cancel = 1;
        	    player.pauseOn = 0;
        	}
            }
        }
#endif /*GPIO_PRIORITIES*/

#if !defined(GPIO_MASK) && !defined(GPIO_PRIORITIES)
        /* Buttons */
        //KeyScanNoUSB(0x1f); /* with feature button */
#endif
#ifdef USE_POWERBUTTON
        if (PERIP(SCI_STATUS) & SCISTF_REGU_POWERBUT) {
            /* In 2FIN2 switching on the uSD regulator affects powerbutton. */
            if (keyOld == (SCISTF_REGU_POWERBUT<<4)) {
        	if (++keyOldTime >= 3) { /* about 0.2 sec */
        	    PERIP(GPIO0_CLEAR_MASK) = GPIO0_SD_POWER;
        	    RealPowerOff();
        	} else {
        	    keyOldTime = 0;
        	}
            } else {
        	keyOld == (SCISTF_REGU_POWERBUT<<4);
            }
        } else {
            keyOld = 0;
        }
#endif
#ifndef DISABLE_USB
        if (USBIsAttached()) {
            cs.cancel = 1;
            player.pauseOn = 0;
        }
#endif
    }
}

void GPIOInit(void) {
  /*TURN GPIO into GPIO mode.*/
#ifdef GPIO_MASK
  PERIP(GPIO0_MODE) &= ~GPIO_MASK;
  PERIP(GPIO0_DDR)  &= ~GPIO_MASK; /* GPIO0[0:n] into inputs */
#endif /*GPIO_MASK*/
#ifdef GPIO_PRIORITIES
  PERIP(GPIO0_MODE) &= ~GPIO_PRIORITIES;
  PERIP(GPIO0_DDR)  &= ~GPIO_PRIORITIES; /* GPIO0[0:n] into inputs */
#endif /*GPIO_PRIORITIES*/

#ifdef GPIO0_PLAYING_INDICATOR
  PERIP(GPIO0_DDR) |= GPIO0_PLAYING_INDICATOR;
#endif
  player.nextFile = -1; //No file is being played when we start
  player.volume = -24; //Max volume
}
