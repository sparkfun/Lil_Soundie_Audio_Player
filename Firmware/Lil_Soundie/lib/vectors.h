// Some handler vectors not declared elsewhere

#ifndef VECTORS_H
#define VECTORS_H

#include "usblowlib.h"

void USBHandler(void);
void RealUSBHandler(void);
void DecodeSetupPacket(void);
void RealDecodeSetupPacket(void);
void MSCPacketFromPC(USBPacket *inPacket);
void RealMSCPacketFromPC(USBPacket *inPacket);

#endif
