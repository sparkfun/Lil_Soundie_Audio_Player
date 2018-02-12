/**
   \file usblowlib.h VS1000 low-level USB definitions.
 */
#ifndef USBLOWLIB_H

#define USBLOWLIB_H

#define AUDIO_ISOC_OUT_EP     0x01 /**< OUT endpoint used for audio. */
#define MSC_BULK_OUT_ENDPOINT 0x03 /**< OUT endpoint used for SCSI. */
#define MSC_BULK_IN_ENDPOINT  0x02 /**< IN endpoint used for SCSI. */

#define ENDPOINT_SIZE_0 64  /**< The endpoint size supported for IN. */
#define ENDPOINT_SIZE_1 512 /**< The max OUT packet size, mainly for audio. */

/*
 * Interrupts
 */
#define BRESET_INT   0x8000 /**< Bus reset interrupt */
#define SOF_INT      0x4000 /**< Start of Frame interrupt */
#define RX_INT       0x2000 /**< OUT packet received */
#define TX_HOLD_INT  0x1000 /**< Copied packet to IN registers */
#define TX_INT       0x0800 /**< Successfully transmitted packet */
#define NAK_SENT_INT 0x0400 /**< Packet NAKed */
#define SETUP_INFO   0x0080 /**< Setup packet */

/*
 * PID tokens
 */
#define P_SETUP   0x10
#define P_DATA    0x20
#define PID_SOF   0x05
#define PID_SETUP 0x0D
#define PID_IN    0x09
#define PID_OUT   0x01
#define PID_DATA0 0x03
#define PID_DATA1 0x0B
#define PID_ACK   0x02
#define PID_NACK  0x0A
#define PID_STALL 0x0E

/*
 * USB addresses
 */

#define USB_CONFIG  (USB_BASE)
#define USB_CONTROL (USB_BASE+1)
#define USB_STATUS  (USB_BASE+2)

#define USB_STF_BUS_RESET (1<<15)
#define USB_STF_SOF       (1<<14)
#define USB_STF_RX        (1<<13)
#define USB_STF_TX_READY  (1<<12)
#define USB_STF_TX_EMPTY  (1<<11)
#define USB_STF_NAK       (1<<10)
#define USB_STF_SETUP     (1<<7)
#define USB_STM_LAST_EP   (15<<0)

#define USB_RDPTR   (USB_BASE+3) /**< USB receive buffer read pointer. */
#define USB_WRPTR   (USB_BASE+4) /**< USB receive buffer write pointer. */

#define USB_EP_SEND0 (USB_BASE+8)
#define USB_EP_SEND1 (USB_BASE+9)
#define USB_EP_SEND2 (USB_BASE+10)
#define USB_EP_SEND3 (USB_BASE+11)
//#define USB_EP_SEND4 (USB_BASE+12)
//#define USB_EP_SEND5 (USB_BASE+13)
//#define USB_EP_SEND6 (USB_BASE+14)
//#define USB_EP_SEND7 (USB_BASE+15)

#define USB_EP_ST0 (USB_BASE+16)
#define USB_EP_ST1 (USB_BASE+17)
#define USB_EP_ST2 (USB_BASE+18)
#define USB_EP_ST3 (USB_BASE+19)
//#define USB_EP_ST4 (USB_BASE+20)
//#define USB_EP_ST5 (USB_BASE+21)
//#define USB_EP_ST6 (USB_BASE+22)
//#define USB_EP_ST7 (USB_BASE+23)

#define USB_STF_OUT_BULK (0<<14)
#define USB_STF_OUT_INT  (1<<14)
#define USB_STF_OUT_ISO  (3<<14)
#define USB_STF_OUT_ENABLE (1<<13)
#define USB_STF_OUT_STALL  (1<<12)
#define USB_STF_OUT_STALL_SENT (1<<11)
#define USB_STF_OUT_EP_SIZE (1<<8)
#define USB_STF_IN_BULK (0<<6)
#define USB_STF_IN_INT  (1<<6)
#define USB_STF_IN_ISO  (3<<6) /** \todo \bug TODO: check!*/
#define USB_STF_IN_ENABLE (1<<5)
#define USB_STF_IN_STALL (1<<4)
#define USB_STF_IN_STALL_SENT (1<<3)
#define USB_STF_IN_NACK_SENT (1<<2)
#define USB_STF_IN_EMPTY (1<<1)
#define USB_STF_IN_FORCE_NACK (1<<0)


#define RING_BUF_SIZE 1024

#ifdef ASM
/* Assembler-specific stuff */

#include <vsasm.h>

#else
/* Here you could put C-specific definitions, like typedefs, C macros, prototypes etc... */

/** Copies packet from USB receive memory ring buffer to X memory buffer.
    The data is in big-endian format, i.e. the high 8 bits of a word are
    the first byte, the low 8 bits the second byte.
    Does not touch USB peripheral registers.
    \param d Destination pointer.
    \param s Source pointer, must point to the USB receive memory.
    \param n The number of words to copy.
 */
auto void RingBufCopyX(register __i2 u_int16 *d,
		      register __i0 const u_int16 *s,
		      register __a0 u_int16 n);
//auto void RingBufCopyY(register __i2 __y u_int16 *d,
//		      register __i0 const u_int16 *s,
//		      register __a0 u_int16 n);

/** Holding space for one received USB packet. */
typedef struct usbpkt {
  u_int16 length;                          /**< packet length in bytes */
  u_int16 payload[(ENDPOINT_SIZE_1+1)>>1]; /**< packet data */
} USBPacket;


#define DT_LANGUAGES 0
#define DT_VENDOR 1
#define DT_MODEL 2
#define DT_SERIAL 3
#define DT_DEVICE 4
#define DT_CONFIGURATION 5

extern struct USBVARS {
  /** Descriptor Pointer Table
      Members are:
      - *stringDescriptor0
      - *stringDescriptor1
      - *stringDescriptor2,
      - *stringDescriptor3,
      - *deviceDescriptor,
      - *configurationDescriptor

      For others than configurationDescriptor, descriptor size is
      first octet of descriptor. 
  */
  const u_int16 *descriptorTable[6];

  /** Length of Configuration Descriptor. 
      (needed because configuration descriptor is actually
      a collection of many descriptors so the first octet
      does not specify length of the entire descriptor) */
  u_int16 configurationDescriptorSize;

  /** Holding space for one received USB packet. */
  USBPacket pkt;

  u_int32 totbytes; /**< Total transferred bytes. */

  /** Is an extra zero-length packet needed after transmission? */
  u_int16 ExtraZeroLengthPacketNeeded[4];

  /** Current USB Endpoint transmit buffer pointers */
  const u_int16 *XmitBuf[4];

  /** Current USB Endpoints' bytes left to transmit */  
  u_int16 XmitLength[4];

  /** Is endpoint ready to transmit new block? */
  u_int16 EPReady[4];// = {1,1,1,1};

  u_int16 lastSofTimeout; /**< Used for suspend detection, although a bit inaccurate. */

  u_int16 configuration;  /**< Stores current configuration. Only used for USB_REQUEST_GET_CONFIGURATION. */
  u_int16 interfaces;  /**< Stores current and alternate setting. Only used for USB_REQUEST_GET_INTERFACE. */
  u_int16 lastSofFill; /**< Audio buf fullness at last SOF */
  u_int32 lastSofTime; /**< When last SOF was received */
} USB;

/** Starts transmission of data in endpoint-size -sized parts.
    Internall calls USBContinueTransmission once to start the transfer.
    \param ep  The endpoint to use.
    \param buf Data to send.
    \param length Length of data to send in bytes.
    \param requestedLength Data size that was requested in bytes.
    \return 0 if successful, non-zero if endpoint was busy.
 */
int USBStartTransmission(u_int16 ep, const void *buf, 
			 u_int16 length, u_int16 requestedLength);
/** Continues sending a packet.
    \param ep The endpoint.
 */
void USBContinueTransmission(u_int16 ep);
/** Hook: Initializes USB descriptor table. Default: RealInitUSBDescriptors.
    \param initDescriptors 0=no init, 1=Mass Storage, 2=Audio
 */
void InitUSBDescriptors(u_int16 initDescriptors);
/** Initializes USB descriptor table.
    \param initDescriptors 0=no init, 1=Mass Storage, 2=Audio
 */
void RealInitUSBDescriptors(u_int16 initDescriptors);
/** Initializes USB subsystem.
    \param initDescriptors 0=no init, 1=Mass Storage, 2=Audio
 */
void InitUSB(u_int16 initDescriptors);
/** Resets an endpoint. Data toggles are also reset and possible IN
    packets waiting for transmit are discarded.
    \param ep 0 for control endpoint, 0x80|endpoint for IN.
 */
void USBResetEndpoint(register __c0 int ep);
/** Fetches a packet from USB receive FIFO. Also clears RX_INT and SETUP_INFO.
    \param packet pointer to the packet structure
    \return endpoint number
 */
u_int16 USBReceivePacket(USBPacket *packet);
/** Sends an empty packet to the control endpoint. Returns when the
    packet has been sent or a bus reset is received.
    \bug: currently is satisfied if any active IN is finished.
*/
void USBSendZeroLengthPacketToEndpoint0(void);
/** Hook: polled handling of USB packets. Default: RealUSBHandler.
 */
void USBHandler(void);
/** Polled handling of USB packets. Both Mass Storage and Audio are
    processed, but the descriptor decides which one the host uses.
 */
void RealUSBHandler();
/** Hook: handles setup packets. Default: DecodeSetupPacket.
 */
void DecodeSetupPacket(void);
/** Processes setup packets.
 */
void RealDecodeSetupPacket(void);
/** Checks if any setup packets are waiting and discards any packets
    that appear before it in the input queue.
 */
void USBCheckForSetupPacket(void);
/** Checks how many bytes of a packet are waiting to be sent.
    \param endpoint the endpoint to check.
    \return the number of bytes left
 */
u_int16 USBXmitLeft(u_int16 endpoint);
/** Sends (at least) one STALL to the specified endpoint.
 \param ep 0x80|endpoint for IN endpoints.
*/
void USBSingleStallEndpoint(register __c0 u_int16 ep);
/** Sets the endpoint to stall state.
 \param ep Endpoint number for OUT endpoint, 0x80|endpoint for IN endpoints.
*/
void USBStallEndpoint(register __c0 int ep);
/** Resets the stall state of an endpoint.
 \param ep 0x80|endpoint for IN endpoints.
*/
void USBResetStall(register __c0 int ep);
/** Returns the stall state of an endpoint.
 \param ep 0x80|endpoint for IN endpoints.
 \return non-zero if endpoint is stalled.
*/
u_int16 USBIsEndpointStalled(register int ep);
/** Swaps byte order */
u_int16 SwapWord(register __a1 u_int16 d);
/** Uses USB pull-up enable and USB_DN to detect if USB is attached.
    USB is attached if the USB pull-up is enabled.
    USB is not attached if USB_DN is high for every sampling in 20000 clock
    cycles, and is attached if USB_DN is low for even one sampling.
    \return 0 is USB is not attached, non-zero if USB is attached. */
auto u_int16 USBIsAttached(void);
/** Checks USB_DP and USB_DN. If both are high, the unit is probably detached.
    You MUST also use the USBWantsSuspend() function to see that there has
    not been SOF's for a while. Otherwise you will get false detections.
    \return 0 is USB is still attached, non-zero if it might be detached. */
auto u_int16 USBIsDetached(void); /*Snapshot only, USBWantsSuspend() required*/
/** Checks if there has been no SOF for 10..20ms.
    \bug USB suspend time is 3ms. Because of our software timer accurary we
         detect suspend when there has been no SOF for 10..20ms.
    \bug This routine requires that the unit is configured before returning non-zero.
    \return 0 if the communication is proceeding normally, non-zero if suspend state is detected.
*/
auto u_int16 USBWantsSuspend(void);


#define USB_MASS_STORAGE 1
#define USB_AUDIO 2

/** MSC */
/** Hook: handle a mass storage packet. Default: RealMSCPacketFromPC.
 */
void MSCPacketFromPC(USBPacket *setupPacket);
/** Handles a mass storage packet.
 */
void RealMSCPacketFromPC(USBPacket *setupPacket);
u_int16 MscSendCsw(u_int16 status);
void DiskProtocolError(char errorcode);


#define AUDIO_DELAY_FRAMES 12 /* 12*44.1 = 529.2 samples */
#define AUDIO_DELAY_FRAMES_STR "\x0c"
/** Called each time a packet is received to the audio endpoint.
    The function must be provided. */
void AudioPacketFromUSB(u_int16 *data, s_int16 words);

#endif
#endif
