/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/**************************************************************
 * This file is used by the interface to the Copley CAN card.
 * It's contents are generally not of much interest to anyone
 * who isn't writing a direct interface to that card.
 *************************************************************/
#ifndef _DEF_INC_COPLEY_INTERNAL
#define _DEF_INC_COPLEY_INTERNAL

/**************************************************************
 * The following structure is used to pass a CAN message 
 * frame to/from the CAN card.
 *************************************************************/
typedef struct
{
   int32_t  timeout;
   uint32_t timestamp;
   uint32_t id;
   uint32_t flags;
   uint8_t  data[8];
} CANCARD_MSG;

// These values are used in the flags member of the CANCARD_MSG structure
#define COPLEYCAN_CANFLG_LENGTH        0x0000000F   // Length of message data (bytes)
#define COPLEYCAN_CANFLG_RTR           0x00000010   // Set for RTR messages
#define COPLEYCAN_CANFLG_SENDNOTIFY    0x00000020   // For transmits, notify on success
#define COPLEYCAN_CANFLG_EXTENDED      0x00000040   // Set for extended messages

// The following command codes are currently defined.
#define CANCARD_CMD_OPENPORT           0x01      // Open a CAN channel (no data)
#define CANCARD_CMD_CLOSEPORT          0x02      // Close a CAN channel (no data)
#define CANCARD_CMD_SETBPS             0x03      // Set bitrate info
#define CANCARD_CMD_GETBPS             0x04      // Get bitrate info
#define CANCARD_CMD_GETVERSION         0x05      // Get firmware version info
#define CANCARD_CMD_SETPARAM           0x09      // Set a parameter
#define CANCARD_CMD_GETPARAM           0x0A      // Get a parameter
#define CANCARD_CMD_RECV_THRESH        0xF9      // Set the receive buffer interrupt threshold 
#define CANCARD_CMD_DRIVERVER          0xFE      // Get driver version
#define CANCARD_CMD_RESET              0xFF      // Reset the card

// The following response codes will be returned by the card
#define COPLEYCAN_ERR_OK                0x00       // No error
#define COPLEYCAN_ERR_UNKNOWN_CMD       0x01       // Passed command ID was unknown
#define COPLEYCAN_ERR_BAD_PARAM         0x02       // Illegal parameter passed
#define COPLEYCAN_ERR_PORT_OPEN         0x03       // Specified CAN port is already open
#define COPLEYCAN_ERR_PORT_CLOSED       0x04       // Specified CAN port is not open
#define COPLEYCAN_ERR_CARD_BUSY         0x05       // Card command area is busy
#define COPLEYCAN_ERR_INTERNAL          0x06       // Some sort of internal device failure
#define COPLEYCAN_ERR_TIMEOUT           0x07       // Card failed to respond to command
#define COPLEYCAN_ERR_SIGNAL            0x08       // Signal received by driver
#define COPLEYCAN_ERR_MISSING_DATA      0x09       // Not enough data was passed
#define COPLEYCAN_ERR_CMDMUTEX_HELD     0x0A       // The command mutex was being held
#define COPLEYCAN_ERR_QUEUECTRL         0x0B       // The CAN message queue head/tail was invalid
#define COPLEYCAN_ERR_FLASH             0x0C       // Failed to erase/program flash memory
#define COPLEYCAN_ERR_NOTERASED         0x0D       // Attempt to write firmware before erasing flash
#define COPLEYCAN_ERR_FLASHFULL         0x0E       // Too much data sent when programming flash
#define COPLEYCAN_ERR_UNKNOWN_IOCTL     0x0F       // Specified IOCTL code was unknown
#define COPLEYCAN_ERR_CMD_TOO_SMALL     0x10       // Command passed without required header
#define COPLEYCAN_ERR_CMD_TOO_BIG       0x11       // Command passed with too much data
#define COPLEYCAN_ERR_CMD_IN_PROGRESS   0x12       // Command already in progress on card
#define COPLEYCAN_ERR_CAN_DATA_LENGTH   0x13       // More then 8 bytes of data sent with CAN message
#define COPLEYCAN_ERR_QUEUE_FULL        0x14       // Transmit queue is full
#define COPLEYCAN_ERR_QUEUE_EMPTY       0x15       // Receive queue is full
#define COPLEYCAN_ERR_READ_ONLY         0x16       // Parameter is read only
#define COPLEYCAN_ERR_MEMORYTEST        0x17       // Memory read/write test failure
#define COPLEYCAN_ERR_ALLOC             0x18       // Memory allocation failure
#define COPLEYCAN_ERR_CMDFINISHED       0x19       // Used internally by driver
#define COPLEYCAN_ERR_DRIVER            0x1A       // Generic device driver error

// Bit rate codes
#define COPLEYCAN_BITRATE_USER                  0
#define COPLEYCAN_BITRATE_1000000               1
#define COPLEYCAN_BITRATE_800000                2
#define COPLEYCAN_BITRATE_500000                3
#define COPLEYCAN_BITRATE_250000                4
#define COPLEYCAN_BITRATE_125000                5
#define COPLEYCAN_BITRATE_100000                6
#define COPLEYCAN_BITRATE_50000                 7
#define COPLEYCAN_BITRATE_20000                 8

// CAN card parameter IDs
#define COPLEYCAN_PARAM_SERIAL                  0
#define COPLEYCAN_PARAM_MFGINFO                 1
#define COPLEYCAN_PARAM_PCIVOLT                 2
#define COPLEYCAN_PARAM_33VOLT                  3
#define COPLEYCAN_PARAM_25VOLT                  4
#define COPLEYCAN_PARAM_STATUS                  5
#define COPLEYCAN_PARAM_OPTIONS                 6
#define COPLEYCAN_PARAM_INTINHIBIT              10

// I/O Ctrl codes
#if defined( WIN32 )
#  include <winioctl.h>
#  define COPLEYCAN_IOCTL_CMD		  CTL_CODE( FILE_DEVICE_UNKNOWN, 0x800, METHOD_BUFFERED, FILE_ANY_ACCESS )
#  define COPLEYCAN_IOCTL_SENDCAN	  CTL_CODE( FILE_DEVICE_UNKNOWN, 0x801, METHOD_BUFFERED, FILE_ANY_ACCESS )
#  define COPLEYCAN_IOCTL_RECVCAN	  CTL_CODE( FILE_DEVICE_UNKNOWN, 0x802, METHOD_BUFFERED, FILE_ANY_ACCESS )
#elif defined( __QNX__ )
#  include <devctl.h>
#  define COPLEYCAN_IOCTL_TYPE            0xB7
#  define COPLEYCAN_IOCTL_CMD		  __DIOTF( COPLEYCAN_IOCTL_TYPE, 0x06, long[64]    )
#  define COPLEYCAN_IOCTL_SENDCAN	  __DIOT ( COPLEYCAN_IOCTL_TYPE, 0x04, CANCARD_MSG )
#  define COPLEYCAN_IOCTL_RECVCAN         __DIOTF( COPLEYCAN_IOCTL_TYPE, 0x05, CANCARD_MSG )
#else
#  include <linux/ioctl.h>
#  define COPLEYCAN_IOCTL_TYPE            0xB7
#  define COPLEYCAN_IOCTL_SENDCAN         _IOW ( COPLEYCAN_IOCTL_TYPE, 0x04, CANCARD_MSG )
#  define COPLEYCAN_IOCTL_RECVCAN         _IOWR( COPLEYCAN_IOCTL_TYPE, 0x05, CANCARD_MSG )
#  define COPLEYCAN_IOCTL_CMD             _IOWR( COPLEYCAN_IOCTL_TYPE, 0x06, long        )
#endif

#endif

