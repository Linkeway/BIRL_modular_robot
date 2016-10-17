/*************************************************************************
**    IXXAT Automation GmbH
**************************************************************************
**
**   $Workfile: Vci2.h $
**    Summary : Virtual-CAN-Interface
**              Virtal CAN interface for CAN applications.
**   $Revision: 29 $
**       State:
**       $Date: 12.02.03 9:52 $
**  Compiler  : Borland C++ Builder Ver. 4.0
**              Microsoft Visual C++ 6.0
**      Author: J.Stolberg, K. Oberhofer, U. Schmid
**
**************************************************************************
**
**  Remarks   : !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
**              !!! Use only for 32Bit Versions of VCI !!!
**              !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
**
**************************************************************************
**    all rights reserved
*************************************************************************/
#ifndef VCI2_DEF
#define VCI2_DEF

#include <windows.h>    // for HWND, WINAPI

/*************************************************************************
**    constants and macros
*************************************************************************/

#define VCI_SW_VERSION              0x0216          /* V02.16 */

#ifndef UINT8
  #define  UINT8  unsigned char
#endif

#ifndef UINT16
  #define  UINT16  unsigned short
#endif

#ifndef UINT32
  #define  UINT32  unsigned long
#endif

#ifndef INT32
  #define  INT32   signed long
#endif

/*
** Definition of calling attributes (VCI_CALLATTR)
*/

#define VCI_CALLATTR     WINAPI

/*
** Definition of call back attributes (VCI_CALLBACKATTR)
*/

#define VCI_CALLBACKATTR __cdecl       // no further attributes WIN32

/*
**  Values for the CAN mode
*/

#define VCI_11B         0              //  11 bit identifier mode
#define VCI_29B         1              //  29 bit identifier mode

#define VCI_LOW_SPEED   2              //  a low speed-busconnector is used
                                       //  (if provided by the hardware)

#define VCI_TX_ECHO     4              //  all sent CAN objects appear
                                       //  as received CAN objectS in the
                                       //  rx queues.

#define VCI_TX_PASSIV   8              //  CAN controller works as passive 
                                       //  CAN node (only monitoring) and
                                       //  therefore it does not send any
                                       //  acknowledge bit for CAN objects
                                       //  sent by another CAN node.

#define VCI_ERRFRM_DET  16             //  errorframes are detected and
                                       //  reported as CAN objects via
                                       //  the rx queues

/*
**  Types of error frames
*/
#define VCI_ERRFRM_NO   0              //  no error frame
#define VCI_ERRFRM_STUF 1              //  stuff error
#define VCI_ERRFRM_FORM 2              //  form error
#define VCI_ERRFRM_ACKN 3              //  acknowledge error
#define VCI_ERRFRM_BIT1 4              //  bit 1 error
#define VCI_ERRFRM_BIT0 5              //  bit 0 error
#define VCI_ERRFRM_CRC  6              //  CRC error


/*
**  Queue types
*/

#define VCI_TX_QUE      0
#define VCI_RX_QUE      1


/*
**  Buffer types
*/

#define VCI_RX_BUF      0
#define VCI_RMT_BUF     1

/*
**  Enabling or disabling of queue objects
*/

#define VCI_ACCEPT      1
#define VCI_REJECT      0


#define VCI_DACAPO      0
#define VCI_CONTINUE    1

/*
**  Table of the return values dependent on the type:
**
**            |  OK  | Error
**   ---------+------+-----------
**     signed | >=0  |   <0
**   ---------+------+-----------
**   unsigned |  >0  |    0
**            |      |
*/

/*
**  VCI return codes of the functions:
**  A user exceptionhandler will be called independent of the functions
**  return code, which can be used for error handling !
*/

#define  VCI_ERR        0              //  standard error value;
                                       //  Further error specification
                                       //  is delivered by the exception
                                       //  handler

#define  VCI_QUE_EMPTY  0              //  Rx queue empty

#define  VCI_QUE_FULL   0              //  Tx queue already full

#define  VCI_OLD        0              //  old buffer data

#define  VCI_OK         1              //  successfull completion


#define  VCI_HWSW_ERR   -1             //  function could not be performed
                                       //  because of hard or software errors

#define  VCI_SUPP_ERR   -2             //  function is not supported
                                       //  this way (support-error)

#define  VCI_PARA_ERR   -3             //  calling parameter(s) is (are)
                                       //  not correct or out of range

#define  VCI_RES_ERR    -4             //  resource error
                                       //  The resorce limit exceeded
                                       //  at creation of a queue or something
                                       //  else

#define VCI_QUE_ERR     -5             //  Queue overrun
                                       //  One or more objects couldn't be
                                       //  inserted into the queue and were lost.
                                       //  The last inserted object was marked
                                       //  with the 'Receive-Queue-Overrun'
                                       //  bit.

#define VCI_TX_ERR      -6             //  A CAN message couldn't be sent
                                       //  for a long time
                                       //  cable error, wrong baud rate etc.

/*
**  Baudrates from CAN in Automation  (CIA)
*/

#define VCI_10KB     0x31,0x1C
#define VCI_20KB     0x18,0x1C
#define VCI_50KB     0x09,0x1C
#define VCI_100KB    0x04,0x1C
#define VCI_125KB    0x03,0x1C
#define VCI_250KB    0x01,0x1C
#define VCI_500KB    0x00,0x1C
#define VCI_800KB    0x00,0x16
#define VCI_1000KB   0x00,0x14

/*
**  possibly supported IXXAT board types
*/

#define VCI_IPCI165           0   // iPC-I 165         ISA slot
#define VCI_IPCI320           1   // iPC-I 320         ISA slot
#define VCI_CANDY             2   // CANdy320          LPT port
#define VCI_PCMCIA            3   // tinCAN            pc card
//                            4   // unused
#define VCI_IPCI386           5   // iPC-I 386         ISA slot
#define VCI_IPCI165_PCI       6   // iPC-I 165         PCI slot
#define VCI_IPCI320_PCI       7   // iPC-I 320         PCI slot
#define VCI_CP350_PCI         8   // iPC-I 165         PCI slot
#define VCI_PMC250_PCI        9   // special hardware from PEP
#define VCI_USB2CAN          10   // USB2CAN           USB port
#define VCI_CANDYLITE        11   // CANdy lite        LPT port
#define VCI_CANANET          12   // CAN@net           ethernet
#define VCI_BFCARD           13   // byteflight Card   pc card
#define VCI_PCI04_PCI        14   // PC-I 04           PCI slot, passive
#define VCI_USB2CAN_COMPACT  15   // USB2CAN compact   USB port
#define VCI_PASSIV           50   // PC-I 03           ISA slot, passive
#define VCI_UNKNOWN         100   // last possible board type
                                  
typedef UINT32 VCI_BOARD_TYPE;    // have to typedef this way to be
                                  // compiler independent

/*
**  possibly supported IXXAT slot types
*/

#define VCI_c_DPRAM       0
#define VCI_c_PORT        1
#define VCI_PCCARD_SLOT   2
#define VCI_PCI_SLOT      3

typedef UINT32 VCI_SLOT_TYPE;          // have to typedef this way to be
                                       // compiler independent

/*
**  function identification numbers of the VCI_t_UsrExcHdlr parameter 'func_num'
*/

#define VCI_INIT                0
#define VCI_SEARCH_BOARD        1
#define VCI_PREPARE_BOARD       2
#define VCI_CANCEL_BOARD        3
#define VCI_TEST_BOARD          4
#define VCI_READ_BOARD_INFO     5
#define VCI_READ_BOARD_STATUS   6
#define VCI_RESET_BOARD         7
#define VCI_READ_CAN_INFO       8
#define VCI_READ_CAN_STATUS     9
#define VCI_INIT_CAN           10 
#define VCI_SET_ACC_MASK       11
#define VCI_RESET_CAN          12
#define VCI_START_CAN          13
#define VCI_RESET_TIME_STAMPS  14
#define VCI_CONFIG_QUEUE       15
#define VCI_ASSIGN_RX_QUE_OBJ  16
#define VCI_CONFIG_BUFFER      17
#define VCI_RECONFIG_BUFFER    18
#define VCI_CONFIG_TIMER       19
#define VCI_READ_QUE_STATUS    20
#define VCI_READ_QUE_OBJ       21
#define VCI_READ_BUF_STATUS    22
#define VCI_READ_BUF_DATA      23
#define VCI_TRANSMIT_OBJ       24
#define VCI_REQUEST_OBJ        25
#define VCI_UPDATE_BUF_OBJ     26
#define VCI_CCI_REQ_DATA       27

typedef UINT32 VCI_FUNC_NUM;           // have to typedef this way to be
                                       // compiler independent

/*
**  The following CAN controllers will be distinguished
*/

#define VCI_82C200            0x0
#define VCI_82527             0x1
#define VCI_81C90             0x2
#define VCI_81C92             0x3
#define VCI_SJA1000           0x4
#define VCI_TOUCAN            0x5
#define VCI_UNKNOWNCTRLTYPE   0xFFFFFFFF

typedef UINT32 VCI_CAN_TYPE;           // have to typedef this way to be
                                       // compiler independent

/*************************************************************************
**    data types
*************************************************************************/
#ifdef _MSC_VER
  #pragma pack(1)               // set alignment to 1 byte (Microsoft style)
#endif // _MSC_VER
#ifdef __BORLANDC__
#if (__BORLANDC__ < 0x460)
  #pragma -a-                   // set alignment to 1 byte (Borland C style)
#else
  #pragma option -a1            // set alignment to 1 byte (Borland C++ Builder style)
#endif
  #pragma alignment             // output alignment in compile window (Borland)
#endif // __BORLANDC__

/*
**  board type info struct
*/

typedef struct{
  VCI_BOARD_TYPE brd_type;
  VCI_SLOT_TYPE  slot_type;
  UINT8          o_irq_enabled;
}VCI_BOARD_TYPE_INFO;

/*
**  Informations, delivered from the interface board:
*/

typedef struct{
    UINT16      hw_version;     //  Hardware version e.g.: 01.00 as
                                //  hex value: 0x1000
    UINT16      fw_version;     //  Firmware version
    UINT16      dd_version;     //  device driver version
    UINT16      sw_version;     //  PC software version
    UINT8       can_num;        //  Number of supported CAN controllers
    UINT16      time_stamp_res; //  min. possible time stamp resolution of the
                                //  VCI-DLL in micro seconds (10E-6)
    UINT16      timeout_res;    //  min. possible timeout resolution of the
                                //  receive queues in ms
    UINT32      mem_pool_size;  //  memory available for buffers and queues
                                //  in bytes
    UINT8       irq_num;        //  installed irq number
    UINT16      board_seg;      //  used memory mapped board segment
    char        serial_num[16]; //  String e.g: "1234567890"
    char        str_hw_type[40];//  String e.g: "iPCI320 V1.00"
}VCI_BOARD_INFO;

/*
**  CAN-Controller information:
*/

typedef struct{
      VCI_CAN_TYPE can_type;    //  Type of CAN controller
      UINT8        bt0;         //  Value set for Baudrate ( bustiming reg 0)
      UINT8        bt1;         //  Value set for Baudrate ( bustiming reg 1)
      UINT32       acc_code;    //  Value set for acceptance filter (id code)
      UINT32       acc_mask;    //  Value set for acceptance filter
                                //  (relevance mask)
}VCI_CAN_INFO;

/*
**  Board status:
*/

typedef struct{
    UINT8 sts;                  //  Bit coded information (1 = True)
                                //  Bit0 : 
                                //  Bit1 :
                                //  Bit2 :
                                //  Bit3 :
                                //  Bit4 : CAN0 running
                                //  Bit5 : CAN1 running
                                //  Bit6 : CAN2 running
                                //  Bit7 : CAN3 running
    UINT8 cpu_load;             //  average of cpu load in %  */
}VCI_BRD_STS;

/*
**  CAN status:
*/

typedef struct{
    UINT8 sts;                  //  Bit coded information (1 = True)
                                //  Bit0 :
                                //  Bit1 :
                                //  Bit2 : RemoteQueue overrun
                                //  Bit3 : CAN TX pending
                                //  Bit4 : CAN Init mode
                                //  Bit5 : CAN Data overrun
                                //  Bit6 : CAN Error warning level
                                //  Bit7 : CAN Bus-Off status
    UINT8 bus_load;             //  average of bus load in %
}VCI_CAN_STS;

/*
**  Structure of a CAN object
*/

typedef struct{
    UINT32      time_stamp;     //  Timestamp for receive queue objects
    UINT32      id;             //  Identifier 11-/29-Bit
                                //  11 bit in low word, 29 bit complete
    UINT8       len:4;          //  number of data bytes (0-8)
    UINT8       rtr:1;          //  RTR-Bit: 0=Dataframe, 1=Remoteframe
    UINT8       res:3;          //  not used
                                //  len,rtr and res are located in one byte
                                //  len resides in the low nibble, rts follows
                                //  and so on
    UINT8       a_data[8];      //  Array for up to 8 data bytes
    UINT8       sts;            //  Bit coded information for
                                //  receive objects
                                //  Bit0-3 : 0 = no error
                                //           1 = stuff error
                                //           2 = form error
                                //           3 = acknowledge error
                                //           4 = bit 1 error
                                //           5 = bit 0 error
                                //           6 = CRC error
                                //  Bit4   :
                                //  Bit5   :
                                //  Bit6   :
                                //  Bit7   : 1 = Receive queue overrun
}VCI_CAN_OBJ;


#ifdef _MSC_VER
  #pragma pack()                // set alignment to standard (Microsoft style)
#endif // _MSC_VER
#ifdef __BORLANDC__
#if (__BORLANDC__ < 0x460)
  #pragma -a.                   // set alignment to standard (Borland C style)
#else
  #pragma option -a.            // set alignment to standard (Borland C++ Builder style)
#endif
  #pragma alignment             // output alignment in compile window (Borland)
#endif // __BORLANDC__

/*************************************************************************
**
**    Typename      : VCI_t_UsrRxIntHdlr
**    Description   : Type definition of a callback handler.
**                    If an interrupt of a receive queue shall be generated
**                    for the client application, a function of this type
**                    must be written by the user and the address must be
**                    passed in VCI_PrepareBoard.
**                    This callback handler will be used for 2 different
**                    interrupt mechanisms:
**                    1) Dispatching of alert messages
**                       The alert messages are passed to the function
**                       through a pointer parameter at function
**                       call.
**                    2) Event of a receive queue for multithreading
**                       applications. In this case the function call
**                       serves as a signal to the application for
**                       further processing (parameter count is 0)
**                    (see VCI_ConfigQueue for further reading).
**                    Attention : This function will be called within the
**                                interrupt in DOS and Windows 16 bit.
**                                Other VCI functions can't be called
**                                within this function!!
**
**                    A separat callback handler must be written for each
**                    interface board, if this functionality is used.
**
**    Parameters    : que_hdl (in)   - handle of the queue which initiated
**                                     the interrupt
**                    count (in)     - number of received objects
**                                     If count = 0, the call is a simple
**                                     signal and the data must be read
**                                     with 'VCI_ReadQueObj'
**                                     The behaviour of alert mode and
**                                     simple receive queue can be altered
**                                     by increasing or decreasing the
**                                     intlimit at 'VCI_ConfigQueue'
**                    p_obj (in)     - Pointer to the received objects
**                                     Int_level <= 13 in VCI_ConfiQueue
**                                     Please save the data by copying it
**                                     out of the interface board.
**
**    Returnvalue   : -
**
*************************************************************************/
typedef void (VCI_CALLBACKATTR *VCI_t_UsrRxIntHdlr)( UINT16        que_hdl
                                                   , UINT16        count
                                                   , VCI_CAN_OBJ  *p_obj);

/*************************************************************************
**
**    Description   : Type definition of a callback handler.
**                    If the error handling shall be used, a function of
**                    this type must be written by the user and the
**                    address of this function must be passed in
**                    VCI_PrepareBoard.
**                    This function will be called always, if a VCI
**                    function returns an error.
**                    (besides of this mechanism the VCI functions also
**                    provide error information by their return values).
**                    A separat callback handler must be written for each
**                    interface board, if this functionality is used.
**
**    Parameters    : func_num (in)   - Type name from the type enumeration
**                                      VCI_FUNC_NUM, which identifies the
**                                      failed function.
**                    err_code (in)   - returncode of the failed
**                                      VCI function
**                                      (VCI_SUPP_ERR, VCI_PARA_ERR, ...).
**                    ext_err (in)    - additional error description of the
**                                      error VCI_ERR.
**                    s (in)          - Error string (max. 40 characters
**                                      without control characters) with
**                                      the functionname of the failed
**                                      function and a further
**                                      description of the error
**    Returnvalue   : -
**
*************************************************************************/
typedef void (VCI_CALLBACKATTR * VCI_t_UsrExcHdlr)( VCI_FUNC_NUM func_num
                                                  , INT32        err_code
                                                  , UINT16       ext_err
                                                  , char        *s );

/*************************************************************************
**
**    Description   : Type definition of a callback handler used to display
**                    strings during the board initialization.
**                    If the string display shall be used, a function with
**                    this call type must be written by the user and
**                    the address must be passed in VCI_PrepareBoard.
**                    Zero terminated ASCII strings with max. 40 characters
**                    (without control characters) will be given.
**                    Each string represents one diplay row.
**
**    Parameters    : s (in)  - Zero terminated string with max. 40
**                              characters
**    Returnvalue   : -
**
*************************************************************************/
typedef void (VCI_CALLBACKATTR * VCI_t_PutS)(char  * s);

/*************************************************************************
**    function prototypes
*************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/*************************************************************************
**
**    Function      : VCI_Get_LibType
**
**    Description   : Determines the different supported interface boards.
**                    VCI_Get_LibType,VCI_GetBrdNameByType and
**                    VCI_GetBrdTypeByName can be used to create a board
**                    selection list without knowledge of the supported
**                    boards in the application.
**                    Each VCI version can support other combinations of
**                    the following boardtypes
**    Parameters    : -
**    Returnvalue   : Bit field
**                    Bit 0:  iPC-I 165
**                    Bit 1:  iPC-I 320
**                    Bit 2:  CANdy
**                    Bit 3:  tinCAN
**                    Bit 5:  iPC-I 386
**                    Bit 6:  iPC-I 165 PCI
**
*************************************************************************/
UINT32  VCI_CALLATTR VCI_Get_LibType();


/*************************************************************************
**
**    Function    : VCI_GetBrdTypeInfo
**                  
**    Description : (to be defined)
**    Parameters  : boardtype (in): a boardtype number from
**                                    enum VCI_BOARD_TYPE
**                  psBrdTypeInfo : return struct of type
**                                    VCI_BOARD_TYPE_INFO
**                  
**    Returnvalues: VCI_OK      : function successfull completed
**                  VCI_ERR     : boardtype is not supported
**                  
*************************************************************************/
INT32 VCI_CALLATTR VCI_GetBrdTypeInfo ( VCI_BOARD_TYPE       boardtype
                                      , VCI_BOARD_TYPE_INFO *psBrdTypeInfo);


/*************************************************************************
**
**    Function      : VCI_GetBrdNameByType
**
**    Description   : Delivers a typename in form of a string
**                    for the given boardtype number (max. 20 characters)
**    Parameters    : boardtype (in)    : a boardtype number from
**                                        enum VCI_BOARD_TYPE
**                    sz_boardname (out): string of the boardtype
**                                        "CANdy"
**                                        "tinCAN"
**                                        "iPC-I 165"
**                                        "iPC-I 320"
**                                        "iPC-I 386"
**                                        "iPC-I 165 PCI"
**
**    Returnvalue   : VCI_OK      : function successfull completed
**                    VCI_PARA_ERR: stringpointer not valid (NULL)
**                    VCI_SUPP_ERR: this hardware is not supported
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_GetBrdNameByType( VCI_BOARD_TYPE  boardtype
                                       , char           *sz_boardname);


/*************************************************************************
**
**    Function      : VCI_GetBrdTypeByName
**
**    Description   : Delivers the boardtype for the given typename
**                    string (max. 20 characters)
**    Parameters    : sz_boardname (in): string of the boardtype from
**                                       VCI_GetBrdNameByType
**                                       "CANdy"
**                                       "tinCAN"
**                                       "iPC-I 165"
**                                       "iPC-I 320"
**                                       "iPC-I 386"
**                                       "iPC-I 165 PCI"
**
**    Returnvalue   : >= 0        : boardtype identifier
**                    VCI_PARA_ERR: stringpointer not valid (NULL)
**                    VCI_SUPP_ERR: this hardware is not supported
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_GetBrdTypeByName(char *sz_boardname);


/*************************************************************************
**
**    Function      : VCI_Init
**
**    Description   : Initialization of the VCI library structures
**                    (without board intialization).
**
**                    !!!!            WINDOWS VERSION                  !!!!!
**                    !!!! Use only for development with interpreter   !!!!!
**                    !!!! environments. All registered boards will be !!!!!
**                    !!!! resetted and all control structures will be !!!!!
**                    !!!! cleared.                                    !!!!!
**                    !!!!            WINDOWS VERSION                  !!!!!
**
**                    DOS: The function must be called before usage of
**                    the library. Already initialized boards will be
**                    resetted and deleted in all structures (handle will
**                    become invalid).
**    Parameters    :
**    Returnvalue   :
**
*************************************************************************/
void VCI_CALLATTR VCI_Init(void);


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                         Board installation functions
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*************************************************************************
**
**    Function      : VCI_SearchBoard
**
**    Description   : VCI_SearchBoard searches for the given board
**                    starting at the given location segment with the
**                    given IRQ number. If the given address and interrupt
**                    is correct, the parameters will be returned unchanged.
**                    The board will be searched, if there is no board at
**                    the given address.
**                    To detect the boards a reset will be performed at each
**                    possible address of the board.
**
**                    !!!!               ATTENTION                   !!!!!
**                    !!!! Usage for installation purposes only.     !!!!!
**                    !!!! It is useful, if the board can't be       !!!!!
**                    !!!! initialized because of wrong address or   !!!!!
**                    !!!! IRQ presettings on the board or in the    !!!!!
**                    !!!! system database                           !!!!!
**                    !!!!               ATTENTION                   !!!!!
**
**                    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
**                    !!!! Removed, only valid for DOS version of VCI !!!!
**                    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
**
**    Parameters    : board_type (in)      - VCI_IPCI320, VCI_IPCI165,
**                                           VCI_CANDY, VCI_PCMCIA, ....
**                    p_location (in/out)  - Pointer to the address segment,
**                                           the LPT number or COM number,
**                                           as start point for the search.
**                    p_irq_num (in/out)   - Pointer to the used IRQ number
**                                           of the board
**                                           (0->Interrupt will be searched)
**
**    Returnvalue   : VCI return codes.
**
*************************************************************************/

// removed, because it should not be used in the windows version !
//
INT32 VCI_CALLATTR VCI_SearchBoard( VCI_BOARD_TYPE  board_type
                                  , UINT8          *p_irq_num
                                  , UINT16         *p_location);



/*************************************************************************
**
**    Function      : VCI_PrepareBoard (mandatory)
**
**                    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
**                    !! This function is provided for compatibility with 
**                    !! VCI prior to version 1.50 and has the same 
**                    !! functionality.
**                    !! If you want to profit from the features of the 
**                    !! new VCI_V2 then have a look at the corresponding
**                    !! function prefixed with VCI2_??????????.
**                    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
**
**    Description   : All important presettings and initializations will
**                    be performed in VCI_PrepareBoard.
**                    This includes the board selection, the download and
**                    start of the firmware, registration of application
**                    callback handlers and interrupt registration
**                    The board will be resetted and the CAN controller
**                    will be set into init mode.
**                    A board handle will be returned, which must be used
**                    to perform further operations with this interface
**                    board. The handles will be returned in an ascending
**                    order (0,1,2,...n).
**                    If a zero is given for the IRQ number, the VCI will
**                    work without interrupt (if the board can work without).
**
**    Parameters    : board_type (in) -
**                      VCI_IPCI320, VCI_IPCI165, VCI_CANDY,
**                      VCI_PCMCIA, VCI_PCI,....
**
**                    location (in)  -
**                      Addess segment/LPT number/COM number
**                      /Socket number of the board.
**
**                      This parameter identifies the location at which the
**                      card could be found.
**                      It depends on the slot_type the card uses:
**
**                      ISA slot
**                        For isa slot cards in this parameter has to provide
**                        the segment address at which the card is mapped
**                        (see jumper settings of the isa card).
**                        If you mapped the card to 0xd000 you have to use the
**                        same value in this parameter.
**
**                      LPT port (CANdy)
**                        For LPT port adapters (only CANdy at the moment) you 
**                        have to specify the PORT index in this parameter, 
**                        e.g. the value 1 for LPT1, 2 for LPT2 and so on. 
**
**                      PC CARD (PCMCIA, tinCAN V2)
**                        For PC Card adapters the parameter "location" provides 
**                        the slot number the adapter is plugged into. Use the 
**                        value 0 for Slot 1 and the value 1 for Slot 2.
**
**                      PCI slot
**                        For PCI adapters you have to provide the SLOT index 
**                        in the the parameter "location".
**                        If you have only one PCI card installed this parameter 
**                        has to be set to 0. If you have more than one 
**                        PCI CAN-Interface card installed in your system they 
**                        are distinguished by using different SLOT indexes.
**                        Due to differences in the operating system the 
**                        number scheme is different for Win9x and WinNT.
**
**                        For example you have four PCI CAN-Interface cards installed,
**                        two IPCI 165 PCI and two IPCI 320 PCI.
**
**                        On Win9x the numbering is (for example):
**                          IPCI 320 PCI    - SlotNr:  0
**                          IPCI 320 PCI    - SlotNr:  1
**                          IPCI 165 PCI    - SlotNr:  0       
**                          IPCI 165 PCI    - SlotNr:  1
**                        The card is determined by the board type and the slot number.
**                        The ordering of the cards is determined by the device 
**                        type and the order the bios scans the PCI Devices.
**
**                        On WinNT on the same machine the numbering is (for example):
**                          IPCI 320 PCI    - SlotNr:  0
**                          IPCI 165 PCI    - SlotNr:  1
**                          IPCI 320 PCI    - SlotNr:  2
**                          IPCI 165 PCI    - SlotNr:  3
**                        The ordering of the cards is determined only by the order
**                        the bios scans the PCI Devices.
**  
**                    irq_num (in)    - 
**                      used board irq
**
**                      This parameter also depends on  the slot type the card uses.
** 
**                      ISA slot && LPT port
**                        the user have to provide the correct IRQ number in this 
**                        parameter.
** 
**                      PC CARD && PCI
**                        for this slot types, the IRQ is determined by the BIOS (PCI)
**                        or by the operating system (PCMCIA) on startup.
**                        Because of this the value the user sets for the parameter
**                        irq_num is a dummy value. 
**                        !!! The only restriction is that it should be not zero !!!
**                        The real interrupt used is then determined by the VCI from the
**                        system settings.
**
**                    fp_puts         - Pointer to callback function used
**                                      for displaying error
**                                      and status messages.
**                                      (NULL -> no callback is used)
**                    fp_int_hdlr     - Pointer to callback function used
**                                      by the interrupt service routine.
**                                      Used for processing of receive objects.
**                                      (NULL -> no callback is used)
**                    fp_exc_hdlr     - Pointer to callback function used as
**                                      exception handler for handling of
**                                      errors.
**                                      (NULL -> no callback is used)
**
**    Returnvalue   : >=0 - Board handle
**                     <0 - VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_PrepareBoard( VCI_BOARD_TYPE     board_type
                                   , UINT16             location
                                   , UINT8              irq_num
                                   , VCI_t_PutS         fp_puts
                                   , VCI_t_UsrRxIntHdlr fp_int_hdlr
                                   , VCI_t_UsrExcHdlr   fp_exc_hdlr);


/*************************************************************************
**
**    Function      : VCI2_PrepareBoard (mandatory)
**
**                    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
**                    !!!! only valid for VCI_V2                      !!!!
**                    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
**
**    Description   : All important presettings and initializations will
**                    be performed in VCI_PrepareBoard.
**                    This includes the board selection, the download and
**                    start of the firmware, registration of application
**                    callback handlers and interrupt registration
**                    The board will be resetted and the CAN controller
**                    will be set into init mode.
**                    A board handle will be returned, which must be used
**                    to perform further operations with this interface
**                    board. The handles will be returned in an ascending
**                    order (0,1,2,...n).
**
**    Parameters    : board_type (in) -
**                      VCI_IPCI320, VCI_IPCI165, VCI_CANDY,
**                      VCI_PCMCIA, VCI_PCI,....
**                      This parameter is only used for consistency checking.
**                      If the board that is registered a positon board_index 
**                      (see next parameter) is not of type board_type, then the
**                      call to this function returns invalid handle.
**
**                    board_index (in)  -
**                      Index number at which the board is registered.
**                      
**                      To get the valid index numbers you have to use the 
**                      functions from the XAT00Reg.DLL delivered with the
**                      VCI_V2.
**                      See the example programs and the documentation there.
**  
**                    s_addinfo (in)    - 
**                      Pointer to buffer with additional information 
**                      (max 256 Bytes ) from the hardware selection dialog box 
**                      (see XAT11Reg.dll).
**                      Set this parameter to 0 if you don't need additional 
**                      information for the hardware to perparewant to receive
**                      this information.
**
**                    b_addLength (in)  -
**                      length of buffer with additional information
**
**                    fp_puts         - Pointer to callback function used
**                                      for displaying error
**                                      and status messages.
**                                      (NULL -> no callback is used)
**                    fp_int_hdlr     - Pointer to callback function used
**                                      by the interrupt service routine.
**                                      Used for processing of receive objects.
**                                      (NULL -> no callback is used)
**                    fp_exc_hdlr     - Pointer to callback function used as
**                                      exception handler for handling of
**                                      errors.
**                                      (NULL -> no callback is used)
**
**    Returnvalue   : >=0 - Board handle
**                     <0 - VCI return codes.
**
*************************************************************************/
// new function only in VCI_V2
//
#if (VCI_SW_VERSION >= 0x0150)
INT32 VCI_CALLATTR VCI2_PrepareBoard( VCI_BOARD_TYPE     board_type
                                    , UINT16             board_index
                                    , char              *s_addinfo
                                    , UINT8              b_addLength
                                    , VCI_t_PutS         fp_puts
                                    , VCI_t_UsrRxIntHdlr fp_int_hdlr
                                    , VCI_t_UsrExcHdlr   fp_exc_hdlr);
#endif // (VCI_SW_VERSION >= 0x0150)


/*************************************************************************
**
**    Function      : VCI_PrepareBoardMsg (mandatory)
**
**                    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
**                    !! This function is provided for compatibility with 
**                    !! VCI prior to version 1.50 and has the same 
**                    !! functionality.
**                    !! If you want to profit from the features of the 
**                    !! new VCI_V2 then have a look at the corresponding
**                    !! function prefixed with VCI2_??????????.
**                    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
**
**    Description   : does the same like 'VCI_PrepareBoard' except of
**                    receive event information.  A windows message will
**                    be sent instead of a callback to fp_int_hdlr.
**                    Therefore 'fp_int_hdlr' was replaced by
**                    'msg_rx_int_hdlr'.
**
**    Parameters    : board_type (in) - see desc. of VCI_PrepareBoard
**                    location (in)   - see desc. of VCI_PrepareBoard
**                    irq_num (in)    - see desc. of VCI_PrepareBoard
**                    fp_puts         - Callback function to display error
**                                      and status messages.
**                                      (NULL -> no callback is used)
**                    msg_rx_int_hdlr - Messagevalue as receive indication.
**                                      (0 -> no message will be sent)
**                    fp_exc_hdlr     - Callback function pointer of the
**                                      Exceptionhandler for treatment of
**                                      errors.
**                                      (NULL -> no callback is used)
**                    apl_handle      - Windowhandle of the client application
**
**    Returnvalue   : >=0 - Boardhandle
**                     <0 - VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_PrepareBoardMsg( VCI_BOARD_TYPE   board_type
                                      , UINT16           location
                                      , UINT8            irq_num
                                      , VCI_t_PutS       fp_puts
                                      , UINT32           msg_rx_int_hdlr
                                      , VCI_t_UsrExcHdlr fp_exc_hdlr
                                      , HWND             apl_handle);


/*************************************************************************
**
**    Function      : VCI2_PrepareBoardMsg (mandatory)
**
**                    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
**                    !!!! only valid for VCI_V2                      !!!!
**                    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
**
**    Description   : does the same like 'VCI_PrepareBoard' except of
**                    receive event information.  A windows message will
**                    be sent instead of a callback to fp_int_hdlr.
**                    Therefore 'fp_int_hdlr' was replaced by
**                    'msg_rx_int_hdlr'.
**
**    Parameters    : board_type (in)   - see desc. of VCI2_PrepareBoard
**                    board_index (in)  - see desc. of VCI2_PrepareBoard
**                    s_addinfo (in)    - see desc. of VCI2_PrepareBoard
**                    b_addLength (in)  - see desc. of VCI2_PrepareBoard
**                    fp_puts           - Callback function to display error
**                                        and status messages.
**                                        (NULL -> no callback is used)
**                    msg_rx_int_hdlr   - Messagevalue as receive indication.
**                                        (0 -> no message will be sent)
**                    fp_exc_hdlr       - Callback function pointer of the
**                                        Exceptionhandler for treatment of
**                                        errors.
**                                        (NULL -> no callback is used)
**                    apl_handle        - Windowhandle of the client application
**
**    Returnvalue   : >=0 - Boardhandle
**                     <0 - VCI return codes.
**
*************************************************************************/
// new function only in VCI_V2
//
#if (VCI_SW_VERSION >= 0x0150)
INT32 VCI_CALLATTR VCI2_PrepareBoardMsg ( VCI_BOARD_TYPE     board_type
                                        , UINT16             board_index
                                        , char              *s_addinfo
                                        , UINT8              b_addLength
                                        , VCI_t_PutS         fp_puts
                                        , UINT32             msg_rx_int_hdlr
                                        , VCI_t_UsrExcHdlr   fp_exc_hdlr
                                        , HWND               apl_handle);
#endif // (VCI_SW_VERSION >= 0x0150)

/*************************************************************************
**
**    Function      : VCI_CancelBoard
**
**    Description   : A board registered with 'VCI_PrepareBoard' can be
**                    deregistered with 'VCI_CancelBoard'. This includes
**                    the reset of the hardware and deinstallation of the
**                    possibly used interrupt.
**    Parameters    : board_hdl (in) - board identifcation handle.
**
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_CancelBoard(UINT16 board_hdl);


/*************************************************************************
**
**    Function      : VCI_TestBoard
**
**    Description   : The board performs a selftest.
**                    After this test the board will be reseted and the
**                    CAN controller will be in init mode.
**
**    Parameters    : board_hdl (in) - board identifcation handle
**                    fp_puts        - pointer to callback function used for
**                                     displaying error  and status messages
**                                     during the test
**                                     (NULL -> no status display)
**
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
UINT8 VCI_CALLATTR VCI_TestBoard( UINT16     board_hdl
                                , VCI_t_PutS fp_puts);

/*************************************************************************
**
**    Function      : VCI_ReadBoardInfo
**
**    Description   : fills an information structure with information
**                    about the board
**                    VCI_BOARD_INFO:
**                    - Hardware version e.g.: 01.00 as hex value: x1000
**                    - Firmware version
**                    - Device driver version
**                    - PC software version (DLL or lib)
**                    - Number of supported CAN controllers
**                    - min. possible time stamp resolution of the
**                      VCI-DLL in micro seconds (10E-6)
**                    - min. possible timeout resolution of the receive
**                      queues in ms
**                    - memory available for buffers and queues in bytes
**                    - installed irq number
**                    - used memory mapped board segment
**                    - String e.g: "1234567890"
**                    - String e.g: "iPCI320 V1.00"
**
**
**    Parameters    : board_hdl (in) - board identifcation handle
**                    p_info (out)   - Pointer to the info struct.
**
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_ReadBoardInfo( UINT16          board_hdl
                                  , VCI_BOARD_INFO *p_info);

/*************************************************************************
**
**    Function      : VCI_ReadBoardStatus
**
**    Description   : fills a structure of the type VCI_BRD_STS:
**                    - Status (0 = OK).
**                    - CPU load in % (0-100).
**
**    Parameters    : board_hdl (in) - board identifcation handle
**                    p_sts (out)    - Pointer to the status structure
**
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_ReadBoardStatus( UINT16       board_hdl
                                     , VCI_BRD_STS *p_sts);

/*************************************************************************
**
**    Function      : VCI_ResetBoard
**
**    Description   : Resets the soft und hardware of the interface board.
**                    The CAN controllers are in init mode after this
**                    function is executed. So te board and the controllers
**                    must be initialized again afterwards.
**
**    Parameters    : board_hdl (in) - board identifcation handle
**
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_ResetBoard(UINT16 board_hdl);


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                        CAN installation functions
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*************************************************************************
**
**    Function      : VCI_ReadCanInfo
**
**    Description   : Is used to get information about the CAN
**                    controller(s). It fills a structure of the type
**                    VCI_BOARD_INFO:
**                    - Type of CAN controller
**                    - Values for baudrate und acceptance filter
**
**    Parameters    : board_hdl (in) - board identifcation handle
**                    can_num (in)   - number of the controller (0..n).
**                    p_info (out)   - Pointer to VCI_BOARD_INFO struct
**
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_ReadCanInfo( UINT16            board_hdl
                                  , UINT8             can_num 
                                  , VCI_CAN_INFO     *p_info);

/*************************************************************************
**
**    Function      : VCI_ReadCanStatus
**
**    Description   : Reads the status of the given CAN controller and
**                    the connected software into a VCI_CAN_STS structure.
**
**    Parameters    : board_hdl (in) - board identifcation handle
**                    can_num (in)   - number of the controller (0..n).
**                    p_sts (out)    - Pointer to VCI_CAN_STS datas
**
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_ReadCanStatus( UINT16       board_hdl
                                    , UINT8        can_num
                                    , VCI_CAN_STS *p_sts);

/*************************************************************************
**
**    Function      : VCI_InitCan
**
**    Description   : Initialization of the timing and output control
**                    register of the CAN-Controller.
**                    The given CAN-Controller will be resetted and
**                    remains in init mode and must be started again with
**                    VCI_StartCan. The values must be used appropriatly
**                    to the settings for the 80C200. The values will be
**                    translated internally for other CAN controller.
**
**    Parameters    : board_hdl (in) - board identifcation handle
**                    can_num (in)   - number of the controller (0..n).
**                    bt0 (in)       - Value for the bus timing register 0
**                    bt1 (in)       - Value for the bus timing register 1
**                    mode (in)      - 11bit/29bit mode (VCI_11B, VCI_29B),
**                                   - VCI_LOW_SPEED.
**
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_InitCan( UINT16 board_hdl
                              , UINT8  can_num
                              , UINT8  bt0
                              , UINT8  bt1
                              , UINT8  mode);


/*************************************************************************
**
**    Function      : VCI_SetAccMask
**
**    Description   : The acceptance mask registers of the CAN controllers
**                    will be set. This is for a global filtering in 11bit
**                    or 29bit mode (if necessary, this function will be
**                    implemented by software).
**                    The CAN controller will be reset and remains in
**                    init mode. In order to start the controller
**                    VCI_StartCan must be called.
**                    The filter is comletely open (0x0UL, 0x0UL) after
**                    initialization of the software.
**
**    Parameters    : board_hdl (in) - board identifcation handle
**                    can_num (in)   - number of the controller (0..n).
**                    acc_code (in)  - Value of the acceptance code reg.
**                                     (gives the id bits to be received)
**                    acc_mask (in)  - Value of the acceptance mask reg.
**                                     (0 - don't care;  1 - relevant)
**                                     Tells the controller which acc_code
**                                     bits are relevant.
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_SetAccMask( UINT16 board_hdl
                                 , UINT8  can_num
                                 , UINT32 acc_code
                                 , UINT32 acc_mask );


/*************************************************************************
**
**    Function      : VCI_ResetCan
**
**    Description   : Sets the CAN controller into init mode and
**                    stops the communication handled by this controller.
**                    The CAN controller doesn't looses its configuration
**                    and can be started again with VCI_StartCan.
**
**    Parameters    : board_hdl (in) - board identifcation handle
**                    can_num (in)   - number of the controller (0..n).
**
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_ResetCan( UINT16 board_hdl, UINT8 can_num );


/*************************************************************************
**
**    Function      : VCI_StartCan
**
**    Description   : Starts the communication over the given
**                    CAN controller.
**
**    Parameters    : board_hdl (in) - board identifcation handle
**                    can_num (in)   - number of the controller (0..n).
**
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_StartCan( UINT16 board_hdl, UINT8 can_num );


/*************************************************************************
**
**    Function      : VCI_ResetTimeStamp
**
**    Description   : Sets the value of the timestamp counter for receive
**                    objects to 0'.
**
**    Parameters    : board_hdl (in) - board identifcation handle
**
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_ResetTimeStamp( UINT16 board_hdl );


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                         Configuration functions
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*************************************************************************
**
**    Function      : VCI_ConfigQueue
**
**    Description   : This function is used for creating a transmit or
**                    a receive queue (FIFO).
**                    A queue handle will be returned in 'p_que_hdl' as a
**                    result of this function. This handle is used for
**                    referencing the queue when calling queue management
**                    functions.
**                    The CAN controller the new queue is connected to
**                    must be in init mode (VCI_ResetCan may be called before).
**
**                    Receive queues can be handled in 3 different ways:
**                    1) Creation of a queue for polling with VCI_ReadQueObj.
**                       The parameters int_limit and int_time will be set
**                       to zero for this mode.
**                    2) Creation of a queue for interrupt processing of
**                       alert objects. Therefore the int_limit has to be
**                       set to a value between 1 and 13 and int_time to zero.
**                       The alert object(s) will be given to the interrupt
**                       callback handler by a pointer. The queue must not 
**                       be read by VCI_ReadQueObj in this mode.
**                    3) Creation of a queue for event driven handling.
**                       For this mode the interrupt only signals received
**                       data. A task in a multitasking environment can
**                       use this event for processing the CAN objects.
**                       Therefore int_limit has to be bigger than 13.
**                       The queue has to be read using VCI_ReadQueObj.
**
**    Parameters    : board_hdl (in)  - board identifcation handle
**                    can_num (in)    - number of the controller (0..n).
**                    que_type (in)   - Queue type (VCI_TX_QUE, VCI_RX_QUE).
**                    que_size (in)   - Size of the queue in CAN objects.
**                                      (must be higher than 20).
**                    int_limit (in)  - Number of objects to be received, to
**                                      initiate an interrupt (must be smaller
**                                      than que_size). Parameter is only valid 
**                                      for receive queues.
**                                      <= 13 alert mode
**                                      >  13 event mode
**                                      0 = no interrupt shall be signaled
**                    int_time (in)   - Time in ms, after which an interrupt
**                                      shall be initiated, if 'int_limit'
**                                      isn't reached. Parameter is only valid 
**                                      for receive queues in alert or
**                                      event mode. For an 'int_time' of 0 ms
**                                      the 'int_time' is internally set to 
**                                      500ms to avoid 100% CPU load.
**                    ts_res (in)     - required timestamp resolutuion
**                                      in micro seconds
**                                      (only valid for receive queue)
**                    p_que_hdl (out) - identification handle of the queue
**
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_ConfigQueue( UINT16  board_hdl
                                  , UINT8   can_num
                                  , UINT8   que_type
                                  , UINT16  que_size
                                  , UINT16  int_limit
                                  , UINT16  int_time
                                  , UINT16  ts_res
                                  , UINT16 *p_que_hdl);


/*************************************************************************
**
**    Function      : VCI_AssignRxQueObj
**
**    Description   : Assigning or releasing CAN objects to or from receive
**                    queues. Identifier groups can be assigned with one
**                    call of 'VCI_AssignRxQueObj' with the usage of
**                    the 'mask' parameter.
**                    Attention: It's not possibles to assigne all CAN
**                               identifier in 29 bit mode.
**                               The number of extended CAN objects depends
**                               on the type of interface board.
**                               To use this function, the CAN controller
**                               must be set to reset state.
**    Parameters    : board_hdl (in)  - board identifcation handle
**                    que_hdl (in)    - queue identifcation handle
**                    mode (in)       - En/disabling of the object(s)
**                                      (VCI_ACCEPT, VCI_REJECT).
**                    id (in)         - identifier of the object(s).
**                    mask (in)       - mask for dertermination of the
**                                      relevant identifierbits in id.
**                                       (0 - don't care;  1 - relevant)
**
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_AssignRxQueObj( UINT16 board_hdl
                                     , UINT16 que_hdl
                                     , UINT8  mode
                                     , UINT32 id
                                     , UINT32 mask);


/*************************************************************************
**
**    Function      : VCI_ConfigBuffer
**
**    Description   : VCI_ConfigBuffer is used to create a receive or
**                    remote buffer. Data in a buffer will be overwritten
**                    by newer data. The access handle will be returned
**                    after successfull completion. Handles will be
**                    given in ascending order (0,1,2,...n).
**
**    Parameters    : board_hdl (in)   - board identifcation handle
**                    can_num (in)     - number of the controller (0..n).
**                    type (in)        - receive or remote buffer
**                                       (VCI_RX_BUF, VCI_RMT_BUF).
**                    id (in)          - Identifier
**                    p_buf_hdl (out)  - pointer to buffer identifcation
**                                       handle
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_ConfigBuffer( UINT16  board_hdl
                                   , UINT8   can_num
                                   , UINT8   type
                                   , UINT32  id
                                   , UINT16 *p_buf_hdl);


/*************************************************************************
**
**    Function      : VCI_ReConfigBuffer
**
**    Description   : Changes the identifier of a receive or
**                    a remote buffer.
**
**    Parameters    : board_hdl (in)  - board identifcation handle
**                    buf_hdl (in)    - buffer identifcation handle
**                    type (in)       - receive or remote buffer
**                                      (VCI_RX_BUF, VCI_RMT_BUF).
**                    id (in)         - Identifier
**
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_ReConfigBuffer( UINT16 board_hdl
                                     , UINT16 buf_hdl
                                     , UINT8  type
                                     , UINT32 id);


/************************************************************************
**
**    Function      : VCI_ConfigTimer
**
**    Description   : This function is obsolete and should no longer
**                    be used.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_ConfigTimer( UINT16 board_hdl, UINT16 int_time);


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                        Read and receive functions
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*************************************************************************
**
**    Function      : VCI_ReadQueStatus
**
**    Description   : Reads the state of the given queue.
**
**    Parameters    : board_hdl (in)  - board identifcation handle
**                    que_hdl (in)    - queue identifcation handle
**
**    Returnvalue   : >0 - Number of objects in queue
**                     0 - (VCI_QUE_EMPTY) - Queue empty.
**                    <0 - VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_ReadQueStatus( UINT16 board_hdl, UINT16 que_hdl);


/*************************************************************************
**
**    Function      : VCI_ReadQueObj
**
**    Description   : VCI_ReadQueObj is used to read the first CAN objects
**                    of a receive queue. The function reads the given
**                    number of CAN objects or all objects until the
**                    queue is empty or the maximum number of objects
**                    deliverable by the interface at one call (14 objects).
**
**    Parameters    : board_hdl (in)  - board identifcation handle
**                    que_hdl (in)    - queue identifcation handle
**                    count (in)      - max number of objects, to be
**                                      read.
**                    p_obj (out)     - Pointer to the object(s) read.
**
**    Returnvalue   : >0 - count of the object(s) read
**                     0 - (VCI_QUE_EMPTY) - Queue empty.
**                    <0 - VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_ReadQueObj( UINT16       board_hdl
                                 , UINT16       que_hdl
                                 , UINT16       count
                                 , VCI_CAN_OBJ *p_obj);


/*************************************************************************
**
**    Function      : VCI_ReadBufStatus
**
**    Description   : Returns the buffer status but doesn't changes it.
**                    The buffer status delivers the number of received
**                    CAN objects since the last read access.
**
**    Parameters    : board_hdl (in)  - board identifcation handle
**                    buf_hdl (in)    - buffer identifcation handle
**
**    Returnvalue   : 0  - VCI_OLD  - no new data received
**                    >0 - count of received objects, after the last read
**                         access.
**                    <0 - VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_ReadBufStatus( UINT16 board_hdl, UINT16 buf_hdl);


/*************************************************************************
**
**    Function      : VCI_ReadBufData
**
**    Description   : Used to read the buffer data and to reset the receive
**                    counter.
**
**    Parameters    : board_hdl (in)  - board identifcation handle
**                    buf_hdl (in)    - buffer identifcation handle
**                    p_data (out)    - pointer to the last received data
**                    p_len (out)     - pointer to the number of data bytes
**
**    Returnvalue   : 0  - VCI_OLD  - no new data received
**                    >0 - count of received objects, after the last read
**                         access.
**                    <0 - VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_ReadBufData( UINT16  board_hdl
                                  , UINT16  buf_hdl
                                  , UINT8  *p_data
                                  , UINT8  *p_len);


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                       Transmit and write functions
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*************************************************************************
**
**    Function      : VCI_TransmitObj
**
**    Description   : Transmits a data object via a transmit queue.
**                    If the return code is 'VCI_QUE_FULL',
**                    the given transmit queue is full at this moment
**                    and the transmission request must be repeated later.
**                    If VCI_TX_ERR will be returned, no messages could be
**                    transmitted for a longer timer. This indicates
**                    possibly that there are no other CAN nodes, cable
**                    errors or a wrong baudrate.
**
**    Parameters    : board_hdl (in)  - board identifcation handle
**                    que_hdl (in)    - queue identifcation handle
**                    id (in)         - identifier of the transmit object
**                    len (in)        - number of the data bytes to be
**                                      transmitted
**                    p_data (in)     - pointer to the data
**
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_TransmitObj( UINT16  board_hdl
                                  , UINT16  que_hdl
                                  , UINT32  id
                                  , UINT8   len
                                  , UINT8  *p_data);


/*************************************************************************
**
**    Function      : VCI_RequestObj
**
**    Description   : This function is used to request a CAN object from
**                    another node by means of a remote object.
**                    There must be a transmit queue created in order
**                    to send this remote request.
**                    If the return code is 'VCI_QUE_FULL',
**                    the given transmit queue is full at this moment
**                    and the transmission request must be repeated later.
**                    If VCI_TX_ERR will be returned, no message could be
**                    transmitted for a longer time. This indicates
**                    possibly that there are no other CAN nodes, cable
**                    errors or a wrong baudrate.
**
**    Parameters    : board_hdl (in)  - board identifcation handle
**                    que_hdl (in)    - queue identifcation handle
**                    id (in)         - identifier of the requested object
**                    len (in)        - number of the data length code
**                                      of the remote frame
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_RequestObj( UINT16 board_hdl
                                 , UINT16 que_hdl
                                 , UINT32 id
                                 , UINT8  len );


/*************************************************************************
**
**    Function      : VCI_UpdateBufObj
**
**    Description   : Data of a remote buffer can be updated with
**                    VCI_UpdateBufObj.
**
**    Parameters    : board_hdl (in)  - board identifcation handle
**                    buf_hdl (in)    - buffer identifcation handle
**                    len (in)        - number of data bytes
**                    p_data (in)     - pointer to data
**
**    Returnvalue   : VCI return codes.
**
*************************************************************************/
INT32 VCI_CALLATTR VCI_UpdateBufObj( UINT16  board_hdl
                                   , UINT16  buf_hdl
                                   , UINT8   len
                                   , UINT8  *p_data );


/*************************************************************************
**
**    Function      : VCI_SetDownloadState
**
**    Description   : Used for setting a flag which indicates to the VCI
**                    that the following PrepareBoard shall be
**                    performed without a firmware download.
**                    The falg state remains unchanged until the Lib will be
**                    removed from memory or VCI_SetDownloadState
**                    is called again
**    Parameters    : o_dld_on: TRUE -> Firmware download shall be performed
**                                      in VCI_PrepareBoard
**                              FALSE-> No firmware download shall be
**                                      performed in VCI_PrepareBoard
**    Returnvalue   : -
**
*************************************************************************/
void VCI_CALLATTR VCI_SetDownloadState( UINT8 o_dld_on);


#ifdef __cplusplus
}
#endif // __cplusplus

#endif // VCI2_DEF

