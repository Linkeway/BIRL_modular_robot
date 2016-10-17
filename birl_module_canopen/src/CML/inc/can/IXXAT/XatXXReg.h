/*************************************************************************
**    IXXAT Automation GmbH, Leibnizstraﬂe 15, 88250 Weingarten
**************************************************************************
**
**   $Workfile: XatXXReg.h $
**     Summary: Interface declaration of IXXAT hardware registration.
**   $Revision: 17 $
**    Compiler: MSVC 6.0
**       $Date: 9.09.03 8:23 $
**      Author: Walter Steinhauser, Klaus Oberhofer, U. Schmid
**
**************************************************************************
**    all rights reserved
*************************************************************************/

#ifndef XATXXREG_H  
#define XATXXREG_H

/*************************************************************************
**    include files
*************************************************************************/

/*************************************************************************
**    constants and macros
*************************************************************************/

#ifndef _HRESULT_DEFINED
#define _HRESULT_DEFINED
typedef LONG HRESULT;
#endif // !_HRESULT_DEFINED


/*
** Dll-Name
*/
#define XATXXREG_MODULENAME    "Xat11reg.dll"

/*
** Dll-Version
*/
#define XATXXREG_MODULEVERSION  "01,01,0,0"

/*
** Definition of calling attributes (XATREG_CALLATTR)
*/
#define XATREG_CALLATTR            WINAPI

/*
** Definition of call back attributes (XATREG_CALLBACKATTR)
*/
#define XATREG_CALLBACKATTR       __cdecl       // no further attributes WIN32

/*
** constants for b_typ parameter of function XAT_FindHwEntry
*/

// find a board to a given adress
#define XATREG_FIND_ADDRESS                     0       
// find board with a given boardtype and a given relative position
#define XATREG_FIND_BOARD_AT_RELATIVE_POSITION  1       
// find a entry/value combination 
#define XATREG_FIND_ENTRY_WITH_VALUE            2       
// find the position of a specific board relative to all boards with a
//  specific board type.               
#define XATREG_FIND_RELATIVE_BTYPE_POSITION     4    

/*
** This is the maximum possible board number.
** Using board numbers outside 0-XATREG_MAX_BOARD_NUMBER returns an 
** E_INVALIG_ARG error.
*/
#define XATREG_MAX_BOARD_NUMBER                 9999   

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
**  board configuration structure
*/
typedef struct _XAT_BoardCFG
{
  WORD            board_no;
  DWORD           board_type;
  char            sz_brd_name[255];   
  char            sz_manufacturer[50];
  char            sz_brd_info[50];
  char            sz_CardAddString[255];
} XAT_BoardCFG;

/*
**  parallel port configuration structure
*/
typedef struct _XAT_LPT_COM_CFG
{
  int             port_type;
  DWORD           dw_irq;
  DWORD           dw_adr;
  DWORD           dw_dma;
  DWORD           dw_addadr;
} XAT_LPT_COM_CFG;


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
**    callback declarations
*************************************************************************/

typedef short (XATREG_CALLBACKATTR * XATREG_ENUM_CALLBACK)( int   i_index
                                                          , int   hw_key
                                                          , char *name
                                                          , char *value
                                                          , char *valuehex
                                                          , void *vp_context);

/*************************************************************************
**    function prototypes
*************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

/*************************************************************************
**
**    Function    : XAT_GetXatRegKey
**                  
**    Description : opens the Ixxat hardware-key in the Registry
**    Parameters  : ph_key: registry key handle
**                  
**    Returnvalues: ERROR_SUCCESS - if successful
**                  HRESULT error code otherwise
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_GetXatRegKey         ( HKEY* ph_key);

/*************************************************************************
**
**    Function    : XAT_GetRegKeyFromHwEntry
**                  
**    Description : returns an RegKey of the defined Index of the IXXAT HW
**    Parameters  : ph_key: registry key handle
**                  dw_key: index to IXXAT HW (board index)
**                  
**    Returnvalues: ERROR_SUCCESS - if successful
**                  E_INVALIDARG  - invalid parameter (dw_key)
**                  HRESULT error code otherwise
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_GetRegKeyFromHwEntry ( HKEY* ph_key
                                                 , DWORD dw_key); 

/*************************************************************************
**
**    Function    : XAT_SetDefaultHwEntry
**                  
**    Description : Sets a default hw entry
**    Parameters  : dw_key: index to IXXAT HW (board index)
**                  
**    Returnvalues: ERROR_SUCCESS - if successful
**                  E_INVALIDARG  - invalid parameter (dw_key)
**                  HRESULT error code otherwise
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_SetDefaultHwEntry  ( DWORD dw_key );

/*************************************************************************
**
**    Function    : XAT_GetDefaultHwEntry
**                  
**    Description : Get the hw entry that is marked as default
**    Parameters  : p_dw_key: index to IXXAT HW (board index)
**                  
**    Returnvalues: ERROR_SUCCESS - if successful
**                  E_INVALIDARG  - invalid parameter (dw_key)
**                  HRESULT error code otherwise
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_GetDefaultHwEntry  ( DWORD *p_dw_key );

/*************************************************************************
**
**    Function    : XAT_CreateNewHwEntry
**                  
**    Description : creates a key for registering new hardware
**    Parameters  : p_dw_key: index to IXXAT HW (board index)
**                  
**    Returnvalues: ERROR_SUCCESS - if successful
**                  HRESULT error code otherwise
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_CreateNewHwEntry ( DWORD *p_dw_key);

/*************************************************************************
**
**    Function    : XAT_DeleteHwEntry
**                  
**    Description : deletes a specified hw entry (delete all subkeys)
**    Parameters  : dw_key : index to IXXAT HW (board index)
**                  bDelKey: TRUE  -> delete the key
**                           FALSE -> delete only the subkeys
**                  
**    Returnvalues: ERROR_SUCCESS - if successful
**                  E_INVALIDARG  - invalid parameter (dw_key)
**                  HRESULT error code otherwise
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_DeleteHwEntry    ( DWORD dw_key
                                             , BYTE  bDelKey );

/*************************************************************************
**
**    Function    : XAT_FindHwEntry
**                  
**    Description : find a specific hw entry
**    Parameters  : b_typ            : search type (XATREG_FIND_??? constants)
**                  p_dw_key         : index to IXXAT HW (board index) found
**                  p_dw_boardtyp    : (in)  board type to search (XATREG_FIND_RELATIVE_BTYPE_POSITION)
**                                     (out) board type found
**                  ca_entryname[255]: entry string
**                                      (used when b_typ == XATREG_FIND_ENTRY_WITH_VALUE)
**                  dw_arg           : additional argument
**                                      (holds adress            when b_typ == XATREG_FIND_ADDRESS)
**                                      (holds relative position when b_typ == XATREG_FIND_BOARD_AT_RELATIVE_POSITION)
**                                      (holds entry value       when b_typ == XATREG_FIND_ENTRY_WITH_VALUE)
**                  
**    Returnvalues: S_OK            - entry found
**                  ERROR_NOT_FOUND - entry not found
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_FindHwEntry ( BYTE   b_typ
                                        , DWORD *p_dw_key
                                        , DWORD *p_dw_boardtyp
                                        , char   ca_entryname[255]
                                        , DWORD  dw_arg );

/*************************************************************************
**
**    Function    : XAT_EnumHwEntry
**                  
**    Description : runs through all HW entries and call the callback function
**    Parameters  : fp_callback: pointer to callback function
**                  vp_context : additional context pointer
**                  
**    Returnvalues: ERROR_SUCCESS - if successful
**                  HRESULT error code otherwise
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_EnumHwEntry ( XATREG_ENUM_CALLBACK  fp_callback
                                        , void                 *vp_context);

/*************************************************************************
**
**    Function    : XAT_GetHwEntryVar
**                  
**    Description : get a specific value from the board registration
**    Parameters  : dw_key      : index to IXXAT HW (board index)
**                  sz_valuename: registry value
**                  sz_value    : value (string)
**                  dw_value    : value (dword)
**                  
**    Returnvalues: ERROR_SUCCESS - if successful
**                  E_INVALIDARG  - invalid parameter (dw_key)
**                  HRESULT error code otherwise
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_GetHwEntryVar ( DWORD  dw_key
                                          , char  *sz_valuename
                                          , char  *sz_value
                                          , DWORD *dw_value);

/*************************************************************************
**
**    Function    : XAT_SetHwEntryVarString
**                  
**    Description : set a specific string value within the board registration
**    Parameters  : dw_key      : index to IXXAT HW (board index)
**                  sz_valuename: registry value
**                  sz_value    : value
**                  
**    Returnvalues: ERROR_SUCCESS - if successful
**                  E_INVALIDARG  - invalid parameter (dw_key)
**                  HRESULT error code otherwise
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_SetHwEntryVarString ( DWORD  dw_key
                                                , char  *sz_valuename
                                                , char  *sz_value);

/*************************************************************************
**
**    Function    : XAT_SetHwEntryVarDWORD
**                  
**    Description : set a specific dword value within the board registration
**    Parameters  : dw_key      : index to IXXAT HW (board index)
**                  sz_valuename: registry value
**                  dw_value    : value
**                  
**    Returnvalues: ERROR_SUCCESS - if successful
**                  E_INVALIDARG  - invalid parameter (dw_key)
**                  HRESULT error code otherwise
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_SetHwEntryVarDWORD ( DWORD  dw_key
                                               , char  *sz_valuename
                                               , DWORD  dw_value);

/*************************************************************************
**
**    Function    : XAT_SelectHardware
**                  
**    Description : starts the select hw dialog
**    Parameters  : hwndOwner: window handle of owner window
**                  pConfig  : configuration structure
**                  
**    Returnvalues:  1 : user clicked ok
**                   0 : user clicked cancel
**                  -1 : error (use GetLastError to get extendend info)
**                  
*************************************************************************/
int XATREG_CALLATTR XAT_SelectHardware ( HWND          hwndOwner
                                       , XAT_BoardCFG *pConfig );

/*************************************************************************
**
**    Function    : XAT_InstallHardware
**                  
**    Description : starts the hw installation dialog
**    Parameters  : hwndOwner: window handle of owner window
**                  
**    Returnvalues:  1 : user clicked ok
**                   0 : user clicked cancel
**                  -1 : error (use GetLastError to get extendend info)
**                  
*************************************************************************/
int XATREG_CALLATTR XAT_InstallHardware( HWND hwndOwner );

/*************************************************************************
**
**    Function    : XAT_GetConfig
**                  
**    Description : get configuration to a specific board
**    Parameters  : dw_key : index to IXXAT HW (board index)
**                  pConfig: configuration structure
**                  
**    Returnvalues: ERROR_SUCCESS - if successful 
**                  HRESULT error code otherwise
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_GetConfig ( DWORD         dw_key
                                      , XAT_BoardCFG *pConfig );

/*************************************************************************
**
**    Function    : XAT_SetPCMCIASettings
**                  
**    Description : reads the WIN32 registry and returns the pcmcia
**                   memory mapped address and the used IRQ
**    Parameters  : dw_cardtype        : card type
**                  dw_location        : location
**                  dw_attribut_address: attribute memory adress
**                  w_irq              : irq
**                  w_CardEnabler      : card enabler present
**                  
**    Returnvalues: S_OK   -> ok
**                  E_FAIL -> not running under win32
**                  HRESULT error code otherwise
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_SetPCMCIASettings( DWORD dw_cardtype
                                             , DWORD dw_location
                                             , DWORD dw_attribut_address
                                             , WORD  w_irq
                                             , WORD  w_CardEnabler);

/*************************************************************************
**
**    Function    : XAT_Get_LPT_Info
**                  
**    Description : find LPT info for a given port
**    Parameters  : w_port: port number
**                  p_sCfg: port configuration
**                  
**    Returnvalues: S_OK   - success
**                  E_FAIL - COM info not found
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_Get_LPT_Info ( WORD             w_port
                                         , XAT_LPT_COM_CFG *p_sCfg );

/*************************************************************************
**
**    Function    : XAT_Get_COM_Info
**                  
**    Description : find COM info for a given port
**    Parameters  : w_port: port number
**                  p_sCfg: port configuration
**                  
**    Returnvalues: S_OK   - success
**                  E_FAIL - COM info not found
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_Get_COM_Info ( WORD             w_port
                                         , XAT_LPT_COM_CFG *p_sCfg );

/*************************************************************************
**
**    Function    : XAT_SetISASettings
**                  
**    Description : set settings for an ISA card in the driver section
**                   (WinNT only)
**    Parameters  : dw_cardtype: card type
**                  dw_location: base adress
**                  w_irq      : IRQ number
**                  
**    Returnvalues: ERROR_SUCCESS - if successful
**                  E_INVALIDARG  - invalid parameter (dw_location == 0)
**                  E_FAIL        - not under WinNT
**                  HRESULT error code otherwise
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_SetISASettings( DWORD dw_cardtype
                                          , DWORD dw_location
                                          , WORD  w_irq);

/*************************************************************************
**
**    Function    : XAT_DelISASettings
**                  
**    Description : claer settings for an ISA card in the driver section
**                   (WinNT only)
**    Parameters  : dw_cardtype: card type
**                  dw_location: base adress
**                  w_irq      : IRQ number
**                  
**    Returnvalues: ERROR_SUCCESS - if successful
**                  E_INVALIDARG  - invalid parameter (dw_location == 0)
**                  E_FAIL        - not under WinNT
**                  HRESULT error code otherwise
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_DelISASettings( DWORD dw_cardtype
                                          , DWORD dw_location
                                          , WORD  w_irq);

/*************************************************************************
**
**    Function    : XAT_GetNewCardType
**                  
**    Description : determine a new card type to an old one
**    Parameters  : w_oldcardtype   : old card type
**                  p_dw_newcardtype: associated new card type
**                  
**    Returnvalues: ERROR_SUCCESS - if successful
**                  E_FAIL        - could not map old card type to new one
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_GetNewCardType( WORD   w_oldcardtype
                                          , DWORD *p_dw_newcardtype);

/*************************************************************************
**
**    Function    : XAT_CleanPCMCIASettings
**                  
**    Description : clears PCMCIA settings
**    Parameters  : dw_cardtype: for further enhancements (SI-Card)
**                  
**    Returnvalues: S_OK   -> ok
**                  E_FAIL -> not running under win32
**                  HRESULT error code otherwise
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_CleanPCMCIASettings( DWORD dw_cardtype );

/*************************************************************************
**
**    Function    : XAT_GetProductVersion
**                  
**    Description : returns the Productversion of the given FileName
**    Parameters  : sFileName       : FileName of the interesting 
**                                    Productversion
**                  sProductVersion : here the Productversion is inserted
**                                    The strings must be allocated outside
**                                    of this function.
**                  
**    Returnvalues: ERROR_SUCCESS - if successful
**                  E_FAIL        - Error 
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_GetProductVersion ( char * sFileName
                                              , char * sProductVersion);

/*************************************************************************
**
**    Function    : XAT_CheckProductVersion
**                  
**    Description : sFileName       : FileName of the interesting 
**                                    Productversion
**                  sCheckVersion   : here the Version to check is inserted
**                  
**    Returnvalues: ERROR_SUCCESS - if successful
**                  E_FAIL        - Error 
**                  
*************************************************************************/
HRESULT XATREG_CALLATTR XAT_CheckProductVersion( char * sFileName
                                               , char * sCheckVersion);



#ifdef __cplusplus
}
#endif

#endif // XATXXREG_H

