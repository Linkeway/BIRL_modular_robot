/*************************************************************************
**    IXXAT Automation GmbH
**************************************************************************
**
**   $Workfile: Xatdynl.h $
**     Summary: Funktions for dynamic loading of XATxxReg.dll 
**               (xx = version number)
**   $Revision: 13 $
**     Version: @(VERSION)
**       $Date: 12.02.03 9:52 $
**    Compiler: MSVC++ 6.0, Borland C++ Builder 4
**      Author: Walter Steinhauser, Klaus Oberhofer
**
**************************************************************************
**    all rights reserved
*************************************************************************/

#ifndef XATDYNL_H
#define XATDYNL_H

/*************************************************************************
**    constants and macros
*************************************************************************/
#define DYNCALL(FUNC) lp##FUNC
#define DECLARE_FUNCTION_POINTER_EXT(FUNC)  extern PF_##FUNC lp##FUNC;  
#define DECLARE_FUNCTION_POINTER(FUNC)      PF_##FUNC lp##FUNC=NULL;  
#define LOAD_FUNCTION_POINTER(DLL,FUNC) lp##FUNC = (PF_##FUNC)GetProcAddress(DLL, #FUNC);\
                                        lProcAddr&= lp##FUNC!=NULL?1:0;

/*************************************************************************
**    data types
*************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif
  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_GetXatRegKey    )    ( HKEY* ph_key);
  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_GetRegKeyFromHwEntry)( HKEY* ph_key, DWORD dw_key); 

  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_CreateNewHwEntry)    ( DWORD *p_dw_key);
  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_DeleteHwEntry   )    ( DWORD  dw_key, BYTE bDelKey);
  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_FindHwEntry     )    ( BYTE   b_typ
                                                                 , DWORD *p_dw_key
                                                                 , DWORD *p_dw_boardtyp
                                                                 , char   ca_entryname[255]
                                                                 , DWORD  dw_arg );

  typedef HRESULT (XATREG_CALLATTR * PF_XAT_SetDefaultHwEntry ) ( DWORD  dw_key );
  typedef HRESULT (XATREG_CALLATTR * PF_XAT_GetDefaultHwEntry ) ( DWORD *p_dw_key );


  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_EnumHwEntry )       ( XATREG_ENUM_CALLBACK  fp_callback
                                                                , void                 *vp_context);

  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_GetHwEntryVar)      ( DWORD  dw_key
                                                                , char  *sz_valuename
                                                                , char  *sz_value
                                                                , DWORD *dw_value);

  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_SetHwEntryVarString)( DWORD  dw_key
                                                                , char  *sz_valuename
                                                                , char  *sz_value);

  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_SetHwEntryVarDWORD) ( DWORD dw_key
                                                                , char* sz_valuename
                                                                , DWORD dw_value);

  typedef int   (XATREG_CALLATTR* PF_XAT_SelectHardware    )    ( HWND          hwndOwner
                                                                , XAT_BoardCFG* pConfig );

  typedef int   (XATREG_CALLATTR* PF_XAT_InstallHardware   )    ( HWND  hwndOwner );

  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_GetConfig)          ( DWORD         dw_key
                                                                , XAT_BoardCFG* pConfig );

  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_CleanPCMCIASettings)( DWORD dw_cardtype );

  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_SetPCMCIASettings)  ( DWORD dw_cardtype
                                                                , DWORD dw_location
                                                                , DWORD dw_attribut_address
                                                                , WORD  w_irq
                                                                , WORD  w_CardEnabler);

  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_Get_LPT_Info)       ( WORD             w_port
                                                                , XAT_LPT_COM_CFG *p_sCfg );

  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_Get_COM_Info )      ( WORD             w_port
                                                                , XAT_LPT_COM_CFG *p_sCfg );

  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_SetISASettings)     ( DWORD  dw_cardtype
                                                                , DWORD  dw_location
                                                                , WORD   w_irq);

  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_DelISASettings)     ( DWORD  dw_cardtype
                                                                , DWORD  dw_location
                                                                , WORD   w_irq);

  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_GetNewCardType)     ( WORD   w_oldcardtype
                                                                , DWORD *p_dw_newcardtype);

  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_GetProductVersion)  ( char * sFileName
                                                                , char * sProductVersion);

  typedef HRESULT  (XATREG_CALLATTR* PF_XAT_CheckProductVersion)( char * sFileName
                                                                , char * sCheckVersion);



#ifdef __cplusplus
}
#endif


BOOL LoadXatLib();
BOOL FreeXatLib();

BOOL UnmapXATRegAccess();
int  MapXATRegAccess();

BOOL UnmapXATDialogs();
int  MapXATDialogs();

int UnmapXAT_CARD_EXPANSION();
int MapXAT_CARD_EXPANSION();


#endif // XATDYNL_H
