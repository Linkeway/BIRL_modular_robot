/*************************************************************************
**    IXXAT Automation GmbH
**************************************************************************
**
**   $Workfile: Xatdynl.cpp $
**     Summary: Funktions for dynamic loading of XATxxReg.dll 
**               (xx = version number)
**   $Revision: 15 $
**     Version: @(VERSION)
**    Compiler: MSVC++ 6.0, Borland C++ Builder 4
**       $Date: 12.02.03 9:52 $
**      Author: Walter Steinhauser, Klaus Oberhofer
**
**************************************************************************
**    all rights reserved
*************************************************************************/

/*************************************************************************
**    compiler directives
*************************************************************************/

/*************************************************************************
**    include files
*************************************************************************/

#include <windows.h>
#include "XATXXReg.h"
#include "Xatdynl.h"

DECLARE_FUNCTION_POINTER (XAT_GetXatRegKey        );
DECLARE_FUNCTION_POINTER (XAT_GetRegKeyFromHwEntry);
DECLARE_FUNCTION_POINTER (XAT_CreateNewHwEntry    );
DECLARE_FUNCTION_POINTER (XAT_DeleteHwEntry       );
DECLARE_FUNCTION_POINTER (XAT_FindHwEntry         );

DECLARE_FUNCTION_POINTER (XAT_SetDefaultHwEntry   );
DECLARE_FUNCTION_POINTER (XAT_GetDefaultHwEntry   );

DECLARE_FUNCTION_POINTER (XAT_EnumHwEntry         );
DECLARE_FUNCTION_POINTER (XAT_GetHwEntryVar       );
DECLARE_FUNCTION_POINTER (XAT_SetHwEntryVarString );
DECLARE_FUNCTION_POINTER (XAT_SetHwEntryVarDWORD  );

DECLARE_FUNCTION_POINTER (XAT_SelectHardware      );
DECLARE_FUNCTION_POINTER (XAT_InstallHardware     );
DECLARE_FUNCTION_POINTER (XAT_GetConfig           );

DECLARE_FUNCTION_POINTER (XAT_SetPCMCIASettings   );
DECLARE_FUNCTION_POINTER (XAT_CleanPCMCIASettings );
DECLARE_FUNCTION_POINTER (XAT_Get_LPT_Info        );
DECLARE_FUNCTION_POINTER (XAT_Get_COM_Info        );

DECLARE_FUNCTION_POINTER (XAT_SetISASettings      );
DECLARE_FUNCTION_POINTER (XAT_DelISASettings      );

DECLARE_FUNCTION_POINTER (XAT_GetNewCardType      );

DECLARE_FUNCTION_POINTER (XAT_GetProductVersion   );

DECLARE_FUNCTION_POINTER (XAT_CheckProductVersion );

static HINSTANCE hInstCanLib;
static DWORD     hInstCounter = 0;

/*************************************************************************
**
**    Function    : LoadXatLib
**                  
**    Description : try to load XAT10Reg.DLL dynamically
**    Parameters  : -
**                  
**    Returnvalues: TRUE : success
**                  FALSE: failure (use GetLastError() for further info)
**                  
*************************************************************************/
BOOL LoadXatLib()
{
  BOOL o_ret = FALSE;
  long lProcAddr = 0x1;

  if ( hInstCounter == 0)
  {
    hInstCanLib = LoadLibrary(XATXXREG_MODULENAME);
    if (hInstCanLib)
    {
      LOAD_FUNCTION_POINTER(hInstCanLib, XAT_GetProductVersion );
      LOAD_FUNCTION_POINTER(hInstCanLib, XAT_CheckProductVersion);

      if(!lProcAddr)  // mindestens eine Funktion wurde nicht gefunden
      {
        FreeLibrary( hInstCanLib );
        return(E_INVALIDARG);
      }

      if ( DYNCALL(XAT_CheckProductVersion) ( XATXXREG_MODULENAME
                                            , XATXXREG_MODULEVERSION) == ERROR_SUCCESS )
      {
        o_ret = TRUE;
        hInstCounter = 1;
      }
      else
      {
        FreeLibrary(hInstCanLib);
      }
    }
  }
  else
    ++hInstCounter;


  return o_ret;
}

/*************************************************************************
**
**    Function    : FreeXatLib
**                  
**    Description : free dynamically loaded XAT10Reg.DLL
**    Parameters  : -
**                  
**    Returnvalues: TRUE : success
**                  FALSE: failure (use GetLastError() for further info)
**                  
*************************************************************************/
BOOL FreeXatLib()
{
  BOOL oRet = FALSE;

  if ( hInstCounter == 0)
  {
    oRet = TRUE;
  }
  else if ( hInstCounter == 1)
  {
    UnmapXATRegAccess();
    UnmapXATDialogs  ();
    UnmapXAT_CARD_EXPANSION ();

    if (hInstCanLib)
    {
      oRet = FreeLibrary(hInstCanLib);
      hInstCounter = 0;
      oRet = TRUE; // only for security -> I have tried to free the library !
    }
  }
  else
    -- hInstCounter;

  return oRet;
}

/*************************************************************************
**
**    Function    : UnmapXATRegAccess
**                  
**    Description : sets function pointers for registry acccess to zero
**    Parameters  : -
**                  
**    Returnvalues: TRUE : success
**                  
*************************************************************************/
BOOL UnmapXATRegAccess()
{
  if ( hInstCounter != 1)
    return FALSE;


  if (hInstCanLib )
  {
    lpXAT_GetXatRegKey          = NULL;
    lpXAT_GetRegKeyFromHwEntry  = NULL; 
    lpXAT_CreateNewHwEntry      = NULL; 
    lpXAT_DeleteHwEntry         = NULL;
    lpXAT_FindHwEntry           = NULL;

    lpXAT_SetDefaultHwEntry     = NULL;;
    lpXAT_GetDefaultHwEntry     = NULL;;

    lpXAT_EnumHwEntry           = NULL;
    lpXAT_GetHwEntryVar         = NULL;

    lpXAT_SetHwEntryVarString   = NULL;
    lpXAT_SetHwEntryVarDWORD    = NULL;

    lpXAT_GetNewCardType        = NULL;
  }
  return TRUE;
}

/*************************************************************************
**
**    Function    : MapXATRegAccess
**                  
**    Description : determine pointers to registry access functions 
**                   within XAT10Reg.DLL
**    Parameters  : -
**                  
**    Returnvalues: 0   -> XAT10Reg.DLL not loaded ( use LoadXATLib() )
**                  1   -> success
**                  <0  -> one of the functions within XAT10Reg.DLL
**                         could not be found
**                  
*************************************************************************/
int MapXATRegAccess()
{ 
  int  i_ret     = 0;
  long lProcAddr = 0x1;

  if (hInstCanLib )
  {     
    i_ret = 1;

    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_GetXatRegKey );
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_GetRegKeyFromHwEntry);
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_CreateNewHwEntry );
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_DeleteHwEntry );
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_FindHwEntry );
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_EnumHwEntry );
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_GetHwEntryVar );
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_SetHwEntryVarString );
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_SetHwEntryVarDWORD );
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_SetDefaultHwEntry );
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_GetDefaultHwEntry );
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_GetNewCardType );

    if(!lProcAddr)  // mindestens eine Funktion wurde nicht gefunden
    { i_ret = -1; }
  }       

  return i_ret;
}


/*************************************************************************
**
**    Function    : UnmapXATDialogs
**                  
**    Description : sets function pointers for dialog functions to zero
**    Parameters  : -
**                  
**    Returnvalues: TRUE: success
**                  
*************************************************************************/
BOOL UnmapXATDialogs()
{
  if ( hInstCounter != 1)
    return FALSE;

  if (hInstCanLib )
  {
    lpXAT_SelectHardware        = NULL;
    lpXAT_InstallHardware       = NULL;
    lpXAT_GetConfig             = NULL;
  }
  return TRUE;
}


/*************************************************************************
**
**    Function    : MapXATDialogs
**                  
**    Description : determine pointers to dialog functions 
**                   within XAT10Reg.DLL
**    Parameters  : -
**                  
**    Returnvalues: 0   -> XAT10Reg.DLL not loaded ( use LoadXATLib() )
**                  1   -> success
**                  <0  -> one of the functions within XAT10Reg.DLL
**                         could not be found
**                  
*************************************************************************/
int MapXATDialogs()
{ 
  int  i_ret    = 0  ;
  long lProcAddr= 0x1;

  if (hInstCanLib)
  {     
    i_ret = 1;

    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_SelectHardware );
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_InstallHardware);
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_GetConfig      );

    if(!lProcAddr)  // mindestens eine Funktion wurde nicht gefunden
    { i_ret = -1; }
  }       
  
  return i_ret;
}

/*************************************************************************
**
**    Function    : UnmapXAT_CARD_EXPANSION
**                  
**    Description : sets function pointers for card settings to zero
**    Parameters  : -
**                  
**    Returnvalues: TRUE: success
**                  
*************************************************************************/
BOOL UnmapXAT_CARD_EXPANSION()
{
  if ( hInstCounter != 1)
    return FALSE;

  if (hInstCanLib )
  {
    lpXAT_SetPCMCIASettings = NULL;
    lpXAT_Get_LPT_Info      = NULL;
    lpXAT_Get_COM_Info      = NULL;
    lpXAT_SetISASettings    = NULL;
    lpXAT_DelISASettings    = NULL;
    lpXAT_CleanPCMCIASettings = NULL;
  }
  return TRUE;
}

/*************************************************************************
**
**    Function    : MapXAT_CARD_EXPANSION
**                  
**    Description : determine pointers to card setting functions 
**                   within XAT10Reg.DLL
**    Parameters  : -
**                  
**    Returnvalues: 0   -> XAT10Reg.DLL not loaded ( use LoadXATLib() )
**                  1   -> success
**                  <0  -> one of the functions within XAT10Reg.DLL
**                         could not be found
**                  
*************************************************************************/
int MapXAT_CARD_EXPANSION()
{ 
  int  i_ret     = 0;
  long lProcAddr = 0x1;

  if (hInstCanLib)
  {     
    i_ret = 1;

    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_SetPCMCIASettings );
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_Get_LPT_Info      );
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_Get_COM_Info      );
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_SetISASettings    );
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_DelISASettings   );
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_CleanPCMCIASettings );
    LOAD_FUNCTION_POINTER(hInstCanLib, XAT_DelISASettings   );

    if(!lProcAddr)  // mindestens eine Funktion wurde nicht gefunden
    { i_ret = -1; }
  }       
  return i_ret;
}
