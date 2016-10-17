/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file 
Top level include file for the CML libraries.

This file serves two purposes; it includes all the other CML header files
and it defines the CML object.  The CML object contains a number of 
utility methods dealing with the library as a whole.  
*/

#ifndef _DEF_INC_CML
#define _DEF_INC_CML

// Include all the other CML headers
#include "CML_Settings.h"
#include "CML_AmpDef.h"
#include "CML_Amp.h"
#include "CML_Can.h"
#include "CML_CanOpen.h"
#include "CML_Copley.h"
#include "CML_Error.h"
#include "CML_EventMap.h"
#include "CML_Filter.h"
#include "CML_Firmware.h"
#include "CML_Geometry.h"
#include "CML_IO.h"
#include "CML_Linkage.h"
#include "CML_Node.h"
#include "CML_Path.h"
#include "CML_PDO.h"
#include "CML_SDO.h"
#include "CML_Threads.h"
#include "CML_Trajectory.h"
#include "CML_TrjTcurve.h"
#include "CML_TrjScurve.h"
#include "CML_Utils.h"

CML_NAMESPACE_START()

/***************************************************************************/
/**
Copley Motion Libraries debug logging level

The CML libraries may be configured to generate a log file for use in debugging
system problems.  This feature is turned off by default, but may be enabled 
by calling the method CopleyMotionLibrary::SetDebugLevel of the global cml
object.

\code
   cml.SetDebugLevel( LOG_EVERYTHING );
\endcode

This enumeration gives the logging levels that may be passed to the SetDebugLevel
function.  Debug logging levels are cumulative, so enabling a high level of logging
will cause all messages that would have been logged at a lower level to be written
to the log as well.  For example, setting the log level to LOG_DEBUG will cause all
debug messages to be written to the log, as well as all warnings and errors.
*/
/***************************************************************************/
enum CML_LOG_LEVEL
{
   LOG_NONE       = 0,   ///< Debug logging is disabled.
   LOG_ERRORS     = 1,   ///< Log serious errors only
   LOG_WARNINGS   = 2,   ///< Log warning messages and errors
   LOG_DEBUG      = 3,   ///< Log some debugging info
   LOG_FILT_CAN   = 5,   ///< Log most CAN messages.  A few common messages are filtered out.
   LOG_CAN        = 6,   ///< Log all CAN messages
   LOG_EVERYTHING = 99   ///< Log everything
};

/***************************************************************************/
/**
Copley Motion Libraries utility object.

This object defines a number of handy methods related to the libraries
as a whole.

A single global CML object is created by the libraries automatically.
*/
/***************************************************************************/
class CopleyMotionLibrary
{
   char *logFileName;
   char *logFileBackup;
   CML_LOG_LEVEL debugLevel;
   bool flushOutput;
   bool destroyed;
   Mutex mutex;
   void *log;
   int32 logSize, maxLogSize;
   bool OpenLogFile( void );
   void ResizeLog( void );
public:
   CopleyMotionLibrary();
   ~CopleyMotionLibrary();
   const char *GetVersionString();
   void SetDebugLevel( CML_LOG_LEVEL level );
   void SetFlushLog( bool flush );
   void SetMaxLogSize( int32 max );
   void FlushLog( void );
   void SetLogFile( const char *fname );
   void Debug( const char *fmt, ... );
   void Warn( const char *fmt, ... );
   void Error( const char *fmt, ... );
   void LogCAN( bool recv, struct CanFrame &frame );

   /// Return the debug level 
   /// @return The debug level presently set
   CML_LOG_LEVEL GetDebugLevel( void ){ return debugLevel; }

   /// Get the state of the log flushing setting.
   /// @return true if enabled.
   bool GetFlushLog( void ){ return flushOutput; }

   /// Return the max log size.
   /// @return The max log size in bytes
   int32 GetMaxLogSize( void ){ return maxLogSize; }

   /// Return the name of the log file
   /// @return The log file name as a zero terminated string
   const char *GetLogFile( void ){ return logFileName; }
};

/// Global CML object
extern CopleyMotionLibrary cml;

CML_NAMESPACE_END()

#endif

