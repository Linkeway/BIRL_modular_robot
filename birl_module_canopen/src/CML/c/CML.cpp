/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file
CML object definition.  This file contains the code used to 
implement the global CML object.
*/

#include "CML.h"

#ifdef CML_FILE_ACCESS_OK
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#endif

// Global CML object
CML_NAMESPACE_START()
CopleyMotionLibrary cml;
CML_NAMESPACE_END()

CML_NAMESPACE_USE();

/***************************************************************************/
/**
Default constructor for the CopleyMotionLibrary object.
*/
/***************************************************************************/
CopleyMotionLibrary::CopleyMotionLibrary()
{
   debugLevel = LOG_NONE;
   logFileName = 0;
   logFileBackup = 0;
   log = 0;
   logSize = 0;
   flushOutput = false;
   destroyed = false;
   maxLogSize = 1000000;

   SetLogFile( "cml.log" );
}

/***************************************************************************/
/**
Destructor for CopleyMotionLibrary object.  This simply closes the log file
if one is open.
*/
/***************************************************************************/
CopleyMotionLibrary::~CopleyMotionLibrary()
{
#ifdef CML_FILE_ACCESS_OK
   mutex.Lock();
   if( log )
   {
      fclose( (FILE*)log );
      log = NULL;
      destroyed = true;
   }

   if( logFileBackup )
   {
      delete logFileBackup;
      logFileBackup = 0;
   }

   if( logFileName )
   {
      delete logFileName;
      logFileName = 0;
   }

   mutex.Unlock();
#endif
}

/***************************************************************************/
/**
Get the CML library version string.
*/
/***************************************************************************/
const char *CopleyMotionLibrary::GetVersionString( void )
{
   return "1.34.00";
}

/***************************************************************************/
/**
Set the debug message level.  The library code includes some debug messages
that may be enabled by setting the debug level to a value greater then LOG_NONE.

If the level was previously set higher then LOG_NONE, and is then set to LOG_NONE, any
open log file will be closed.

The default log level is LOG_NONE (no messages).

@param level The log level.  
*/
/***************************************************************************/

void CopleyMotionLibrary::SetDebugLevel( CML_LOG_LEVEL level )
{
   debugLevel = level;

#ifdef CML_FILE_ACCESS_OK
   mutex.Lock();
   if( !level && log )
   {
      fclose( (FILE*)log );
      log = NULL;
   }
   mutex.Unlock();
#endif
}

/***************************************************************************/
/**
  Enable/Disable the flushing of the log file after each write.  This can be
  useful if debug messages are being lost due to a crash, however it can 
  increase the time necessary to write to the file.

  The default is false (don't flush).

  @param flush True if file flushing is desired
  */
/***************************************************************************/
void CopleyMotionLibrary::SetFlushLog( bool flush )
{
   flushOutput = flush;
}

/***************************************************************************/
/**
Flush the log file (if one is open).  This forces the log contents to be
written to disk, thus preventing it from being lost if the program exits
without calling the CML object destructor.
*/
/***************************************************************************/
void CopleyMotionLibrary::FlushLog( void )
{
#ifdef CML_FILE_ACCESS_OK
   mutex.Lock();
   if( log )
      fflush( (FILE*)log );
   mutex.Unlock();
#endif
}

/***************************************************************************/
/**
Set the debug message log file name.  This file will be used to log 
debug messages.  The file will be created (or truncated if it alread exists)
when the first message is written to the file.  Note that the debug level 
must be set > LOG_NONE for any messages to be written.  Also note that file access
must be enabled in CML_Settings.h for the log file to be used.

If the log file is already open when this method is called, it will be closed
and a new log file with the specified name will be open on the next write to it.

The default log file name is "cml.log"

@param fname The log file name
*/
/***************************************************************************/
void CopleyMotionLibrary::SetLogFile( const char *fname )
{
#ifdef CML_FILE_ACCESS_OK
   if( destroyed ) return;

   if( logFileName )
   {
      delete logFileName;
      logFileName = 0;
   }

   logFileName = new char[ strlen(fname) + 1 ];
   strcpy( logFileName, fname );

   if( logFileBackup )
   {
      delete logFileBackup;
      logFileBackup = 0;
   }

   logFileBackup = new char[ strlen(logFileName) + 5 ];
   strcpy( logFileBackup, logFileName );
   strcat( logFileBackup, ".bak" );

   // If the log is open, close it so that next time it 
   // is written to a new log with the proper name will
   // be created.
   ResizeLog();
#endif
}

/***************************************************************************/
/**
Set the max CML log file size.  This option can be used to prevent a debug
log file from getting so large it uses all available disk space on long runs.
Once the log file exceeds this size, it will be renamed logfilename.bak (where
logfilename is replaced by the log file name), and a new log file will be 
started.  Any old backup log file will be overwritten.

The default max log size is 1,000,000 bytes.

@param max Maximum log file size in bytes
*/
/***************************************************************************/
void CopleyMotionLibrary::SetMaxLogSize( int32 max )
{
   maxLogSize = max;
}

/***************************************************************************/
/**
Write the CAN frame to the log file.  The log level must be at least LOG_FILT_CAN
for this to be written.

@param recv True if this was a received message, false for transmit messages
@param frame The frame to log
*/
/***************************************************************************/
void CopleyMotionLibrary::LogCAN( bool recv, struct CanFrame &frame )
{
#ifdef CML_FILE_ACCESS_OK
   if( debugLevel < LOG_FILT_CAN ) return;

   // Debug level LOG_FILT_CAN filters out some CAN messages
   if( debugLevel < LOG_CAN )
   {
      if( frame.id == 0x080 ) return;
      if( frame.id == 0x180 ) return;
      if( (frame.id > 0x700) && (frame.id < 0x780) ) return;
   }

   // Return immediately if the log file can't be open
   if( !OpenLogFile() ) return;

   // Log a message describing the CAN frame
   switch( frame.type )
   {
      case CAN_FRAME_DATA:
      {
	 if( recv )
	    logSize += fprintf( (FILE *)log, "CAN.R: 0x%08x - ", frame.id );
	 else
	    logSize += fprintf( (FILE *)log, "CAN.X: 0x%08x - ", frame.id );

	 for( int i=0; i<frame.length; i++ )
	    logSize += fprintf( (FILE *)log, "0x%02x ", frame.data[i] );

	 logSize += fprintf( (FILE *)log, "\n" );
	 break;
      }

      case CAN_FRAME_REMOTE:
	 if( recv )
	    logSize += fprintf( (FILE *)log, "CAN.R: 0x%08x - RMT\n", frame.id );
	 else
	    logSize += fprintf( (FILE *)log, "CAN.X: 0x%08x - RMT\n", frame.id );
	 break;

      case CAN_FRAME_ERROR:
	 logSize += fprintf( (FILE*)log, "CAN Error frame\n" );
	 break;

      default:
	 logSize += fprintf( (FILE*)log, "Unknown CAN frame type!\n" );
	 break;
   }

   mutex.Unlock();

   // Flush the file if requested to do so
   if( flushOutput )
      FlushLog();

   if( logSize > maxLogSize )
      ResizeLog();
#endif
}

/***************************************************************************/
/**
Write a debug message to the log file.  The debug level must be set >= LOG_DEBUG
for this message to be written.

Note that message logging is only available if file support is enabled in 
the CML_Settings.h file.

@param fmt A printf style format string
*/
/***************************************************************************/
void CopleyMotionLibrary::Debug( const char *fmt, ... )
{
#ifdef CML_FILE_ACCESS_OK
   if( debugLevel < LOG_DEBUG ) return;

   // Return immediately if the log file can't be open
   if( !OpenLogFile() ) return;

   // Write to the file.
   va_list ap;
   va_start( ap, fmt );
   logSize += vfprintf( (FILE *)log, fmt, ap );
   mutex.Unlock();
   va_end( ap );

   // Flush the file if requested to do so
   if( flushOutput )
      FlushLog();

   if( logSize > maxLogSize )
      ResizeLog();
#endif
}

/***************************************************************************/
/**
Write a warning message to the log file.  The debug level must be set >= LOG_WARNINGS
for this message to be written.

Note that message logging is only available if file support is enabled in 
the CML_Settings.h file.

@param fmt A printf style format string
*/
/***************************************************************************/
void CopleyMotionLibrary::Warn( const char *fmt, ... )
{
#ifdef CML_FILE_ACCESS_OK
   if( debugLevel < LOG_WARNINGS ) return;

   // Return immediately if the log file can't be open
   if( !OpenLogFile() ) return;

   // Write to the file.
   va_list ap;
   va_start( ap, fmt );
   logSize += vfprintf( (FILE *)log, fmt, ap );
   mutex.Unlock();
   va_end( ap );

   // Flush the file if requested to do so
   if( flushOutput )
      FlushLog();

   if( logSize > maxLogSize )
      ResizeLog();
#endif
}

/***************************************************************************/
/**
Write an error message to the log file.  The debug level must be set >= LOG_ERRORS
for this message to be written.

Note that message logging is only available if file support is enabled in 
the CML_Settings.h file.

@param fmt A printf style format string
*/
/***************************************************************************/
void CopleyMotionLibrary::Error( const char *fmt, ... )
{
#ifdef CML_FILE_ACCESS_OK
   if( debugLevel < LOG_ERRORS ) return;

   // Return immediately if the log file can't be open
   if( !OpenLogFile() ) return;

   // Write to the file.
   va_list ap;
   va_start( ap, fmt );
   logSize += vfprintf( (FILE *)log, fmt, ap );
   mutex.Unlock();
   va_end( ap );

   // Flush the file if requested to do so
   if( flushOutput )
      FlushLog();

   if( logSize > maxLogSize )
      ResizeLog();
#endif
}

/***************************************************************************/
/**
Open the log file.  

This method tries to open the log file.  If successful, or if the file was
already open, it returns true with the mutex locked.

If the file can't be open for any reason, the debug level
will be set to zero to prevent any further attempts to write to it.
In this case the method will return false with the mutex unlocked.

@return true if the file was opened successfully.
*/
/***************************************************************************/
bool CopleyMotionLibrary::OpenLogFile( void )
{
   if( destroyed ) return false;

#ifndef CML_FILE_ACCESS_OK
   return false;
#else
   mutex.Lock();

   if( destroyed )
   {
      mutex.Unlock();
      return false;
   }

   // Return true (with the mutex locked) if the log is already open
   if( log )
      return true;

   // Backup any old log file
   if( logFileBackup )
   {
      remove( logFileBackup );
      rename( logFileName, logFileBackup );
   }

   // Try to open the log file.
   log = (void*)fopen( logFileName, "w+" );
   if( log == NULL )
   {
      debugLevel = LOG_NONE;
      mutex.Unlock();
      return false;
   }
   return true;
#endif
}

/***************************************************************************/
/**
Resize the log file.  This is called when the log file size exceeds the max
value.  The log file will be closed and renamed as a backup (overwriting any
existing backup), and a new file will be created.
*/
/***************************************************************************/
void CopleyMotionLibrary::ResizeLog( void )
{
#ifdef CML_FILE_ACCESS_OK
   mutex.Lock();
   if( log )
   {
      fclose( (FILE*)log );
      log = 0;
      logSize = 0;
   }
   mutex.Unlock();
#endif
}
