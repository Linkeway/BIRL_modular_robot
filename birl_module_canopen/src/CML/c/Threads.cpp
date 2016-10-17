/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file 
This file only contains definitions for the generic thread error objects.
The code used to implement the OS specific thread methods is located in
Operating system specific files such as Threads_posix.cpp and Threads_w32.cpp.
*/

#include "CML_Threads.h"

CML_NAMESPACE_USE();

/* static thread error codes */
CML_NEW_ERROR( ThreadError, Start,     "Error starting the thread" );
CML_NEW_ERROR( ThreadError, Running,   "Thread has already been started" );
CML_NEW_ERROR( ThreadError, Timeout,   "Timeout" );
CML_NEW_ERROR( ThreadError, General,   "General thread error" );
CML_NEW_ERROR( ThreadError, BadParam,  "Illegal parameter value" );
CML_NEW_ERROR( ThreadError, Alloc,     "Memory allocation error" );


