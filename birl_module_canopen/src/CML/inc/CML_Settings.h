/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file																				 `
This file provides some configuration options used to customize 
the Copley Motion Libraries.
*/

#ifndef _DEF_INC_CML_SETTINGS
#define _DEF_INC_CML_SETTINGS

/// Library namespace.  This gives the name of the C++ namespace 
/// which will be used to contain the library.  If no namespace
/// is desired, just comment out the #define
#define CML_NAMESPACE     CML

/// Size of the hash table used to associate CAN messages with
/// their receivers.  Larger tables give faster access, but
/// use more memory.
///
/// The following values have been selected as good options
/// for a typical CANopen system: 2053, 1483, 1097, 683, 409.
#define CML_HASH_SIZE    1483

/// Enable file access.  The libraries have some features
/// which require the standard C library functions to open,
/// read, and write files.
/// Some embedded systems do not support a file system, so
/// these features may be disabled by commenting out the
/// #define.  For systems which do support the standard C
/// file access functions, this should be enabled.
#define CML_FILE_ACCESS_OK

/// Allow use of floating point math.  If this is defined, 
/// double precision floating point math will be used in 
/// some areas of the libraries.  These areas include 
/// trajectory generation and unit conversions.  If not
/// defined, then no floating point math will be used, but
/// some features will be disabled.
#define CML_ALLOW_FLOATING_POINT

/// Enable user units.  If this is defined, then all
/// position, velocity, acceleration & jerk values will
/// be specified in double precision floating point, and
/// the units used for these values will be programmable.
/// If not defined, then these parameters will all be
/// specified as 32-bit integers using internal amplifier
/// parameters.  This is less convenient, but can be much
/// faster for systems without a floating point processor.
#define CML_ENABLE_USER_UNITS

/// Enable debug assertions.  If this is defined, then some
/// debug code will be added to the library which will use
/// the standard C assert() function to test for some 
/// programming errors.  Commenting this out will remove the
/// checks.
/// The standard C header file assert.h must be available 
/// if this feature is used.
#define CML_DEBUG_ASSERT

/// This defines the maximum number of amplifiers that may be 
/// controlled by a single linkage object.  The absolute maximum 
/// value that this can accept is 32, however it can be lowered
/// to reduce the memory requirements of the library.
/// This setting also limits the number of independent axes/link.
/// Normally, the number of amps & axes is the same, but the 
/// Linkage object may be extended for conditions where this isn't
/// true.  In any case, there can be no more then this many axes/link.
#define CML_MAX_AMPS_PER_LINK       8

/// This parameter controls the size of the trajectory buffer used by
/// the linkage object.  The linkage object uses this buffer when 
/// streaming multi-axis PVT profiles.  If multi-axis PVT profiles are
/// not required then setting may be commented out.  Doing so will 
/// significantly reduce the size of the linkage object.  If multi-axis
/// PVTs are required, set this to the length of the buffer.  A value of
/// 50 is a reasonable choice.
#define CML_LINKAGE_TRJ_BUFFER_SIZE  50
	
/// Size of the hash table used by the Error::Lookup method.
/// This may be set to zero to disable this feature.  Most systems
/// will not require this feature and can safely set this parameter
/// to zero.
#define CML_ERROR_HASH_SIZE         64

/// The CML::Error object includes a text message for each error
/// type.  If this setting is commented out then those messages 
/// will not be compiled in with the library.  This can be useful
/// for embedded environments where such messages are not used and
/// represent a large amount of wasted memory.
#define CML_ERROR_MESSAGES

/// This setting enables/disables the use of PDO objects within the
/// IOModule class.  The IOModule class is used to access standard
/// CANopen I/O modules on the network.  Normally, these modules may 
/// be accessed using fast PDO transfers, however in very low memory
/// embedded systems the extra RAM required to maintain the PDO objects
/// may not be available.  In such situations this setting may be 
/// commented out to reduce the memory footprint of the IOModule class.
#define CML_ENABLE_IOMODULE_PDOS

/// Size of extra private data for the Thread object.
/// This setting should be left undefined for most systems.
/// It's provided to allow greater flexibility when porting
/// the libraries to another operating system.
//#define CML_EXTRA_THREAD_DATA      0

/// Size of extra private data for the Mutex object.
/// This setting should be left undefined for most systems.
/// It's provided to allow greater flexibility when porting
/// the libraries to another operating system.
//#define CML_EXTRA_MUTEX_DATA       0

/// Size of extra private data for the Semaphore object.
/// This setting should be left undefined for most systems.
/// It's provided to allow greater flexibility when porting
/// the libraries to another operating system.
//#define CML_EXTRA_SEMAPHORE_DATA   0

/**************************************************
* End of user configurable options.
* The rest of this file contains some macro definitions
* used by the library to implement these options.
**************************************************/
#ifdef CML_NAMESPACE

	#define CML_NAMESPACE_START()    namespace CML_NAMESPACE{
	#define CML_NAMESPACE_END()      }
	#define CML_NAMESPACE_USE()      using namespace CML_NAMESPACE

#else

	#define CML_NAMESPACE_START()
	#define CML_NAMESPACE_END()
	#define CML_NAMESPACE_USE()

#endif

/**************************************************
* If debug assertions are enabled, set them up
**************************************************/
#ifdef CML_DEBUG_ASSERT
	#include <assert.h>
	#define CML_ASSERT(x)  assert(x)
#else
	#define CML_ASSERT(x)
#endif

/**************************************************
* Make sure floating point is enabled if user units are
**************************************************/
#ifndef CML_ALLOW_FLOATING_POINT
	#ifdef CML_ENABLE_USER_UNITS
		#error "Floating point must be enabled for user unit support"
	#endif
#endif

#endif

