/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file
This file holds various handy utility types and functions.
*/

#ifndef _DEF_INC_UTILS
#define _DEF_INC_UTILS

#include <limits.h>
#include "CML_Settings.h"

//CML_NAMESPACE_START() //由于命名空间uchar byte这些造成cml与其它的平台中的uchar等重复定义，所以把这句话移动下面。

/* handy types */
typedef unsigned char uchar;       ///< unsigned character
typedef unsigned char byte;        ///< unsigned character
typedef unsigned int  uint;        ///< unsigned integer type
typedef unsigned long ulong;       ///< unsigned long type
typedef signed char int8;          ///< 8-bit integer type
typedef unsigned char uint8;       ///< 8-bit unsigned integer type
CML_NAMESPACE_START()
/** \typedef int16
Signed 16-bit integer type.                                       
Note that the actual definition of this type will depend   
on the compiler being used.  The standard C language header
file limits.h will be used to determine how to create      
the type definition.                                       
*/
#if SHRT_MAX == 32767
typedef short int16;
#elif INT_MAX == 32767
typedef int int16;
#else
#  error "Unable to find a suitable 16-bit int type!"
#endif

/** \typedef uint16
Unsigned 16-bit integer type.                                       
Note that the actual definition of this type will depend   
on the compiler being used.  The standard C language header
file limits.h will be used to determine how to create      
the type definition.                                       
*/
#if USHRT_MAX == 65535
typedef unsigned short uint16;     ///< 16-bit unsigned integer type
#elif UINT_MAX == 65535
typedef unsigned int uint16;       ///< 16-bit unsigned integer type
#else
#  error "Unable to find a suitable 16-bit unsigned int type!"
#endif

/** \typedef int32
Signed 32-bit integer type.                                       
Note that the actual definition of this type will depend   
on the compiler being used.  The standard C language header
file limits.h will be used to determine how to create      
the type definition.                                       
*/
#if INT_MAX == 2147483647
typedef int int32;
#elif LONG_MAX == 2147483647L
typedef long int32;
#else
#  error "Unable to find a suitable 32-bit int type!"
#endif

/** \typedef uint32
Unsigned 32-bit integer type.                                       
Note that the actual definition of this type will depend   
on the compiler being used.  The standard C language header
file limits.h will be used to determine how to create      
the type definition.                                       
*/
#if UINT_MAX == 4294967295U
typedef unsigned int uint32;
#elif ULONG_MAX == 4294967295UL
typedef unsigned long uint32;
#else
#  error "Unable to find a suitable 32-bit unsigned int type!"
#endif

/***************************************************************************/
/** \typedef uunit
User programmable unit.  If user units are enabled in the CML_Settings.h file,
then this type will resolve to double precision floating point.  If not, then
this type will resolve to a 32-bit integer.

User units are used for all position, velocity, acceleration, and jerk values
passed to/from an Amp object.
*/
/***************************************************************************/
#ifdef CML_ENABLE_USER_UNITS
typedef double uunit;
#else
typedef int32 uunit;
#endif

/***************************************************************************/
/** \def ByteCast
The ByteCast() macro is used to cast a value to a byte type (unsigned char).
The reason that a macro is used rather then a simple cast is that some 
processors have characters that are more then 8 bits long.  This is particularly
common with 16-bit or 32-bit microcontrollers and DSPs.  On such systems the
ByteCast macro will strip off any upper bits and then cast the result to a byte.
On systems with 8-bit bytes the ByteCast macro simply casts the passed value to
a byte.
 */
/***************************************************************************/
#if CHAR_BIT > 8
#define ByteCast(x)    ((byte)((x) & 0x00FF))
#else
#define ByteCast(x)    ((byte)(x))
#endif

uint32 bytes_to_uint32( byte *b );
uint16 bytes_to_uint16( byte *b );
int32 bytes_to_int32( byte *b );
int16 bytes_to_int16( byte *b );
void int16_to_bytes( int16 i, byte *b );
void uint16_to_bytes( uint16 i, byte *b );
void int32_to_bytes( int32 i, byte *b );
void uint32_to_bytes( uint32 i, byte *b );

CML_NAMESPACE_END()

#endif

