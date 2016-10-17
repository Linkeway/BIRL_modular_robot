/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file
Implementation of the Filter class.
*/

#include "CML.h"

CML_NAMESPACE_USE();

/***************************************************************************/
/**
Default constructor for filter object.  Simply sets all coefficients to zero.
*/
/***************************************************************************/
Filter::Filter( void )
{
   // Setting the filter info field to 0xffff 
   // disables the filter
   for( int i=0; i<3; i++ )
      info[i] = 0xFFFF;

   a0 = a1 = a2 = b1 = b2 = k = 0;
}

/***************************************************************************/
/**
Convert the filter's coefficients to an array of 18 byte values which can
then be downloaded to the amplifier.
@param data An array of 18+ bytes which will be filled with the converted
coefficient values.
*/
/***************************************************************************/
void Filter::toBytes( byte data[] )
{
   for( int i=0; i<3; i++ )
   {
      data[2*i] = ByteCast(info[i]);
      data[2*i+1] = ByteCast(info[i]>>8);
   }

   data[ 6] = ByteCast(a0); data[ 7] = ByteCast(a0>>8);
   data[ 8] = ByteCast(a1); data[ 9] = ByteCast(a1>>8);
   data[10] = ByteCast(a2); data[11] = ByteCast(a2>>8);
   data[12] = ByteCast(b1); data[13] = ByteCast(b1>>8);
   data[14] = ByteCast(b2); data[15] = ByteCast(b2>>8);
   data[16] = ByteCast( k); data[17] = ByteCast( k>>8);
}

/***************************************************************************/
/**
Load the filter coefficient structure given an array of 18 bytes of data.
This function is useful for loading the filter based on data received over
the CANopen interface.
*/
/***************************************************************************/
void Filter::fromBytes( byte data[] )
{
   for( int i=0; i<3; i++ )
      info[i] = bytes_to_uint16( &data[2*i] );

   a0 = bytes_to_uint16( &data[ 6] );
   a1 = bytes_to_uint16( &data[ 8] );
   a2 = bytes_to_uint16( &data[10] );
   b1 = bytes_to_uint16( &data[12] );
   b2 = bytes_to_uint16( &data[14] );
   k  = bytes_to_uint16( &data[16] );
}

/***************************************************************************/
/**
Load the filter coefficient structure given an array of 9 16-bit words of data.
This function is useful for loading the filter based on data read from a 
CME-2 amplifier data file.
*/
/***************************************************************************/
void Filter::fromWords( int16 data[] )
{
   for( int i=0; i<3; i++ )
      info[i] = data[i];

   a0 = data[3];
   a1 = data[4];
   a2 = data[5];
   b1 = data[6];
   b2 = data[7];
   k  = data[8];
}

