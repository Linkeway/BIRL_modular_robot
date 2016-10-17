/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

This file defines the Filter object.

The Filter object represents a two pole filter structure used
in various locations within the amplifier.

*/

#ifndef _DEF_INC_FILTER
#define _DEF_INC_FILTER

#include "CML_Settings.h"
#include "CML_Utils.h"

CML_NAMESPACE_START()

/***************************************************************************/
/**
Generic filter structure.  This structure holds the coefficients used by the
amplifier in various configurable filters.
*/
/***************************************************************************/
class Filter
{
   public:
      /// These three words hold information about the filter.  They are 
      /// presently reserved for use by the CME program.
      uint16 info[3];

      /// Filter coefficients
      int16 a0,a1,a2,b1,b2;

      /// Scaling factor for coefficients
      int16 k;

   public:
      Filter( void );
      void toBytes( byte data[] );
      void fromBytes( byte data[] );
      void fromWords( int16 data[] );
};

CML_NAMESPACE_END()

#endif

