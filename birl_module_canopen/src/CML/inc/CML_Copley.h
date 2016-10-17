/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

This header file defines a generic Copley node type.  This 
is the base class of all CANopen devices produced by 
Copley Controls Corp.

*/

#ifndef _DEF_INC_COPLEY
#define _DEF_INC_COPLEY

#include "CML_Settings.h"
#include "CML_Firmware.h"
#include "CML_Node.h"

CML_NAMESPACE_START()

/**
This class represents errors that can be returned by the CopleyNode class.
There is one static member for each defined error.
*/
class CopleyNodeError: public NodeError
{
public:
   /// Too much data passed for a serial port command
   static const CopleyNodeError SerialMsgLen;

   /// The device return a serial port error code
   static const CopleyNodeError SerialError;

protected:
   /// Standard protected constructor
   CopleyNodeError( uint16 id, const char *desc ): NodeError( id, desc ){}
};

/***************************************************************************/
/**
Copley CANopen Node class.  
Objects of this class represent Copley products attached to the CANopen bus.
*/
/***************************************************************************/
class CopleyNode: public Node
{
   /// Private copy constructor (not supported)
   CopleyNode( const CopleyNode& );

   /// Private assignment operator (not supported)
   CopleyNode& operator=( const CopleyNode& );

   uint8 lastSerialError;
public:
   CopleyNode(){}
   CopleyNode( CanOpen &co, int16 nodeID ): Node(co,nodeID){}
   virtual ~CopleyNode(){};

   const Error *FirmwareUpdate( Firmware &fw );
   const Error *SerialCmd( uint8 opcode, uint8 &ct, uint8 max=0, uint16 *data=0 );
   uint8 GetLastSerialError( void ){ return lastSerialError; }
};

CML_NAMESPACE_END()

#endif

