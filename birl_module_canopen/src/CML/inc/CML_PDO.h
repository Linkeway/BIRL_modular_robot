/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** 
\file

This header file defines the classes used to communicate
to CANopen nodes using Process Data Objects (PDOs).

*/

#ifndef _DEF_INC_CO_PDO
#define _DEF_INC_CO_PDO

#include "CML_Settings.h"
#include "CML_CanOpen.h"
#include "CML_Utils.h"

CML_NAMESPACE_START()

/// Number of variables that may be added to a PDO's map.
#define PDO_MAP_LEN  8

/// Flag used by transmit PDOs to indicate that RTR requests are allowed
#define FLG_RTR_OK    0x01

/**
This class represents error conditions related to PDOs.
*/
class PDO_Error: public CanOpenError
{
public:
   /// The variable map associated with the PDO is already full
   static const PDO_Error MapFull;

   /// Adding the variable to the map would cause the map to
   /// be too long (more then 64 bits).
   static const PDO_Error BitOverflow;

   /// PDO Map variables of the passed bit size are 
   /// not presently supported
   static const PDO_Error BitSizeError;

protected:
   /// Standard protected constructor
   PDO_Error( uint16 id, const char *desc ): CanOpenError( id, desc ){}
};

/***************************************************************************/
/**
This class allows variables to be mapped into a PDO.  This class can be used
directly for transmit PDOs if the received data is not of interest (it will
simply be discarded by the Set() function.  Using this for receive PDOs is
not recommended since the Get() function doesn't add any data to the output
stream and therefore the data transmitted to the node will be undefined.
*/
/***************************************************************************/
class Pmap
{
   /// Private copy constructor (not supported)
   Pmap( const Pmap& );

   /// Private assignment operator (not supported)
   Pmap& operator=( const Pmap& );

public:
   /// Default constructor for a generic PDO mapping variable
   Pmap(){}

   /// Construct a generic PDO mapping variable and initialize
   /// it's size and object ID.
   /// @param index The index of the variable in the object dictionary
   /// @param sub The variable's sub-index in the object dictionary
   /// @param bits The size of the variable in bits
   Pmap( uint16 index, byte sub, byte bits )
   {
      Init( index, sub, bits );
   }

   /// Virtual destructor
   virtual ~Pmap(){}

   /// Initialize a generic PDO mapping variable
   /// @param index The index of the variable in the object dictionary
   /// @param sub The variable's sub-index in the object dictionary
   /// @param bits The size of the variable in bits
   /// @return An error object
   virtual const Error *Init( uint16 index, byte sub, byte bits )
   {
      this->index = index;
      this->sub   = ByteCast(sub);
      this->bits  = ByteCast(bits);
      return 0;
   }

   /// Called when a receive PDO is about to be transmitted.
   /// This virtual function does nothing and therefore objects of this
   /// generic type shouldn't be used when actually transmitting PDOs
   virtual void Get( byte *cptr ){};

   /// Called when a transmit PDO is received.  This virtual function
   /// doesn't do anything and therefore objects of this base class should
   /// only be used for variables that are not of interest and can therefore
   /// be ignored.
   virtual void Set( byte *cptr ){};

   /// Return the 32-bit code used to identify this variable in the
   /// CANopen node's PDO mapping block
   virtual uint32 GetMapCode()
   {
      return (uint32)index<<16 | (uint32)sub<<8 | bits;
   }

   /// Get the object index associated with this variable
   /// @return The 16-bit object index.
   uint16 GetIndex(){
      return index;
   }

   /// Get the object sub-index associated with this variable
   /// @return The 8-bit object sub-index.
   byte GetSub(){
      return sub;
   }

   /// Get the number of bits in this variable
   /// @return The number of bits.
   byte GetBits(){
      return bits;
   }

protected:
   /// The 16-bit index of the object in the object dictionary
   uint16 index;

   /// The 8-bit sub-index of the object in the object dictionary
   byte sub;

   /// The number of bits that this object takes up.
   byte bits;
};

/***************************************************************************/
/**
This is the most generic PDO variable mapping class.  It doesn't do any
special formatting, just holds up to 8 bytes of raw data.
*/
/***************************************************************************/
class PmapRaw: public Pmap
{
public:
   /// Default constructor 
   PmapRaw()
   {
      for( int i=0; i<8; i++ ) data[i] = 0;
   }

   /// Create a new mapping object
   /// @param index Object index associated with this variable
   /// @param sub Object sub-index
   /// @param bits The size of the variable in bits
   PmapRaw( uint16 index, byte sub, byte bits ):Pmap(index, sub, bits)
   {
      for( int i=0; i<8; i++ ) data[i] = 0;
   }

   /// Copy the current value of this variable into the passed
   /// character array.  This function is called when a receive
   /// PDO is about to be transmitted to a node.
   /// @param cptr A character pointer that references a char
   ///        array large enough to hold this objects data.
   virtual void Get( byte *cptr ){
      int bytes = bits>>3;
      m.Lock();
      for( int i=0; i<bytes; i++ ) *cptr++ = ByteCast(data[i]);
      m.Unlock();
   }

   /// Update the value of this variable based on the data passed
   /// in a character array.  This function is called when a transmit
   /// PDO that this variable is mapped to is received.
   /// @param cptr A pointer to the received data.
   virtual void Set( byte *cptr ){
      int bytes = bits>>3;
      m.Lock();
      for( int i=0; i<bytes; i++ ) data[i] = ByteCast(cptr[i]);
      m.Unlock();
   }

private:
   /// A mutex used to protect access to this variable
   Mutex m;

   /// The data associated with this mapping.
   byte data[8];
};

/***************************************************************************/
/**
This is a PDO variable mapping class that extends the virtual Pmap class to
handle 32-bit integers.
*/
/***************************************************************************/
class Pmap32: public Pmap
{
public:
   /// Default constructor for a 32-bit mapping object
   Pmap32(): data(0) {}

   /// Create a new 32-bit mapping object
   /// @param index Object index associated with this variable
   /// @param sub Object sub-index (defaults to 0)
   Pmap32( uint16 index, byte sub=0 ):Pmap(index,sub,32), data(0){}

   /// Initialize a 32-bit mapping object
   /// @param index Object index associated with this variable
   /// @param sub Object sub-index (defaults to 0)
   /// @return An error object
   const Error *Init( uint16 index, byte sub=0 ){
      return Pmap::Init( index, sub, 32 );
   }

   /// Copy the current value of this variable into the passed
   /// character array.  This function is called when a receive
   /// PDO is about to be transmitted to a node.
   /// @param cptr A character pointer that references a char
   ///        array of at least 4 bytes.  The current value
   ///        of this variable will be copied there.
   virtual void Get( byte *cptr ){
      m.Lock();
      int32 v = data;
      m.Unlock();
      cptr[0] = ByteCast(v);
      cptr[1] = ByteCast(v>>8);
      cptr[2] = ByteCast(v>>16);
      cptr[3] = ByteCast(v>>24);
   }

   /// Update the value of this variable based on the data passed
   /// in a character array.  This function is called when a transmit
   /// PDO that this variable is mapped to is received.
   /// @param cptr A character pointer that references a char
   ///        array of at least 4 bytes.  The value of this variable 
   ///        will be updated with the data passed in this array.
   virtual void Set( byte *cptr ){
      int32 v = bytes_to_int32( cptr );
      m.Lock();
      data = v;
      m.Unlock();
   }

   /// Read the current value of this variable
   /// @return The current value of this variable.
   virtual int32 Read( void ){
      m.Lock();
      int32 d = data;
      m.Unlock();
      return d;
   }

   /// Write a new value to this variable.
   /// @param d The new value to write.
   virtual void Write( int32 d ){
      m.Lock();
      data = d;
      m.Unlock();
   }

private:
   /// A mutex used to protect access to this variable
   Mutex m;

   /// The 32-bit integer associated with this mapping.
   int32 data;
};

/***************************************************************************/
/**
This is a PDO variable mapping class that extends the virtual Pmap class to
handle 24-bit integers.
*/
/***************************************************************************/
class Pmap24: public Pmap
{
public:
   /// Default constructor for a 24-bit mapping object
   Pmap24():data(0){}

   /// Create a new 24-bit mapping object
   /// @param index Object index associated with this variable
   /// @param sub Object sub-index (defaults to 0)
   Pmap24( uint16 index, byte sub=0 ):Pmap(index,sub,24), data(0){}

   /// Initialize a 24-bit mapping object
   /// @param index Object index associated with this variable
   /// @param sub Object sub-index (defaults to 0)
   /// @return An error object
   const Error *Init( uint16 index, byte sub=0 ){
      return Pmap::Init( index, sub, 24 );
   }

   /// Copy the current value of this variable into the passed
   /// character array.  This function is called when a receive
   /// PDO is about to be transmitted to a node.
   /// @param cptr A character pointer that references a char
   ///        array of at least 4 bytes.  The current value
   ///        of this variable will be copied there.
   virtual void Get( byte *cptr ){
      m.Lock();
      int32 v = data;
      m.Unlock();
      cptr[0] = ByteCast(v);
      cptr[1] = ByteCast(v>>8);
      cptr[2] = ByteCast(v>>16);
   }

   /// Update the value of this variable based on the data passed
   /// in a character array.  This function is called when a transmit
   /// PDO that this variable is mapped to is received.
   /// @param cptr A character pointer that references a char
   ///        array of at least 4 bytes.  The value of this variable 
   ///        will be updated with the data passed in this array.
   virtual void Set( byte *cptr ){
      int32 v = bytes_to_int32(cptr);
      v = (v<<8) >> 8;
      m.Lock();
      data = v;
      m.Unlock();
   }

   /// Read the current value of this variable
   /// @return The current value of this variable.
   virtual int32 Read( void ){
      m.Lock();
      int32 d = data;
      m.Unlock();
      return d;
   }

   /// Write a new value to this variable.
   /// @param d The new value to write.
   virtual void Write( int32 d ){
      m.Lock();
      data = d;
      m.Unlock();
   }

private:
   /// A mutex used to protect access to this variable
   Mutex m;

   /// The 32-bit integer associated with this mapping.
   int32 data;
};


/***************************************************************************/
/**
This is a PDO variable mapping class that extends the virtual Pmap class to
handle 16-bit integers.
*/
/***************************************************************************/
class Pmap16: public Pmap
{
public:
   /// Default constructor for a 16-bit mapping object
   Pmap16():data(0){}

   /// Create a new 16-bit mapping object
   /// @param index Object index associated with this variable
   /// @param sub Object sub-index (defaults to 0)
   Pmap16( uint16 index, byte sub=0 ):Pmap(index,sub,16),data(0) {}

   /// Initialize a 16-bit mapping object
   /// @param index Object index associated with this variable
   /// @param sub Object sub-index (defaults to 0)
   /// @return An error object
   const Error *Init( uint16 index, byte sub=0 ){
      return Pmap::Init( index, sub, 16 );
   }

   /// Copy the current value of this variable into the passed
   /// character array.  This function is called when a receive
   /// PDO is about to be transmitted to a node.
   /// @param cptr A character pointer that references a char
   ///        array of at least 2 bytes.  The current value
   ///        of this variable will be copied there.
   virtual void Get( byte *cptr ){
      m.Lock();
      int16 v = data;
      m.Unlock();
      cptr[0] = ByteCast(v);
      cptr[1] = ByteCast(v>>8);
   }

   /// Update the value of this variable based on the data passed
   /// in a character array.  This function is called when a transmit
   /// PDO that this variable is mapped to is received.
   /// @param cptr A character pointer that references a char
   ///        array of at least 2 bytes.  The value of this variable 
   ///        will be updated with the data passed in this array.
   virtual void Set( byte *cptr ){
      int16 v = bytes_to_int16(cptr);
      m.Lock();
      data = v;
      m.Unlock();
   }

   /// Read the current value of this variable
   /// @return The current value of this variable.
   virtual int16 Read( void ){
      m.Lock();
      int16 d = data;
      m.Unlock();
      return d;
   }

   /// Write a new value to this variable.
   /// @param d The new value to write.
   virtual void Write( int16 d ){
      m.Lock();
      data = d;
      m.Unlock();
   }

private:
   /// A mutex used to protect access to this variable
   Mutex m;

   /// The 16-bit integer associated with this mapping.
   int16 data;
};

/***************************************************************************/
/**
This is a PDO variable mapping class that extends the virtual Pmap class to
handle 8-bit integers.
*/
/***************************************************************************/
class Pmap8: public Pmap
{
public:
   /// Default constructor for a 8-bit mapping object
   Pmap8():data(0){}

   /// Create a new 8-bit mapping object
   /// @param index Object index associated with this variable
   /// @param sub Object sub-index (defaults to 0)
   Pmap8( uint16 index, byte sub=0 ):Pmap(index,sub,8),data(0){}

   /// Initialize a 8-bit mapping object
   /// @param index Object index associated with this variable
   /// @param sub Object sub-index (defaults to 0)
   /// @return An error object
   const Error *Init( uint16 index, byte sub=0 ){
      return Pmap::Init( index, sub, 8 );
   }

   /// Copy the current value of this variable into the passed
   /// character array.  This function is called when a receive
   /// PDO is about to be transmitted to a node.
   /// @param cptr A character pointer that references a char
   ///        array of at least 1 bytes.  The current value
   ///        of this variable will be copied there.
   virtual void Get( byte *cptr ){
      cptr[0] = ByteCast(data);
   }

   /// Update the value of this variable based on the data passed
   /// in a character array.  This function is called when a transmit
   /// PDO that this variable is mapped to is received.
   /// @param cptr A character pointer that references a char
   ///        array of at least 1 bytes.  The value of this variable 
   ///        will be updated with the data passed in this array.
   virtual void Set( byte *cptr ){
      data = ByteCast(cptr[0]);
   }

   /// Read the current value of this variable
   /// @return The current value of this variable.
   virtual byte Read( void ){
      return ByteCast(data);
   }

   /// Write a new value to this variable.
   /// @param d The new value to write.
   virtual void Write( byte d ){
      data = ByteCast(d);
   }

private:
   /// The 8-bit integer associated with this mapping.
   byte data;
};

/***************************************************************************/
/**
PDO (Process Data Object) base class.  
*/
/***************************************************************************/
class PDO
{
   /// Private copy constructor (not supported)
   PDO( const PDO& );

   /// Private assignment operator (not supported)
   PDO& operator=( const PDO& );

public:
   /// Default constructor.  Simply initializes some variables.
   PDO( void ){
      flags = FLG_RTR_OK;
      mapCt = 0;
      bitCt = 0;
      type = 0;
   }

   /// Virtual destructor
   virtual ~PDO(){}

   /// Initialize the PDO.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Init( void )
   {
      flags = FLG_RTR_OK;
      mapCt = 0;
      bitCt = 0;
      type = 0;
      return 0;
   }

   /// Return the base index in the object dictionary
   /// for this type of PDOs communication parameters
   virtual uint16 BaseIndex( void ) = 0;

   /// Get the COB ID associated with the PDO
   /// @return The COB-ID of the PDO
   virtual uint32 GetID( void ) = 0;

   /// Set the PDO transmission type code.
   /// @param t Transmission type code
   /// @return An error object.
   virtual const Error *SetType( byte t ){
      type= ByteCast(t);
      return 0;
   }

   /// Return the PDO transmission type associated with this PDO.
   /// @return The 8-bit type code
   virtual byte GetType( void ){
      return type;
   }

   /// Clear the variable map associated with this PDO.
   /// @return An error code
   virtual const Error *ClearMap( void ){
      mapCt = 0;
      return 0;
   }

   virtual const Error *AddVar( Pmap &var );

   /// Return non-zero if RTR requests are OK for this PDO.
   /// Note that this only really makes sense for transmit PDOs
   /// @return zero if RTR not allowed, non-zero if allowed.
   virtual int GetRtrOk( void ){
      return flags & FLG_RTR_OK;
   }

   /// Fill the array of 32-bit ints with the PDO mapping codes
   /// used by this PDO.  
   /// @param codes An array of at least PDO_MAP_LEN 32-bit ints.
   ///        The mapping codes will be stored here
   /// @return The number of elements mapped into this PDO.
   virtual int GetMapCodes( uint32 codes[] )
   {
      int i;
      for( i=0; i<mapCt; i++ )
	 codes[i] = map[i]->GetMapCode();
      return i;
   }

protected:
   /// Misc flags associated with the PDO
   byte flags;

   /// Transmission type code
   byte type;

   /// Number of elements in the variable map
   int mapCt;

   /// Number of bits mapped so far
   int bitCt;

   /// Array of pointers to Pmap objects that describe 
   /// the variables transmitted by this PDO.
   Pmap *map[ PDO_MAP_LEN ];
};

/***************************************************************************/
/**
Transmit PDO (transmitted by node, received by this software).
*/
/***************************************************************************/
class TPDO: public Receiver, public PDO
{
public:
   /// Default constructor.
   TPDO(){}

   /// Calls Init() at construction time.
   TPDO( CanOpen &co, uint32 cobID ){
      Init( co, cobID );
   }

   /// Virtual destructor
   virtual ~TPDO(){}

   /// Initialize the PDO
   virtual const Error *Init( CanOpen &co, uint32 cobID ){
      const Error *err = PDO::Init();
      if( !err ) err = Receiver::Init( co, cobID );
      return err;
   }

   /// Return the base index in the object dictionary
   /// for this type of PDOs communication parameters
   virtual uint16 BaseIndex( void ){ return 0x1800; }

   /// Enable or disable RTR requests for this PDO
   /// @param ok zero for no RTR, non-zero for RTR allowed
   virtual void SetRtrOk( int ok ){
      if( ok ) flags |= FLG_RTR_OK;
      else flags &= ~FLG_RTR_OK;
   }

   /// Get the COB ID associated with the PDO
   /// @return The COB-ID of the PDO
   virtual uint32 GetID( void ){
      return getRecvID();
   }

   /// This function is called by the CANopen read thread
   /// when this PDO has been received.
   virtual void Received(){}

   virtual const Error *Request( void );
   virtual int NewFrame( CanFrame &frame );
};

/***************************************************************************/
/**
Receive PDO (received by node, transmitted by this software).
*/
/***************************************************************************/
class RPDO: public PDO
{
public:
   /// Default constructor. 
   RPDO(){ id = 0; }

   /// Construct the PDO object and initialize it.
   /// @param cobID The CAN message ID associated with this PDO
   RPDO( uint32 cobID ){
      Init( cobID );
   }

   /// Virtual destructor
   virtual ~RPDO(){}

   /// Initialize the PDO object.
   /// @param cobID The CAN message ID associated with this PDO
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Init( uint32 cobID ){
      id = cobID;
      return PDO::Init();
   }

   virtual const Error *Transmit( CanOpen &co );

   /// Return the base index in the object dictionary
   /// for this type of PDOs communication parameters
   virtual uint16 BaseIndex( void ){ return 0x1400; }

   /// Get the COB ID associated with the PDO
   /// @return The COB-ID of the PDO
   virtual uint32 GetID( void ){
      return id;
   }

private:
   uint32 id;
};

CML_NAMESPACE_END()

#endif

