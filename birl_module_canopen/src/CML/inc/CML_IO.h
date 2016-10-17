/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file 

  Standard CANopen I/O module support.

*/

#ifndef _DEF_INC_IO
#define _DEF_INC_IO

#include "CML_Settings.h"
#include "CML_Node.h"
#include "CML_PDO.h"

CML_NAMESPACE_START()

/***************************************************************************/
/**
  I/O module errors.  This class is used to represent errors that may be returned
  by a standard I/O module.
  */
/***************************************************************************/
class IOError: public NodeError
{
   public:
      static const IOError BadID;        ///< The passed digital I/O pin ID number is invalid
      static const IOError BadIOCount;   ///< The number of passed I/O ID blocks is invalid

   protected:
      /// Standard protected constructor
      IOError( uint16 id, const char *desc ): NodeError( id, desc ){}
};

/***************************************************************************/
/**
  Object dictionary ID values used on standard I/O modules.
  */
/***************************************************************************/
enum IO_OBJID
{
   IOOBJID_DIN_8_VALUE             = 0x6000,  ///<  8-bit digital input value 
   IOOBJID_DIN_8_POL               = 0x6002,  ///<  8-bit digital input polarity
   IOOBJID_DIN_8_FILT              = 0x6003,  ///<  8-bit digital input filter constant
   IOOBJID_DIN_INTENA              = 0x6005,  ///< Digital input interrupt enable
   IOOBJID_DIN_8_MASK_ANY          = 0x6006,  ///<  8-bit digital input int mask, any change
   IOOBJID_DIN_8_MASK_L2H          = 0x6007,  ///<  8-bit digital input int mask, low to high
   IOOBJID_DIN_8_MASK_H2L          = 0x6008,  ///<  8-bit digital input int mask, high to low
   IOOBJID_DIN_1_VALUE             = 0x6020,  ///<  1-bit digital input value
   IOOBJID_DIN_1_POL               = 0x6030,  ///<  1-bit digital input polarity
   IOOBJID_DIN_1_FILT              = 0x6038,  ///<  1-bit digital input filter constant
   IOOBJID_DIN_1_MASK_ANY          = 0x6050,  ///<  1-bit digital input int mask, any change
   IOOBJID_DIN_1_MASK_L2H          = 0x6060,  ///<  1-bit digital input int mask, low to high
   IOOBJID_DIN_1_MASK_H2L          = 0x6070,  ///<  1-bit digital input int mask, high to low
   IOOBJID_DIN_16_VALUE            = 0x6100,  ///< 16-bit digital input value
   IOOBJID_DIN_16_POL              = 0x6102,  ///< 16-bit digital input polarity
   IOOBJID_DIN_16_FILT             = 0x6103,  ///< 16-bit digital input filter constant
   IOOBJID_DIN_16_MASK_ANY         = 0x6106,  ///< 16-bit digital input int mask, any change
   IOOBJID_DIN_16_MASK_L2H         = 0x6107,  ///< 16-bit digital input int mask, low to high
   IOOBJID_DIN_16_MASK_H2L         = 0x6108,  ///< 16-bit digital input int mask, high to low
   IOOBJID_DIN_32_VALUE            = 0x6120,  ///< 32-bit digital input value
   IOOBJID_DIN_32_POL              = 0x6122,  ///< 32-bit digital input polarity
   IOOBJID_DIN_32_FILT             = 0x6123,  ///< 32-bit digital input filter constant
   IOOBJID_DIN_32_MASK_ANY         = 0x6126,  ///< 32-bit digital input int mask, any change
   IOOBJID_DIN_32_MASK_L2H         = 0x6127,  ///< 32-bit digital input int mask, low to high
   IOOBJID_DIN_32_MASK_H2L         = 0x6128,  ///< 32-bit digital input int mask, high to low

   IOOBJID_DOUT_8_VALUE            = 0x6200,  ///<  8-bit digital output value
   IOOBJID_DOUT_8_POL              = 0x6202,  ///<  8-bit digital output polarity
   IOOBJID_DOUT_8_ERRMODE          = 0x6206,  ///<  8-bit digital output error mode
   IOOBJID_DOUT_8_ERRVAL           = 0x6207,  ///<  8-bit digital output error value
   IOOBJID_DOUT_8_FILT             = 0x6208,  ///<  8-bit digital output filter mask
   IOOBJID_DOUT_1_VALUE            = 0x6220,  ///<  1-bit digital output value
   IOOBJID_DOUT_1_POL              = 0x6240,  ///<  1-bit digital output polarity
   IOOBJID_DOUT_1_ERRMODE          = 0x6250,  ///<  1-bit digital output error mode
   IOOBJID_DOUT_1_ERRVAL           = 0x6260,  ///<  1-bit digital output error value
   IOOBJID_DOUT_1_FILT             = 0x6270,  ///<  1-bit digital output filter mask
   IOOBJID_DOUT_16_VALUE           = 0x6300,  ///< 16-bit digital output value
   IOOBJID_DOUT_16_POL             = 0x6302,  ///< 16-bit digital output polarity
   IOOBJID_DOUT_16_ERRMODE         = 0x6306,  ///< 16-bit digital output error mode
   IOOBJID_DOUT_16_ERRVAL          = 0x6307,  ///< 16-bit digital output error value
   IOOBJID_DOUT_16_FILT            = 0x6308,  ///< 16-bit digital output filter mask
   IOOBJID_DOUT_32_VALUE           = 0x6320,  ///< 32-bit digital output value
   IOOBJID_DOUT_32_POL             = 0x6322,  ///< 32-bit digital output polarity
   IOOBJID_DOUT_32_ERRMODE         = 0x6326,  ///< 32-bit digital output error mode
   IOOBJID_DOUT_32_ERRVAL          = 0x6327,  ///< 32-bit digital output error value
   IOOBJID_DOUT_32_FILT            = 0x6328,  ///< 32-bit digital output filter mask

   IOOBJID_AIN_8_VALUE             = 0x6400,  ///<  8-bit analog input value
   IOOBJID_AIN_16_VALUE            = 0x6401,  ///< 16-bit analog input value
   IOOBJID_AIN_32_VALUE            = 0x6402,  ///< 32-bit analog input value
   IOOBJID_AIN_FLT_VALUE           = 0x6403,  ///< floating point analog input value
   IOOBJID_AIN_MFG_VALUE           = 0x6404,  ///< manufacturer specific analog input value

   IOOBJID_AOUT_8_VALUE            = 0x6410,  ///<  8-bit analog output value
   IOOBJID_AOUT_16_VALUE           = 0x6411,  ///< 16-bit analog output value
   IOOBJID_AOUT_32_VALUE           = 0x6412,  ///< 32-bit analog output value
   IOOBJID_AOUT_FLT_VALUE          = 0x6413,  ///< floating point analog output value
   IOOBJID_AOUT_MFG_VALUE          = 0x6414,  ///< manufacturer specific analog output value

   IOOBJID_AIN_TRIG                = 0x6421,  ///< Analog input trigger selection
   IOOBJID_AIN_INTSRC              = 0x6422,  ///< Analog input interrupt source
   IOOBJID_AIN_INTENA              = 0x6423,  ///< Analog input interrupt enable

   IOOBJID_AIN_32_UPLIM            = 0x6424,  ///< 32-bit analog input upper limit
   IOOBJID_AIN_32_LWLIM            = 0x6425,  ///< 32-bit analog input lower limit
   IOOBJID_AIN_32_UDELTA           = 0x6426,  ///< 32-bit analog input unsigned delta
   IOOBJID_AIN_32_NDELTA           = 0x6427,  ///< 32-bit analog input negative delta
   IOOBJID_AIN_32_PDELTA           = 0x6428,  ///< 32-bit analog input positive delta

   IOOBJID_AIN_FLT_UPLIM           = 0x6429,  ///< floating point analog input upper limit
   IOOBJID_AIN_FLT_LWLIM           = 0x642A,  ///< floating point analog input lower limit
   IOOBJID_AIN_FLT_UDELTA          = 0x642B,  ///< floating point analog input unsigned delta
   IOOBJID_AIN_FLT_NDELTA          = 0x642C,  ///< floating point analog input negative delta
   IOOBJID_AIN_FLT_PDELTA          = 0x642D,  ///< floating point analog input positive delta

   IOOBJID_AIN_FLT_OFFSET          = 0x642E,  ///< floating point analog input offset
   IOOBJID_AIN_FLT_SCALE           = 0x642F,  ///< floating point analog input scaling
   IOOBJID_AIN_UNIT                = 0x6430,  ///< analog input SI Unit
   IOOBJID_AIN_32_OFFSET           = 0x6431,  ///< 32-bit analog input offset
   IOOBJID_AIN_32_SCALE            = 0x6432,  ///< 32-bit analog input scaling

   IOOBJID_AOUT_FLT_OFFSET         = 0x6441,  ///< floating point analog output offset
   IOOBJID_AOUT_FLT_SCALE          = 0x6442,  ///< floating point analog output scaling
   IOOBJID_AOUT_ERRMODE            = 0x6443,  ///< analog output error mode
   IOOBJID_AOUT_32_ERRVAL          = 0x6444,  ///< 32-bit analog output error value
   IOOBJID_AOUT_FLT_ERRVAL         = 0x6445,  ///< floating point analog output error value
   IOOBJID_AOUT_32_OFFSET          = 0x6446,  ///< 32-bit analog output offset
   IOOBJID_AOUT_32_SCALE           = 0x6447,  ///< 32-bit analog output scaling
   IOOBJID_AOUT_UNIT               = 0x6450   ///< analog output SI Unit
};

/***************************************************************************/
/**
  This enumeration is used to define the types of events that may cause an
  analog input to generate an interrupt event.
  */
/***************************************************************************/
enum IO_AIN_TRIG_TYPE
{
   IOAINTRIG_UPPER_LIM             = 0x0001, ///< Input above upper limit 
   IOAINTRIG_LOWER_LIM             = 0x0002, ///< Input below lower limit
   IOAINTRIG_UDELTA                = 0x0004, ///< Input changed by more then the unsigned delta amount
   IOAINTRIG_NDELTA                = 0x0008, ///< Input reduced by more then the negative delta amount
   IOAINTRIG_PDELTA                = 0x0010  ///< Input increased by more then the positive delta
};

/***************************************************************************/
/**
  This enumeration gives the various events that can be waited on.  The default
  events are simply the reception of one of the standard transmit PDO objects.
  */
/***************************************************************************/
enum IOMODULE_EVENTS
{
   /// Digital input PDO 0 was received.  By default, this PDO is
   /// transmitted by the module when any of the first 64 digital
   /// inputs changes state.
   IOEVENT_DIN_PDO0                = 0x00000001,

   /// Analog input PDO 0 was received.  By default, this PDO is
   /// transmitted by the module when any of the first 4 16-bit
   /// analog inputs generates an event.
   ///
   /// There are many different types of events that are programmable
   /// for analog inputs, however not all I/O module manufacturers 
   /// support all (or any) of these events.  The function 
   /// IOmodule::AinSetTrigType can be used to set the type of event
   /// associated with an analog input.
   ///
   /// Consult the documentation provided with the I/O module to determine
   /// what types of analog input events are available for your module.
   IOEVENT_AIN_PDO0                = 0x00010000,

   /// Analog input PDO 1 was received.  This PDO is similar to analog input
   /// PDO 0, however it maps the second group of 4 16-bit analog inputs.
   IOEVENT_AIN_PDO1                = 0x00020000,

   /// Analog input PDO 2 was received.  This PDO is similar to analog input
   /// PDO 0, however it maps the third group of 4 16-bit analog inputs.
   IOEVENT_AIN_PDO2                = 0x00040000
};

/***************************************************************************/
/**
  Standard CANopen I/O module settings.  This structure may be passed to an
  I/O module object during initialization.  It allows custom settings to be
  assigned to the module.
  */
/***************************************************************************/
struct IOModuleSettings
{
   /// The CANopen heartbeat protocol is one of two standard methods used
   /// to constantly watch for network or device problems.  
   /// When the heartbeat protocol is used, each device on the CANopen
   /// network transmits a 'heartbeat' message at a specified interval.
   /// The network master watches for these messages, and is able to 
   /// detect a device error if it's heartbeat message is not received
   /// within the expected time.
   ///
   /// This parameter configures the heartbeat period (milliseconds)
   /// that will be used by this module to transmit it's heartbeat
   /// message.
   ///
   /// If this parameter is set to zero, then the heartbeat protocol
   /// is disabled on this node.
   ///
   /// Default: zero (not used)
   uint16 heartbeatPeriod;

   /// Additional time to wait before generating a heartbeat error (milliseconds)
   /// If the heartbeat protocol is used, then this value, combined with the
   /// heartbeatTime will determine how long the network master waits for the
   /// node's heartbeat message before it generates a heartbeat error.
   ///
   /// Note that setting this to zero does not disable the heartbeat protocol.
   /// set the heartbeatPeriod value to zero to disable heartbeat.
   ///
   /// Default 100 (ms)
   uint16 heartbeatTimeout;

   /// Node guarding guard time (milliseconds)
   ///
   /// The CANopen node guarding protocol is a second method (the first being the
   /// heartbeat protocol) for devices on the network to watch for network problems.
   /// In this protocol, the master controller sends a request message out to the 
   /// slave device at a specified interval.  The slave device responds to this 
   /// request with a message indicating it's state.
   ///
   /// The main difference between this protocol and the heartbeat protocol is that
   /// both the slave node and the master are able to recognize network errors.
   /// With the heartbeat protocol only the network master is able to identify 
   /// network problems.
   ///
   /// Note that only one of these two protocols can be active in a node device at
   /// any time.  If the heartbeat period is non-zero, then the heartbeat protocol
   /// will be used.
   ///
   /// This parameter gives the node guarding period for use with this node.  This
   /// is the period between node guarding request messages sent by the master 
   /// controller.
   ///
   /// Note that both this parameter, and the life time factor must be non-zero for
   /// node guarding to be used.
   ///
   /// Default 0 (ms)
   uint16 guardTime;

   /// Node guarding life time factor
   ///
   /// When the node guarding protocol is used, this parameter is used by the slave
   /// device to determine how long to wait for a node guarding request from the 
   /// master controller before signaling an error condition.
   ///
   /// The life time factor is treated as a multiple of the guard time.  
   ///
   /// If this parameter and the node guard time are both non-zero,
   /// and the heartbeatTime is zero, then node guarding will be setup 
   /// for the amplifier.
   ///
   /// Default 3 (multiples of the guard time)
   uint8 lifeFactor;

   /// Use the standard digital input PDO object.  If true (default) then 
   /// a standard PDO object will be configured to read up to 64 digital
   /// inputs any time one of them changes.  If false, then this PDO 
   /// will not be configured.
   bool useStandardDinPDO;

   /// Use the standard digital output PDO object.  If true (default) then
   /// a standard PDO object will be configured to transmit updated settings
   /// for the first 64 digital output pins when any of them are changed.
   /// If false, no such PDO will be configured.
   bool useStandardDoutPDO;

   /// Use the standard analog input PDO objects.  If true (default) then 
   /// up to three standard PDO objects will be configured to read the 
   /// first 12 16-bit analog input pins when they generate events.
   /// If false, then these PDOs will not be configured.
   bool useStandardAinPDO;

   /// Use the standard analog output PDO objects.  If true (default) then 
   /// up to three standard PDO objects will be configured to transmit the
   /// analog output data for up to 12 16-bit analog outputs.
   /// If false, then these PDOs will not be configured.
   bool useStandardAoutPDO;

   IOModuleSettings( void )
   {
      heartbeatPeriod    = 0;
      heartbeatTimeout   = 100;
      guardTime          = 0;
      lifeFactor         = 3;
      useStandardDinPDO  = true;
      useStandardDoutPDO = true;
      useStandardAinPDO  = true;
      useStandardAoutPDO = true;
   }
};

/***************************************************************************/
/**
  Standard CANopen I/O module.  This class represents and I/O module device
  conforming to the DS401 CANopen specification.  The class may be extended
  to provide additional manufacturer specific features.

  Note that the CANopen standard defines a very large number of parameters that
  may be used with a standard I/O module.  Of these, only a small subset are 
  requred by the spec.  In practice, it seems that most of the major manufacturers
  of CANopen I/O modules only implement the minimum required by the spec.  The 
  result is that many of the optional functions have not been tested with real 
  hardware due to the lack of availability.  Please contact Copley Controls if
  you believe that you have found a problem with any of these functions.

  For the typical I/O module, you can expect the following functionality to be
  supported based on the type of I/O the module supports:

  Digital Inputs: Reading the inputs via PDO or SDO in groups of 8 should be 
  supported.  Other features are optional.

  Digital Outputs: Writing to the outputs via PDO or SDO in groups of 8 should
  be supported.  Other features are optional.

  Analog Inputs: Reading 16-bit analog inputs via PDO or SDO is normally supported.  
  Other input sizes and features are optional.

  Analog Outputs: Writing 16-bit analog outputs via PDO or SDO is normally supported.  
  Other output sizes and features are optional.
  */
/***************************************************************************/
class IOModule: public Node
{
public:
   IOModule( void );
   IOModule( CanOpen &co, int16 nodeID );
   IOModule( CanOpen &co, int16 nodeID, IOModuleSettings &settings );
   virtual ~IOModule();

   virtual const Error *Init( CanOpen &co, int16 nodeID );
   virtual const Error *Init( CanOpen &co, int16 nodeID, IOModuleSettings &settings );

   virtual const Error *WaitIOEvent( IOMODULE_EVENTS event, int32 timeout=-1 );
   virtual const Error *WaitIOEvent( Event &e, int32 timeout, IOMODULE_EVENTS &match );

   /***************************************************************************/
   /** \name Digital input control

     If the module contains digital inputs, these methods may be used to configure
     and read those inputs.  The inputs may be read and controlled individually, 
     or in groups of 8, 16 or 32 inputs.

     All I/O modules should support access to digital inputs in groups of 8.  
     Support for individual access or different groupings is optional under the 
     spec.  If a particular device does not support such groupings, an attempt 
     to use them should return an error code.

     Each input pin or group of pins is assigned an ID number used to access it.
     When single inputs are accessed, these ID numbers range from 0 (the first input)
     to N-1 (the last input), where N is the total number of input pins available
     on the module.

     When groups of inputs are accessed as a unit, the group is assigned a number.
     The first group of inputs will be assigned ID number 0, the second will be ID 1,
     etc.  The number of groups of a particular size will be the total number of inputs
     divided by the group size.

     For example, to access the fifty third input pin individually you would use id
     number 52.  To access it as part of a group of 8 inputs, you would access group
     number 6 (52/8).  Input 52 would be bit 4 (52%8) of that group.
     */
   /***************************************************************************/
   //@{

   /// Get the current setting of the global interrupt enable for digital inputs.
   /// A return value of true indicates that interrupts are enabled, false disabled.
   /// @param value The current interrupt enable setting is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DinGetIntEna( bool &value ){
      uint8 v;
      const Error *err = sdo.Upld8( IOOBJID_DIN_INTENA, 0, v );
      value = (v!=0);
      return err;
   }

   /// Set the current setting of the global interrupt enable for digital inputs.
   /// Setting this parameter to true enables interrupts, false disables.
   /// @param value The interrupt enable setting.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DinSetIntEna( bool value ){
      const Error *err = sdo.Dnld8( IOOBJID_DIN_INTENA, 0, (uint8)value );
      if( !err ) dinIntEna = value;
      return err;
   }

   /// Return the number of individual inputs available on this device.
   /// @param ct The count is returned here.  Zero is returned on error.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DinGetCt( uint16 &ct ){
      ct = 0;
      return BitCount( IOOBJID_DIN_1_VALUE, ct );
   }

   virtual const Error *DinRead( uint16 id, bool &value, bool viaSDO=false );

   /// Get the current polarity settings for a digital input.
   /// Polarity inversion is enabled if true, disabled if false.
   /// @param id Identifies the digital input.
   /// @param value The current polarity setting.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DinGetPol( uint16 id, bool &value ){
      return BitUpld( IOOBJID_DIN_1_POL, id, value );
   }

   /// Set the current polarity setting for a digital input.
   /// Polarity inversion is enabled if true, disabled if false.
   /// @param id Identifies the digital input.
   /// @param value The new polarity setting.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DinSetPol( uint16 id, bool value ){
      return BitDnld( IOOBJID_DIN_1_POL, id, value );
   }

   /// Get the current filter constant setting for a digital input.
   /// The filter constant is enabled if true, disabled if false.
   /// @param id Identifies the digital input.
   /// @param value The current filter setting.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DinGetFilt( uint16 id, bool &value ){
      return BitUpld( IOOBJID_DIN_1_FILT, id, value );
   }

   /// Set the current filter constant setting for a digital input.
   /// The filter constant is enabled if true, disabled if false.
   /// @param id Identifies the digital input.
   /// @param value The new filter setting.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DinSetFilt( uint16 id, bool value ){
      return BitDnld( IOOBJID_DIN_1_FILT, id, value );
   }

   /// Get the 'any transition' interrupt mask settings for a digital input.
   /// If true, any transition on the input will generate an interrupt.
   /// @param id Identifies the digital input.
   /// @param value The current interrupt mask setting
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DinGetMaskAny( uint16 id, bool &value ){
      return BitUpld( IOOBJID_DIN_1_MASK_ANY, id, value );
   }

   /// Set the 'any transition' interrupt mask settings for a digital input.
   /// If true, any transition on the input will generate an interrupt.
   /// @param id Identifies the digital input.
   /// @param value The new interrupt mask setting.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DinSetMaskAny( uint16 id, bool value ){
      return BitDnld( IOOBJID_DIN_1_MASK_ANY, id, value );
   }

   /// Get the 'low to high' interrupt mask settings for a digital input.
   /// If true, a low to high transition on the input will generate an interrupt.
   /// @param id Identifies the digital input.
   /// @param value The current interrupt mask setting
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DinGetMaskLow2High( uint16 id, bool &value ){
      return BitUpld( IOOBJID_DIN_1_MASK_L2H, id, value );
   }

   /// Set the 'low to high' interrupt mask settings for a digital input.
   /// If true, a low to high transition on the input will generate an interrupt.
   /// @param id Identifies the digital input.
   /// @param value The new interrupt mask setting.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DinSetMaskLow2High( uint16 id, bool value ){
      return BitDnld( IOOBJID_DIN_1_MASK_L2H, id, value );
   }

   /// Get the 'high to low' interrupt mask settings for a digital input.
   /// If true, a high to low transition on the input will generate an interrupt.
   /// @param id Identifies the digital input.
   /// @param value The current interrupt mask setting
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DinGetMaskHigh2Low( uint16 id, bool &value ){
      return BitUpld( IOOBJID_DIN_1_MASK_H2L, id, value );
   }

   /// Set the 'high to low' interrupt mask settings for a digital input.
   /// If true, a high to low transition on the input will generate an interrupt.
   /// @param id Identifies the digital input.
   /// @param value The new interrupt mask setting.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DinSetMaskHigh2Low( uint16 id, bool value ){
      return BitDnld( IOOBJID_DIN_1_MASK_H2L, id, value );
   }

   /// Return the number of 8-bit groups of inputs available on this device.
   /// @param ct The count is returned here.  Zero is returned on error.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din8GetCt( uint8 &ct ){
      ct = 0;
      return sdo.Upld8( IOOBJID_DIN_8_VALUE, 0, ct );
   }

   virtual const Error *Din8Read( uint8 id, uint8 &value, bool viaSDO=false );

   /// Get the current polarity settings for a group of 8 digital inputs.
   /// For each input in the group, a value of 1 enables inversion and 0 disables.
   /// @param id Identifies which group of 8 inputs to read.  
   /// @param value The current polarity setting of the 8 input lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din8GetPol( uint8 id, uint8 &value ){
      return sdo.Upld8( IOOBJID_DIN_8_POL, id+1, value );
   }

   /// Set the current polarity setting for a group of 8 digital inputs.
   /// For each input in the group, a value of 1 enables inversion and 0 disables.
   /// @param id Identifies which group of 8 inputs to effect.
   /// @param value The new polarity setting of the 8 input lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din8SetPol( uint8 id, uint8 value ){
      return sdo.Dnld8( IOOBJID_DIN_8_POL, id+1, value );
   }

   /// Get the current filter constant settings for a group of 8 digital inputs.
   /// For each input in the group, a value of 1 enables the filter, 0 disables.
   /// @param id Identifies which group of 8 inputs to read.  
   /// @param value The current filter setting of the 8 input lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din8GetFilt( uint8 id, uint8 &value ){
      return sdo.Upld8( IOOBJID_DIN_8_FILT, id+1, value );
   }

   /// Set the current filter constant setting for a group of 8 digital inputs.
   /// For each input in the group, a value of 1 enables the filter, 0 disables.
   /// @param id Identifies which group of 8 inputs to effect.
   /// @param value The new filter setting of the 8 input lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din8SetFilt( uint8 id, uint8 value ){
      return sdo.Dnld8( IOOBJID_DIN_8_FILT, id+1, value );
   }

   /// Get the 'any transition' interrupt mask settings for a group of 8 digital inputs.
   /// For each input in the group, a value of 1 enables interrupts on any change,
   /// and a value of 0 disables the interrupt.
   /// @param id Identifies which group of 8 inputs to read.  
   /// @param value The current interrupt mask setting of the 8 input lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din8GetMaskAny( uint8 id, uint8 &value ){
      return sdo.Upld8( IOOBJID_DIN_8_MASK_ANY, id+1, value );
   }

   /// Set the 'any transition' interrupt mask settings for a group of 8 digital inputs.
   /// For each input in the group, a value of 1 enables interrupts on any transition,
   /// and a value of 0 disables.
   /// @param id Identifies which group of 8 inputs to effect.
   /// @param value The new interrupt mask value.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din8SetMaskAny( uint8 id, uint8 value ){
      return sdo.Dnld8( IOOBJID_DIN_8_MASK_ANY, id+1, value );
   }

   /// Get the 'low to high' interrupt mask settings for a group of 8 digital inputs.
   /// For each input in the group, a value of 1 enables interrupts on a low to high
   /// transition, and a value of 0 disables the interrupt.
   /// @param id Identifies which group of 8 inputs to read.  
   /// @param value The current interrupt mask setting of the 8 input lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din8GetMaskLow2High( uint8 id, uint8 &value ){
      return sdo.Upld8( IOOBJID_DIN_8_MASK_L2H, id+1, value );
   }

   /// Set the 'low to high' interrupt mask settings for a group of 8 digital inputs.
   /// For each input in the group, a value of 1 enables interrupts on low to high 
   /// transitions, and a value of 0 disables.
   /// @param id Identifies which group of 8 inputs to effect.
   /// @param value The new interrupt mask value.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din8SetMaskLow2High( uint8 id, uint8 value ){
      return sdo.Dnld8( IOOBJID_DIN_8_MASK_L2H, id+1, value );
   }

   /// Get the 'high to low' interrupt mask settings for a group of 8 digital inputs.
   /// For each input in the group, a value of 1 enables interrupts on a high to low
   /// transition, and a value of 0 disables the interrupt.
   /// @param id Identifies which group of 8 inputs to read.  
   /// @param value The current interrupt mask setting of the 8 input lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din8GetMaskHigh2Low( uint8 id, uint8 &value ){
      return sdo.Upld8( IOOBJID_DIN_8_MASK_H2L, id+1, value );
   }

   /// Set the 'high to low' interrupt mask settings for a group of 8 digital inputs.
   /// For each input in the group, a value of 1 enables interrupts on high to low
   /// transitions, and a value of 0 disables.
   /// @param id Identifies which group of 8 inputs to effect.
   /// @param value The new interrupt mask value.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din8SetMaskHigh2Low( uint8 id, uint8 value ){
      return sdo.Dnld8( IOOBJID_DIN_8_MASK_H2L, id+1, value );
   }

   /// Return the number of 16-bit groups of inputs available on this device.
   /// @param ct The count is returned here.  Zero is returned on error.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din16GetCt( uint8 &ct ){
      ct = 0;
      return sdo.Upld8( IOOBJID_DIN_16_VALUE, 0, ct );
   }

   virtual const Error *Din16Read( uint8 id, uint16 &value, bool viaSDO=false );

   /// Get the current polarity settings for a group of 16 digital inputs.
   /// For each input in the group, a value of 1 enables inversion and 0 disables.
   /// @param id Identifies which group of inputs to read.  
   /// @param value The current polarity setting of the input lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din16GetPol( uint8 id, uint16 &value ){
      return sdo.Upld16( IOOBJID_DIN_16_POL, id+1, value );
   }

   /// Set the current polarity setting for a group of 16 digital inputs.
   /// For each input in the group, a value of 1 enables inversion and 0 disables.
   /// @param id Identifies which group of inputs to effect.
   /// @param value The new polarity setting of the input lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din16SetPol( uint8 id, uint16 value ){
      return sdo.Dnld16( IOOBJID_DIN_16_POL, id+1, value );
   }

   /// Get the current filter constant settings for a group of 16 digital inputs.
   /// For each input in the group, a value of 1 enables the filter, 0 disables.
   /// @param id Identifies which group of inputs to read.  
   /// @param value The current filter setting of the input lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din16GetFilt( uint8 id, uint16 &value ){
      return sdo.Upld16( IOOBJID_DIN_16_FILT, id+1, value );
   }

   /// Set the current filter constant setting for a group of 16 digital inputs.
   /// For each input in the group, a value of 1 enables the filter, 0 disables.
   /// @param id Identifies which group of inputs to effect.
   /// @param value The new filter setting of the input lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din16SetFilt( uint8 id, uint16 value ){
      return sdo.Dnld16( IOOBJID_DIN_16_FILT, id+1, value );
   }

   /// Get the 'any transition' interrupt mask settings for a group of 16 digital inputs.
   /// For each input in the group, a value of 1 enables interrupts on any change,
   /// and a value of 0 disables the interrupt.
   /// @param id Identifies which group of inputs to read.  
   /// @param value The current interrupt mask setting of the input lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din16GetMaskAny( uint8 id, uint16 &value ){
      return sdo.Upld16( IOOBJID_DIN_16_MASK_ANY, id+1, value );
   }

   /// Set the 'any transition' interrupt mask settings for a group of 16 digital inputs.
   /// For each input in the group, a value of 1 enables interrupts on any transition,
   /// and a value of 0 disables.
   /// @param id Identifies which group of inputs to effect.
   /// @param value The new interrupt mask value.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din16SetMaskAny( uint8 id, uint16 value ){
      return sdo.Dnld16( IOOBJID_DIN_16_MASK_ANY, id+1, value );
   }

   /// Get the 'low to high' interrupt mask settings for a group of 16 digital inputs.
   /// For each input in the group, a value of 1 enables interrupts on a low to high
   /// transition, and a value of 0 disables the interrupt.
   /// @param id Identifies which group of inputs to read.  
   /// @param value The current interrupt mask setting of the input lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din16GetMaskLow2High( uint8 id, uint16 &value ){
      return sdo.Upld16( IOOBJID_DIN_16_MASK_L2H, id+1, value );
   }

   /// Set the 'low to high' interrupt mask settings for a group of 16 digital inputs.
   /// For each input in the group, a value of 1 enables interrupts on low to high 
   /// transitions, and a value of 0 disables.
   /// @param id Identifies which group of inputs to effect.
   /// @param value The new interrupt mask value.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din16SetMaskLow2High( uint8 id, uint16 value ){
      return sdo.Dnld16( IOOBJID_DIN_16_MASK_L2H, id+1, value );
   }

   /// Get the 'high to low' interrupt mask settings for a group of 16 digital inputs.
   /// For each input in the group, a value of 1 enables interrupts on a high to low
   /// transition, and a value of 0 disables the interrupt.
   /// @param id Identifies which group of inputs to read.  
   /// @param value The current interrupt mask setting of the input lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din16GetMaskHigh2Low( uint8 id, uint16 &value ){
      return sdo.Upld16( IOOBJID_DIN_16_MASK_H2L, id+1, value );
   }

   /// Set the 'high to low' interrupt mask settings for a group of 16 digital inputs.
   /// For each input in the group, a value of 1 enables interrupts on high to low
   /// transitions, and a value of 0 disables.
   /// @param id Identifies which group of inputs to effect.
   /// @param value The new interrupt mask value.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din16SetMaskHigh2Low( uint8 id, uint16 value ){
      return sdo.Dnld16( IOOBJID_DIN_16_MASK_H2L, id+1, value );
   }

   /// Return the number of 32-bit groups of inputs available on this device.
   /// @param ct The count is returned here.  Zero is returned on error.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din32GetCt( uint8 &ct ){
      ct = 0;
      return sdo.Upld8( IOOBJID_DIN_32_VALUE, 0, ct );
   }

   virtual const Error *Din32Read( uint8 id, uint32 &value, bool viaSDO=false );

   /// Get the current polarity settings for a group of 32 digital inputs.
   /// For each input in the group, a value of 1 enables inversion and 0 disables.
   /// @param id Identifies which group of inputs to read.  
   /// @param value The current polarity setting of the input lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din32GetPol( uint8 id, uint32 &value ){
      return sdo.Upld32( IOOBJID_DIN_32_POL, id+1, value );
   }

   /// Set the current polarity setting for a group of 32 digital inputs.
   /// For each input in the group, a value of 1 enables inversion and 0 disables.
   /// @param id Identifies which group of inputs to effect.
   /// @param value The new polarity setting of the input lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din32SetPol( uint8 id, uint32 value ){
      return sdo.Dnld32( IOOBJID_DIN_32_POL, id+1, value );
   }

   /// Get the current filter constant settings for a group of 32 digital inputs.
   /// For each input in the group, a value of 1 enables the filter, 0 disables.
   /// @param id Identifies which group of inputs to read.  
   /// @param value The current filter setting of the input lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din32GetFilt( uint8 id, uint32 &value ){
      return sdo.Upld32( IOOBJID_DIN_32_FILT, id+1, value );
   }

   /// Set the current filter constant setting for a group of 32 digital inputs.
   /// For each input in the group, a value of 1 enables the filter, 0 disables.
   /// @param id Identifies which group of inputs to effect.
   /// @param value The new filter setting of the input lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din32SetFilt( uint8 id, uint32 value ){
      return sdo.Dnld32( IOOBJID_DIN_32_FILT, id+1, value );
   }

   /// Get the 'any transition' interrupt mask settings for a group of 32 digital inputs.
   /// For each input in the group, a value of 1 enables interrupts on any change,
   /// and a value of 0 disables the interrupt.
   /// @param id Identifies which group of inputs to read.  
   /// @param value The current interrupt mask setting of the input lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din32GetMaskAny( uint8 id, uint32 &value ){
      return sdo.Upld32( IOOBJID_DIN_32_MASK_ANY, id+1, value );
   }

   /// Set the 'any transition' interrupt mask settings for a group of 32 digital inputs.
   /// For each input in the group, a value of 1 enables interrupts on any transition,
   /// and a value of 0 disables.
   /// @param id Identifies which group of inputs to effect.
   /// @param value The new interrupt mask value.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din32SetMaskAny( uint8 id, uint32 value ){
      return sdo.Dnld32( IOOBJID_DIN_32_MASK_ANY, id+1, value );
   }

   /// Get the 'low to high' interrupt mask settings for a group of 32 digital inputs.
   /// For each input in the group, a value of 1 enables interrupts on a low to high
   /// transition, and a value of 0 disables the interrupt.
   /// @param id Identifies which group of inputs to read.  
   /// @param value The current interrupt mask setting of the input lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din32GetMaskLow2High( uint8 id, uint32 &value ){
      return sdo.Upld32( IOOBJID_DIN_32_MASK_L2H, id+1, value );
   }

   /// Set the 'low to high' interrupt mask settings for a group of 32 digital inputs.
   /// For each input in the group, a value of 1 enables interrupts on low to high 
   /// transitions, and a value of 0 disables.
   /// @param id Identifies which group of inputs to effect.
   /// @param value The new interrupt mask value.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din32SetMaskLow2High( uint8 id, uint32 value ){
      return sdo.Dnld32( IOOBJID_DIN_32_MASK_L2H, id+1, value );
   }

   /// Get the 'high to low' interrupt mask settings for a group of 32 digital inputs.
   /// For each input in the group, a value of 1 enables interrupts on a high to low
   /// transition, and a value of 0 disables the interrupt.
   /// @param id Identifies which group of inputs to read.  
   /// @param value The current interrupt mask setting of the input lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din32GetMaskHigh2Low( uint8 id, uint32 &value ){
      return sdo.Upld32( IOOBJID_DIN_32_MASK_H2L, id+1, value );
   }

   /// Set the 'high to low' interrupt mask settings for a group of 32 digital inputs.
   /// For each input in the group, a value of 1 enables interrupts on high to low
   /// transitions, and a value of 0 disables.
   /// @param id Identifies which group of inputs to effect.
   /// @param value The new interrupt mask value.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Din32SetMaskHigh2Low( uint8 id, uint32 value ){
      return sdo.Dnld32( IOOBJID_DIN_32_MASK_H2L, id+1, value );
   }

   //@}

   /***************************************************************************/
   /** \name Digital output control

     If the module contains digital outputs, these methods may be used to configure
     and set those outputs.  The outputs may be set and controlled individually, 
     or in groups of 8, 16 or 32 outputs.

     All I/O modules should support access to digital outputs in groups of 8.  
     Support for individual access or different groupings is optional under the 
     spec.  If a particular device does not support such groupings, an attempt 
     to use them should return an error code.

     Each output pin or group of pins is assigned an ID number used to access it.
     When single outputs are accessed, these ID numbers range from 0 (the first output)
     to N-1 (the last output), where N is the total number of output pins available
     on the module.

     When groups of outputs are accessed as a unit, the group is assigned a number.
     The first group of outputs will be assigned ID number 0, the second will be ID 1,
     etc.  The number of groups of a particular size will be the total number of outputs
     divided by the group size.

     For example, to access the twenty seventy output pin individually you would use id
     number 26.  To access it as part of a group of 8 outputs, you would access group
     number 3 (26/8).  Output 26 would be bit 2 (26%8) of that group.

*/
   /***************************************************************************/
   //@{

   /// Return the number of individual outputs available on this device.
   /// @param ct The count is returned here.  Zero is returned on error.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DoutGetCt( uint16 &ct ){
      ct = 0;
      return BitCount( IOOBJID_DOUT_1_VALUE, ct );
   }

   virtual const Error *DoutWrite( uint16 id, bool value, bool viaSDO=false );

   /// Get the current polarity setting for an individual digital output.
   /// A value of true enables inversion and false disables.
   /// @param id Identifies the output to read.
   /// @param value The current polarity setting of the output line is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DoutGetPol( uint16 id, bool &value ){
      return BitUpld( IOOBJID_DOUT_1_POL, id, value );
   }

   /// Set the current polarity setting for an individual digital output.
   /// A value of true enables inversion and false disables.
   /// @param id Identifies which digital output to effect.
   /// @param value The new polarity setting of the output line.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DoutSetPol( uint16 id, bool value ){
      return BitDnld( IOOBJID_DOUT_1_POL, id, value );
   }

   /// Get the current filter constant setting for an individual digital output.
   /// A value of true enables the filter, false disables.
   /// @param id Identifies the output to read.
   /// @param value The current filter setting of the output line is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DoutGetFilt( uint16 id, bool &value ){
      return BitUpld( IOOBJID_DOUT_1_FILT, id, value );
   }

   /// Set the current filter constant setting for an individual digital output.
   /// A value of true enables the filter, false disables.
   /// @param id Identifies which digital output to effect.
   /// @param value The new filter setting of the output line.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DoutSetFilt( uint16 id, bool value ){
      return BitDnld( IOOBJID_DOUT_1_FILT, id, value );
   }

   /// Get the current error mode setting for an individual digital output.
   /// A value of true will cause the output to take it's programmed error value 
   /// on a device failure.  Setting the mode to false will cause the output to 
   /// hold it's programmed value on failure.
   /// @param id Identifies the output to read.
   /// @param value The current error mode setting of the output line is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DoutGetErrMode( uint16 id, bool &value ){
      return BitUpld( IOOBJID_DOUT_1_ERRMODE, id, value );
   }

   /// Set the current error mode setting for an individual digital output.
   /// A value of true will cause the output to take it's programmed error value 
   /// on a device failure.  Setting the mode to false will cause the output to 
   /// hold it's programmed value on failure.
   /// @param id Identifies which digital output to effect.
   /// @param value The new error mode setting of the output line.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DoutSetErrMode( uint16 id, bool value ){
      return BitDnld( IOOBJID_DOUT_1_ERRMODE, id, value );
   }


   /// Get the current error value setting for an individual digital output.
   /// Error values define the state of the output if a device failure occurs.
   /// The error value will only be set for those output pins which have an error
   /// mode set to true.  Those with error mode set to false will not be changed by
   /// a device failure.
   /// @param id Identifies the output to read.
   /// @param value The current error value setting of the output line is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DoutGetErrValue( uint16 id, bool &value ){
      return BitUpld( IOOBJID_DOUT_1_ERRVAL, id, value );
   }

   /// Set the current error value setting for an individual digital output.
   /// Error values define the state of the output if a device failure occurs.
   /// The error value will only be set for those output pins which have an error
   /// mode set to true.  Those with error mode set to false will not be changed by
   /// a device failure.
   /// @param id Identifies which digital output to effect.
   /// @param value The new error value setting of the output line.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *DoutSetErrValue( uint16 id, bool value ){
      return BitDnld( IOOBJID_DOUT_1_ERRVAL, id, value );
   }

   /// Return the number of 8-bit groups of outputs available on this device.
   /// @param ct The count is returned here.  Zero is returned on error.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout8GetCt( uint8 &ct ){
      ct = 0;
      return sdo.Upld8( IOOBJID_DOUT_8_VALUE, 0, ct );
   }

   const Error *Dout8Write( uint8 id, uint8 value, bool viaSDO=false );

   /// Read back the last value written to this bank of 8 digital outputs.
   /// @param id Identifies which group of outputs to read.
   /// @param value The current state of the output lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   const Error *Dout8Read( uint8 id, uint8 &value ){
      return sdo.Upld8( IOOBJID_DOUT_8_VALUE, id+1, value );
   }

   /// Get the current polarity settings for a group of 8 digital outputs.
   /// For each output in the group, a value of 1 enables inversion and 0 disables.
   /// @param id Identifies which group of outputs to read.
   /// @param value The current polarity setting of the output lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout8GetPol( uint8 id, uint8 &value ){
      return sdo.Upld8( IOOBJID_DOUT_8_POL, id+1, value );
   }

   /// Set the current polarity setting for a group of 8 digital outputs.
   /// For each output in the group, a value of 1 enables inversion and 0 disables.
   /// @param id Identifies which group of outputs to effect.
   /// @param value The new polarity setting of the output lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout8SetPol( uint8 id, uint8 value ){
      return sdo.Dnld8( IOOBJID_DOUT_8_POL, id+1, value );
   }

   /// Get the current filter constant settings for a group of 8 digital outputs.
   /// For each output in the group, a value of 1 enables the filter, 0 disables.
   /// @param id Identifies which group of outputs to read.  
   /// @param value The current filter setting of the output lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout8GetFilt( uint8 id, uint8 &value ){
      return sdo.Upld8( IOOBJID_DOUT_8_FILT, id+1, value );
   }

   /// Set the current filter constant setting for a group of 8 digital outputs.
   /// For each output in the group, a value of 1 enables the filter, 0 disables.
   /// @param id Identifies which group of outputs to effect.
   /// @param value The new filter setting of the output lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout8SetFilt( uint8 id, uint8 value ){
      return sdo.Dnld8( IOOBJID_DOUT_8_FILT, id+1, value );
   }

   /// Get the current error mode settings for a group of 8 digital outputs.
   /// For each output in the group, a value of 1 will cause the output to take it's 
   /// programmed error value on a device failure.  Setting the mode to 0 will cause
   /// the output to hold it's programmed value on failure.
   /// @param id Identifies which group of outputs to read.
   /// @param value The current error mode setting of the output lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout8GetErrMode( uint8 id, uint8 &value ){
      return sdo.Upld8( IOOBJID_DOUT_8_ERRMODE, id+1, value );
   }

   /// Set the current error mode settings for a group of 8 digital outputs.
   /// For each output in the group, a value of 1 will cause the output to take it's 
   /// programmed error value on a device failure.  Setting the mode to 0 will cause
   /// the output to hold it's programmed value on failure.
   /// @param id Identifies which group of outputs to effect.
   /// @param value The new error mode setting of the output lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout8SetErrMode( uint8 id, uint8 value ){
      return sdo.Dnld8( IOOBJID_DOUT_8_ERRMODE, id+1, value );
   }


   /// Get the current error value settings for a group of 8 digital outputs.
   /// Error values define the state of the output if a device failure occurs.
   /// The error value will only be set for those output pins which have an error
   /// mode set to 1.  Those with error mode set to zero will not be changed by
   /// a device failure.
   /// @param id Identifies which group of outputs to read.
   /// @param value The current error value setting of the output lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout8GetErrValue( uint8 id, uint8 &value ){
      return sdo.Upld8( IOOBJID_DOUT_8_ERRVAL, id+1, value );
   }

   /// Set the current error value settings for a group of 8 digital outputs.
   /// Error values define the state of the output if a device failure occurs.
   /// The error value will only be set for those output pins which have an error
   /// mode set to 1.  Those with error mode set to zero will not be changed by
   /// a device failure.
   /// @param id Identifies which group of outputs to effect.
   /// @param value The new error value setting of the output lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout8SetErrValue( uint8 id, uint8 value ){
      return sdo.Dnld8( IOOBJID_DOUT_8_ERRVAL, id+1, value );
   }

   /// Return the number of 16-bit groups of outputs available on this device.
   /// @param ct The count is returned here.  Zero is returned on error.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout16GetCt( uint8 &ct ){
      ct = 0;
      return sdo.Upld8( IOOBJID_DOUT_16_VALUE, 0, ct );
   }

   virtual const Error *Dout16Write( uint8 id, uint16 value, bool viaSDO=false );

   /// Read back the last value written to this bank of 16 digital outputs.
   /// @param id Identifies which group of outputs to read.
   /// @param value The current state of the output lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   const Error *Dout16Read( uint8 id, uint16 &value ){
      return sdo.Upld16( IOOBJID_DOUT_16_VALUE, id+1, value );
   }

   /// Get the current polarity settings for a group of 16 digital outputs.
   /// For each output in the group, a value of 1 enables inversion and 0 disables.
   /// @param id Identifies which group of outputs to read.
   /// @param value The current polarity setting of the output lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout16GetPol( uint8 id, uint16 &value ){
      return sdo.Upld16( IOOBJID_DOUT_16_POL, id+1, value );
   }

   /// Set the current polarity setting for a group of 16 digital outputs.
   /// For each output in the group, a value of 1 enables inversion and 0 disables.
   /// @param id Identifies which group of outputs to effect.
   /// @param value The new polarity setting of the output lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout16SetPol( uint8 id, uint16 value ){
      return sdo.Dnld16( IOOBJID_DOUT_16_POL, id+1, value );
   }

   /// Get the current filter constant settings for a group of 16 digital outputs.
   /// For each output in the group, a value of 1 enables the filter, 0 disables.
   /// @param id Identifies which group of outputs to read.  
   /// @param value The current filter setting of the output lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout16GetFilt( uint8 id, uint16 &value ){
      return sdo.Upld16( IOOBJID_DOUT_16_FILT, id+1, value );
   }

   /// Set the current filter constant setting for a group of 16 digital outputs.
   /// For each output in the group, a value of 1 enables the filter, 0 disables.
   /// @param id Identifies which group of outputs to effect.
   /// @param value The new filter setting of the output lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout16SetFilt( uint8 id, uint16 value ){
      return sdo.Dnld16( IOOBJID_DOUT_16_FILT, id+1, value );
   }

   /// Get the current error mode settings for a group of 16 digital outputs.
   /// For each output in the group, a value of 1 will cause the output to take it's 
   /// programmed error value on a device failure.  Setting the mode to 0 will cause
   /// the output to hold it's programmed value on failure.
   /// @param id Identifies which group of outputs to read.
   /// @param value The current error mode setting of the output lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout16GetErrMode( uint8 id, uint16 &value ){
      return sdo.Upld16( IOOBJID_DOUT_16_ERRMODE, id+1, value );
   }

   /// Set the current error mode settings for a group of 16 digital outputs.
   /// For each output in the group, a value of 1 will cause the output to take it's 
   /// programmed error value on a device failure.  Setting the mode to 0 will cause
   /// the output to hold it's programmed value on failure.
   /// @param id Identifies which group of outputs to effect.
   /// @param value The new error mode setting of the output lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout16SetErrMode( uint8 id, uint16 value ){
      return sdo.Dnld16( IOOBJID_DOUT_16_ERRMODE, id+1, value );
   }


   /// Get the current error value settings for a group of 16 digital outputs.
   /// Error values define the state of the output if a device failure occurs.
   /// The error value will only be set for those output pins which have an error
   /// mode set to 1.  Those with error mode set to zero will not be changed by
   /// a device failure.
   /// @param id Identifies which group of outputs to read.
   /// @param value The current error value setting of the output lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout16GetErrValue( uint8 id, uint16 &value ){
      return sdo.Upld16( IOOBJID_DOUT_16_ERRVAL, id+1, value );
   }

   /// Set the current error value settings for a group of 16 digital outputs.
   /// Error values define the state of the output if a device failure occurs.
   /// The error value will only be set for those output pins which have an error
   /// mode set to 1.  Those with error mode set to zero will not be changed by
   /// a device failure.
   /// @param id Identifies which group of outputs to effect.
   /// @param value The new error value setting of the output lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout16SetErrValue( uint8 id, uint16 value ){
      return sdo.Dnld16( IOOBJID_DOUT_16_ERRVAL, id+1, value );
   }

   /// Return the number of 32-bit groups of outputs available on this device.
   /// @param ct The count is returned here.  Zero is returned on error.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout32GetCt( uint8 &ct ){
      ct = 0;
      return sdo.Upld8( IOOBJID_DOUT_32_VALUE, 0, ct );
   }

   virtual const Error *Dout32Write( uint8 id, uint32 value, bool viaSDO=false );

   /// Read back the last value written to this bank of 32 digital outputs.
   /// @param id Identifies which group of outputs to read.
   /// @param value The current state of the output lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   const Error *Dout32Read( uint8 id, uint32 &value ){
      return sdo.Upld32( IOOBJID_DOUT_32_VALUE, id+1, value );
   }

   /// Get the current polarity settings for a group of 32 digital outputs.
   /// For each output in the group, a value of 1 enables inversion and 0 disables.
   /// @param id Identifies which group of outputs to read.
   /// @param value The current polarity setting of the output lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout32GetPol( uint8 id, uint32 &value ){
      return sdo.Upld32( IOOBJID_DOUT_32_POL, id+1, value );
   }

   /// Set the current polarity setting for a group of 32 digital outputs.
   /// For each output in the group, a value of 1 enables inversion and 0 disables.
   /// @param id Identifies which group of outputs to effect.
   /// @param value The new polarity setting of the output lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout32SetPol( uint8 id, uint32 value ){
      return sdo.Dnld32( IOOBJID_DOUT_32_POL, id+1, value );
   }

   /// Get the current filter constant settings for a group of 32 digital outputs.
   /// For each output in the group, a value of 1 enables the filter, 0 disables.
   /// @param id Identifies which group of outputs to read.  
   /// @param value The current filter setting of the output lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout32GetFilt( uint8 id, uint32 &value ){
      return sdo.Upld32( IOOBJID_DOUT_32_FILT, id+1, value );
   }

   /// Set the current filter constant setting for a group of 32 digital outputs.
   /// For each output in the group, a value of 1 enables the filter, 0 disables.
   /// @param id Identifies which group of outputs to effect.
   /// @param value The new filter setting of the output lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout32SetFilt( uint8 id, uint32 value ){
      return sdo.Dnld32( IOOBJID_DOUT_32_FILT, id+1, value );
   }

   /// Get the current error mode settings for a group of 32 digital outputs.
   /// For each output in the group, a value of 1 will cause the output to take it's 
   /// programmed error value on a device failure.  Setting the mode to 0 will cause
   /// the output to hold it's programmed value on failure.
   /// @param id Identifies which group of outputs to read.
   /// @param value The current error mode setting of the output lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout32GetErrMode( uint8 id, uint32 &value ){
      return sdo.Upld32( IOOBJID_DOUT_32_ERRMODE, id+1, value );
   }

   /// Set the current error mode settings for a group of 32 digital outputs.
   /// For each output in the group, a value of 1 will cause the output to take it's 
   /// programmed error value on a device failure.  Setting the mode to 0 will cause
   /// the output to hold it's programmed value on failure.
   /// @param id Identifies which group of outputs to effect.
   /// @param value The new error mode setting of the output lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout32SetErrMode( uint8 id, uint32 value ){
      return sdo.Dnld32( IOOBJID_DOUT_32_ERRMODE, id+1, value );
   }


   /// Get the current error value settings for a group of 32 digital outputs.
   /// Error values define the state of the output if a device failure occurs.
   /// The error value will only be set for those output pins which have an error
   /// mode set to 1.  Those with error mode set to zero will not be changed by
   /// a device failure.
   /// @param id Identifies which group of outputs to read.
   /// @param value The current error value setting of the output lines is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout32GetErrValue( uint8 id, uint32 &value ){
      return sdo.Upld32( IOOBJID_DOUT_32_ERRVAL, id+1, value );
   }

   /// Set the current error value settings for a group of 32 digital outputs.
   /// Error values define the state of the output if a device failure occurs.
   /// The error value will only be set for those output pins which have an error
   /// mode set to 1.  Those with error mode set to zero will not be changed by
   /// a device failure.
   /// @param id Identifies which group of outputs to effect.
   /// @param value The new error value setting of the output lines.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Dout32SetErrValue( uint8 id, uint32 value ){
      return sdo.Dnld32( IOOBJID_DOUT_32_ERRVAL, id+1, value );
   }
   //@}

   /***************************************************************************/
   /** \name Analog input control

     If the module contains analog inputs, these methods may be used to configure
     and read those inputs.

     Most manufacturers support 16-bit access to analog inputs.  Other input sizes
     are optional in the spec. and may or may not be available.
     */
   /***************************************************************************/
   //@{
   /// Return the number of 8-bit analog inputs available on this device
   /// @param ct The count is returned here.  Zero is returned on error.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain8GetCt( uint8 &ct ){
      ct = 0;
      return sdo.Upld8( IOOBJID_AIN_8_VALUE, 0, ct );
   }

   /// Read an 8-bit analog input.
   /// @param id The analog input channel ID
   /// @param value The analog input value
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain8Read( uint8 id, int8 &value ){
      return sdo.Upld8( IOOBJID_AIN_8_VALUE, id+1, value );
   }

   /// Return the number of 16-bit analog inputs available on this device
   /// @param ct The count is returned here.  Zero is returned on error.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain16GetCt( uint8 &ct ){
      ct = 0;
      return sdo.Upld8( IOOBJID_AIN_16_VALUE, 0, ct );
   }

   virtual const Error *Ain16Read( uint8 id, int16 &value, bool viaSDO=false );

   /// Return the number of 32-bit analog inputs available on this device
   /// @param ct The count is returned here.  Zero is returned on error.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain32GetCt( uint8 &ct ){
      ct = 0;
      return sdo.Upld8( IOOBJID_AIN_32_VALUE, 0, ct );
   }

   /// Read a 32-bit analog input.
   /// @param id The analog input channel ID
   /// @param value The analog input value
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain32Read( uint8 id, int32 &value ){
      return sdo.Upld32( IOOBJID_AIN_32_VALUE, id+1, value );
   }

#ifdef CML_ALLOW_FLOATING_POINT
   /// Return the number of floating point analog inputs available on this device
   /// @param ct The count is returned here.  Zero is returned on error.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinFltGetCt( uint8 &ct ){
      ct = 0;
      return sdo.Upld8( IOOBJID_AIN_FLT_VALUE, 0, ct );
   }

   /// Read a floating point analog input.
   /// @param id The analog input channel ID
   /// @param value The analog input value
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinFltRead( uint8 id, float &value ){
      return sdo.UpldFlt( IOOBJID_AIN_FLT_VALUE, id+1, value );
   }
#endif

   /// Get the analog input offset value as a 32-bit integer.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain32GetOffset( uint8 id, int32 &value ){
      return sdo.Upld32( IOOBJID_AIN_32_OFFSET, id+1, value );
   }

   /// Set the analog input offset value as a 32-bit integer.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain32SetOffset( uint8 id, int32 value ){
      return sdo.Dnld32( IOOBJID_AIN_32_OFFSET, id+1, value );
   }

   /// Get the analog input scaling factor as a 32-bit integer.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain32GetScaling( uint8 id, int32 &value ){
      return sdo.Upld32( IOOBJID_AIN_32_SCALE, id+1, value );
   }

   /// Set the analog input scaling factor as a 32-bit integer.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain32SetScaling( uint8 id, int32 value ){
      return sdo.Dnld32( IOOBJID_AIN_32_SCALE, id+1, value );
   }

#ifdef CML_ALLOW_FLOATING_POINT
   /// Get the analog input offset value as a floating point value.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinFltGetOffset( uint8 id, float &value ){
      return sdo.UpldFlt( IOOBJID_AIN_FLT_OFFSET, id+1, value );
   }

   /// Set the analog input offset value as a floating point value.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinFltSetOffset( uint8 id, float value ){
      return sdo.DnldFlt( IOOBJID_AIN_FLT_OFFSET, id+1, value );
   }

   /// Get the analog input scaling factor as a floating point value.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinFltGetScaling( uint8 id, float &value ){
      return sdo.UpldFlt( IOOBJID_AIN_FLT_SCALE, id+1, value );
   }

   /// Set the analog input scaling factor as a floating point value.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinFltSetScaling( uint8 id, float value ){
      return sdo.DnldFlt( IOOBJID_AIN_FLT_SCALE, id+1, value );
   }
#endif

   /// Get the current setting of the global interrupt enable for analog inputs.
   /// A return value of true indicates that interrupts are enabled, false disabled.
   /// @param value The current interrupt enable setting is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinGetIntEna( bool &value ){
      uint8 v;
      const Error *err = sdo.Upld8( IOOBJID_AIN_INTENA, 0, v );
      value = (v!=0);
      return err;
   }

   /// Set the current setting of the global interrupt enable for analog inputs.
   /// Setting this parameter to true enables interrupts, false disables.
   /// @param value The interrupt enable setting.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinSetIntEna( bool value ){
      const Error *err = sdo.Dnld8( IOOBJID_AIN_INTENA, 0, (uint8)value );
      if( !err ) ainIntEna = value;
      return err;
   }

   /// Get the analog input trigger type associated with the input channel.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinGetTrigType( uint8 id, IO_AIN_TRIG_TYPE &value ){
      uint8 v;
      const Error *err = sdo.Upld8( IOOBJID_AIN_TRIG, id+1, v );
      value = (IO_AIN_TRIG_TYPE)v;
      return err;
   }

   /// Set the analog input trigger type associated with the input channel.
   /// @param id The analog input channel ID
   /// @param value The value to set
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinSetTrigType( uint8 id, IO_AIN_TRIG_TYPE value ){
      return sdo.Dnld8( IOOBJID_AIN_TRIG, id+1, (uint8)value );
   }

   /// Get the analog input interrupt source.  This variable may be used to 
   /// determine which analog input has produced an interrupt.  There are 
   /// eight banks of interrupt source registers, each of which covers 32 
   /// analog inputs in it's 32 bits.  Bank 0 identifies analog inputs 0 to 31,
   /// Bank 1 identifies analog inputs 32 to 63, etc.
   /// The bit associated with the analog input generating the latest interrupt
   /// will be set in the value returned by this read.  Reading this variable
   /// causes all it's bit to be automatically reset.
   /// @param id The bank number to read (0 to 7)
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinGetIntSource( uint8 id, uint32 &value ){
      return sdo.Upld32( IOOBJID_AIN_INTSRC, id+1, value );
   }

   /// Get the analog input upper limit value as a 16-bit integer.
   /// The upper limit defines the value at which an interrupt will be 
   /// generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain16GetUpperLimit( uint8 id, int16 &value ){
      return sdo.Upld16( IOOBJID_AIN_32_UPLIM, id+1, value );
   }

   /// Set the analog input upper limit value as a 16-bit integer.
   /// The upper limit defines the value at which an interrupt will be 
   /// generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain16SetUpperLimit( uint8 id, int16 value ){
      return sdo.Dnld16( IOOBJID_AIN_32_UPLIM, id+1, value );
   }

   /// Get the analog input lower limit value as a 16-bit integer.
   /// The lower limit defines the value at which an interrupt will be 
   /// generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain16GetLowerLimit( uint8 id, int16 &value ){
      return sdo.Upld16( IOOBJID_AIN_32_LWLIM, id+1, value );
   }

   /// Set the analog input lower limit value as a 16-bit integer.
   /// The lower limit defines the value at which an interrupt will be 
   /// generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain16SetLowerLimit( uint8 id, int16 value ){
      return sdo.Dnld16( IOOBJID_AIN_32_LWLIM, id+1, value );
   }

   /// Get the analog input unsigned delta value as a 16-bit integer.
   /// The unsigned delta defines the amount of change at which an 
   /// interrupt will be generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain16GetUnsignedDelta( uint8 id, int16 &value ){
      return sdo.Upld16( IOOBJID_AIN_32_UDELTA, id+1, value );
   }

   /// Set the analog input unsigned delta value as a 16-bit integer.
   /// The unsigned delta defines the amount of change at which an 
   /// interrupt will be generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain16SetUnsignedDelta( uint8 id, int16 value ){
      return sdo.Dnld16( IOOBJID_AIN_32_UDELTA, id+1, value );
   }

   /// Get the analog input negative delta value as a 16-bit integer.
   /// The negative delta defines the amount of change at which an 
   /// interrupt will be generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain16GetNegativeDelta( uint8 id, int16 &value ){
      return sdo.Upld16( IOOBJID_AIN_32_NDELTA, id+1, value );
   }

   /// Set the analog input negative delta value as a 16-bit integer.
   /// The negative delta defines the amount of change at which an 
   /// interrupt will be generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain16SetNegativeDelta( uint8 id, int16 value ){
      return sdo.Dnld16( IOOBJID_AIN_32_NDELTA, id+1, value );
   }

   /// Get the analog input positive delta value as a 16-bit integer.
   /// The positive delta defines the amount of change at which an 
   /// interrupt will be generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain16GetPositiveDelta( uint8 id, int16 &value ){
      return sdo.Upld16( IOOBJID_AIN_32_PDELTA, id+1, value );
   }

   /// Set the analog input positive delta value as a 16-bit integer.
   /// The positive delta defines the amount of change at which an 
   /// interrupt will be generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain16SetPositiveDelta( uint8 id, int16 value ){
      return sdo.Dnld16( IOOBJID_AIN_32_PDELTA, id+1, value );
   }

   /// Get the analog input upper limit value as a 32-bit integer.
   /// The upper limit defines the value at which an interrupt will be 
   /// generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain32GetUpperLimit( uint8 id, int32 &value ){
      return sdo.Upld32( IOOBJID_AIN_32_UPLIM, id+1, value );
   }

   /// Set the analog input upper limit value as a 32-bit integer.
   /// The upper limit defines the value at which an interrupt will be 
   /// generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain32SetUpperLimit( uint8 id, int32 value ){
      return sdo.Dnld32( IOOBJID_AIN_32_UPLIM, id+1, value );
   }

   /// Get the analog input lower limit value as a 32-bit integer.
   /// The lower limit defines the value at which an interrupt will be 
   /// generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain32GetLowerLimit( uint8 id, int32 &value ){
      return sdo.Upld32( IOOBJID_AIN_32_LWLIM, id+1, value );
   }

   /// Set the analog input lower limit value as a 32-bit integer.
   /// The lower limit defines the value at which an interrupt will be 
   /// generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain32SetLowerLimit( uint8 id, int32 value ){
      return sdo.Dnld32( IOOBJID_AIN_32_LWLIM, id+1, value );
   }

   /// Get the analog input unsigned delta value as a 32-bit integer.
   /// The unsigned delta defines the amount of change at which an 
   /// interrupt will be generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain32GetUnsignedDelta( uint8 id, int32 &value ){
      return sdo.Upld32( IOOBJID_AIN_32_UDELTA, id+1, value );
   }

   /// Set the analog input unsigned delta value as a 32-bit integer.
   /// The unsigned delta defines the amount of change at which an 
   /// interrupt will be generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain32SetUnsignedDelta( uint8 id, int32 value ){
      return sdo.Dnld32( IOOBJID_AIN_32_UDELTA, id+1, value );
   }

   /// Get the analog input negative delta value as a 32-bit integer.
   /// The negative delta defines the amount of change at which an 
   /// interrupt will be generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain32GetNegativeDelta( uint8 id, int32 &value ){
      return sdo.Upld32( IOOBJID_AIN_32_NDELTA, id+1, value );
   }

   /// Set the analog input negative delta value as a 32-bit integer.
   /// The negative delta defines the amount of change at which an 
   /// interrupt will be generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain32SetNegativeDelta( uint8 id, int32 value ){
      return sdo.Dnld32( IOOBJID_AIN_32_NDELTA, id+1, value );
   }

   /// Get the analog input positive delta value as a 32-bit integer.
   /// The positive delta defines the amount of change at which an 
   /// interrupt will be generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain32GetPositiveDelta( uint8 id, int32 &value ){
      return sdo.Upld32( IOOBJID_AIN_32_PDELTA, id+1, value );
   }

   /// Set the analog input positive delta value as a 32-bit integer.
   /// The positive delta defines the amount of change at which an 
   /// interrupt will be generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Ain32SetPositiveDelta( uint8 id, int32 value ){
      return sdo.Dnld32( IOOBJID_AIN_32_PDELTA, id+1, value );
   }

#ifdef CML_ALLOW_FLOATING_POINT
   /// Get the analog input upper limit value as a floating point value.
   /// The upper limit defines the value at which an interrupt will be 
   /// generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinFltGetUpperLimit( uint8 id, float &value ){
      return sdo.UpldFlt( IOOBJID_AIN_FLT_UPLIM, id+1, value );
   }

   /// Set the analog input upper limit value as a floating point value.
   /// The upper limit defines the value at which an interrupt will be 
   /// generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinFltSetUpperLimit( uint8 id, float value ){
      return sdo.DnldFlt( IOOBJID_AIN_FLT_UPLIM, id+1, value );
   }

   /// Get the analog input lower limit value as a floating point value.
   /// The lower limit defines the value at which an interrupt will be 
   /// generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinFltGetLowerLimit( uint8 id, float &value ){
      return sdo.UpldFlt( IOOBJID_AIN_FLT_LWLIM, id+1, value );
   }

   /// Set the analog input lower limit value as a floating point value.
   /// The lower limit defines the value at which an interrupt will be 
   /// generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinFltSetLowerLimit( uint8 id, float value ){
      return sdo.DnldFlt( IOOBJID_AIN_FLT_LWLIM, id+1, value );
   }

   /// Get the analog input unsigned delta value as a floating point value.
   /// The unsigned delta defines the amount of change at which an 
   /// interrupt will be generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinFltGetUnsignedDelta( uint8 id, float &value ){
      return sdo.UpldFlt( IOOBJID_AIN_FLT_UDELTA, id+1, value );
   }

   /// Set the analog input unsigned delta value as a floating point value.
   /// The unsigned delta defines the amount of change at which an 
   /// interrupt will be generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinFltSetUnsignedDelta( uint8 id, float value ){
      return sdo.DnldFlt( IOOBJID_AIN_FLT_UDELTA, id+1, value );
   }

   /// Get the analog input negative delta value as a floating point value.
   /// The negative delta defines the amount of change at which an 
   /// interrupt will be generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinFltGetNegativeDelta( uint8 id, float &value ){
      return sdo.UpldFlt( IOOBJID_AIN_FLT_NDELTA, id+1, value );
   }

   /// Set the analog input negative delta value as a floating point value.
   /// The negative delta defines the amount of change at which an 
   /// interrupt will be generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinFltSetNegativeDelta( uint8 id, float value ){
      return sdo.DnldFlt( IOOBJID_AIN_FLT_NDELTA, id+1, value );
   }

   /// Get the analog input positive delta value as a floating point value.
   /// The positive delta defines the amount of change at which an 
   /// interrupt will be generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinFltGetPositiveDelta( uint8 id, float &value ){
      return sdo.UpldFlt( IOOBJID_AIN_FLT_PDELTA, id+1, value );
   }

   /// Set the analog input positive delta value as a floating point value.
   /// The positive delta defines the amount of change at which an 
   /// interrupt will be generated if it is enabled.
   /// @param id The analog input channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AinFltSetPositiveDelta( uint8 id, float value ){
      return sdo.DnldFlt( IOOBJID_AIN_FLT_PDELTA, id+1, value );
   }
#endif
   //@}

   /***************************************************************************/
   /** \name Analog output control

     If the module contains analog outputs, these methods may be used to configure
     and write to those outputs.

     Most manufacturers support 16-bit access to analog inputs.  Other input sizes
     are optional in the spec. and may or may not be available.
     */
   /***************************************************************************/
   //@{

   /// Return the number of 8-bit analog outputs available on this device
   /// @param ct The count is returned here.  Zero is returned on error.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Aout8GetCt( uint8 &ct ){
      ct = 0;
      return sdo.Upld8( IOOBJID_AOUT_8_VALUE, 0, ct );
   }

   /// Write to an 8-bit analog output.
   /// @param id The analog input channel ID
   /// @param value The value to write.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Aout8Write( uint8 id, int8 value ){
      return sdo.Dnld8( IOOBJID_AOUT_8_VALUE, id+1, value );
   }

   /// Return the number of 16-bit analog outputs available on this device
   /// @param ct The count is returned here.  Zero is returned on error.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Aout16GetCt( uint8 &ct ){
      ct = 0;
      return sdo.Upld8( IOOBJID_AOUT_16_VALUE, 0, ct );
   }

   virtual const Error *Aout16Write( uint8 id, int16 value, bool viaSDO=false );

   /// Return the number of 32-bit analog outputs available on this device
   /// @param ct The count is returned here.  Zero is returned on error.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Aout32GetCt( uint8 &ct ){
      ct = 0;
      return sdo.Upld8( IOOBJID_AOUT_32_VALUE, 0, ct );
   }

   /// Write to a 32-bit analog output.
   /// @param id The analog input channel ID
   /// @param value The value to write.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Aout32Write( uint8 id, int32 value ){
      return sdo.Dnld32( IOOBJID_AOUT_32_VALUE, id+1, value );
   }

#ifdef CML_ALLOW_FLOATING_POINT
   /// Return the number of floating point analog outputs available on this device
   /// @param ct The count is returned here.  Zero is returned on error.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AoutFltGetCt( uint8 &ct ){
      ct = 0;
      return sdo.Upld8( IOOBJID_AOUT_FLT_VALUE, 0, ct );
   }

   /// Write to a floating point analog output.
   /// @param id The analog input channel ID
   /// @param value The value to write.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AoutFltWrite( uint8 id, float value ){
      return sdo.DnldFlt( IOOBJID_AOUT_FLT_VALUE, id+1, value );
   }
#endif

   /// Get the analog output offset value as a 32-bit integer.
   /// @param id The analog output channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Aout32GetOffset( uint8 id, int32 &value ){
      return sdo.Upld32( IOOBJID_AOUT_32_OFFSET, id+1, value );
   }

   /// Set the analog output offset value as a 32-bit integer.
   /// @param id The analog output channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Aout32SetOffset( uint8 id, int32 value ){
      return sdo.Dnld32( IOOBJID_AOUT_32_OFFSET, id+1, value );
   }

   /// Get the analog output scaling factor as a 32-bit integer.
   /// @param id The analog output channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Aout32GetScaling( uint8 id, int32 &value ){
      return sdo.Upld32( IOOBJID_AOUT_32_SCALE, id+1, value );
   }

   /// Set the analog output scaling factor as a 32-bit integer.
   /// @param id The analog output channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Aout32SetScaling( uint8 id, int32 value ){
      return sdo.Dnld32( IOOBJID_AOUT_32_SCALE, id+1, value );
   }

#ifdef CML_ALLOW_FLOATING_POINT
   /// Get the analog output offset value as a floating point value.
   /// @param id The analog output channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AoutFltGetOffset( uint8 id, float &value ){
      return sdo.UpldFlt( IOOBJID_AOUT_FLT_OFFSET, id+1, value );
   }

   /// Set the analog output offset value as a floating point value.
   /// @param id The analog output channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AoutFltSetOffset( uint8 id, float value ){
      return sdo.DnldFlt( IOOBJID_AOUT_FLT_OFFSET, id+1, value );
   }

   /// Get the analog output scaling factor as a floating point value.
   /// @param id The analog output channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AoutFltGetScaling( uint8 id, float &value ){
      return sdo.UpldFlt( IOOBJID_AOUT_FLT_SCALE, id+1, value );
   }

   /// Set the analog output scaling factor as a floating point value.
   /// @param id The analog output channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AoutFltSetScaling( uint8 id, float value ){
      return sdo.DnldFlt( IOOBJID_AOUT_FLT_SCALE, id+1, value );
   }
#endif

   /// Get the analog output error mode.
   /// If the error mode is true, then the analog output will change it's value
   /// to the programmed error value in the case of a device failure.  If false,
   /// a device failure will not cause a change in the analog output value.
   /// @param id The analog output channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AoutGetErrMode( uint8 id, bool &value ){
      uint8 v;
      const Error *err = sdo.Upld8( IOOBJID_AOUT_ERRMODE, id+1, v );
      value = (v!=0);
      return err;
   }

   /// Set the analog output error mode.
   /// If the error mode is true, then the analog output will change it's value
   /// to the programmed error value in the case of a device failure.  If false,
   /// a device failure will not cause a change in the analog output value.
   /// @param id The analog output channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AoutSetErrMode( uint8 id, bool value ){
      return sdo.Dnld8( IOOBJID_AOUT_ERRMODE, id+1, (uint8)value );
   }

   /// Get the analog output error value as a 32-bit integer.
   /// The error value is the value that the analog output will assume
   /// on device error, if it's error mode is set to true.
   /// @param id The analog output channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Aout32GetErrValue( uint8 id, int32 &value ){
      return sdo.Upld32( IOOBJID_AOUT_32_ERRVAL, id+1, value );
   }

   /// Set the analog output error value as a 32-bit integer.
   /// The error value is the value that the analog output will assume
   /// on device error, if it's error mode is set to true.
   /// @param id The analog output channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Aout32SetErrValue( uint8 id, int32 value ){
      return sdo.Dnld32( IOOBJID_AOUT_32_ERRVAL, id+1, value );
   }

   /// Get the analog output error value as a 16-bit integer.
   /// The error value is the value that the analog output will assume
   /// on device error, if it's error mode is set to true.
   /// @param id The analog output channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Aout16GetErrValue( uint8 id, int16 &value ){
      return sdo.Upld16( IOOBJID_AOUT_32_ERRVAL, id+1, value );
   }

   /// Set the analog output error value as a 32-bit integer.
   /// The error value is the value that the analog output will assume
   /// on device error, if it's error mode is set to true.
   /// @param id The analog output channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *Aout16SetErrValue( uint8 id, int16 value ){
      return sdo.Dnld16( IOOBJID_AOUT_32_ERRVAL, id+1, value );
   }

#ifdef CML_ALLOW_FLOATING_POINT
   /// Get the analog output error value as a floating point value.
   /// The error value is the value that the analog output will assume
   /// on device error, if it's error mode is set to true.
   /// @param id The analog output channel ID
   /// @param value The value is returned here.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AoutFltGetErrValue( uint8 id, float &value ){
      return sdo.UpldFlt( IOOBJID_AOUT_FLT_ERRVAL, id+1, value );
   }

   /// Set the analog output error value as a floating point value.
   /// The error value is the value that the analog output will assume
   /// on device error, if it's error mode is set to true.
   /// @param id The analog output channel ID
   /// @param value The value to be set.
   /// @return A pointer to an error object, or NULL on success
   virtual const Error *AoutFltSetErrValue( uint8 id, float value ){
      return sdo.DnldFlt( IOOBJID_AOUT_FLT_ERRVAL, id+1, value );
   }
#endif
   //@}

protected:

   EventMap  eventMap;

   bool dinIntEna;
   bool ainIntEna;

   /// Upload a setting for a single digital I/O pin.  The object dictionary
   /// entry is calculated based on the pin ID and the base object dictionary
   /// ID passed.
   /// @param base The object dictionary base index for this parameter
   /// @param id The I/O pin ID.  Must range from 0 to 1024
   /// @param value The boolean value is returned here
   /// @return A pointer to an error object, or NULL on success
   const Error *BitUpld( uint16 base, uint16 id, bool &value )
   {
      if( id >= 1024 ) return &IOError::BadID;
      base += id>>7;

      uint8 v;
      const Error *err = sdo.Upld8( base, (uint8)( (id&0x7F)+1 ), v );
      value = (v!=0);
      return err;
   }

   /// Download a setting for a single digital I/O pin.  The object dictionary
   /// index/sub-index is calculated from the pin ID and a passed base address.
   /// @param base The object dictionary base index for this parameter
   /// @param id The I/O pin ID.  Must range from 0 to 1024
   /// @param value The boolean value is passed here.
   /// @return A pointer to an error object, or NULL on success
   const Error *BitDnld( uint16 base, uint16 id, bool value )
   {
      if( id >= 1024 ) return &IOError::BadID;
      base += id>>7;
      return sdo.Dnld8( base, (uint8)( (id&0x7F)+1 ), (uint8)value );
   }

   /// Count the number of individual I/O pins available on the device.
   /// @param base The base index for the object dictionary
   /// @param ct The count is returned here
   /// @return A pointer to an error object, or NULL on success
   const Error *BitCount( uint16 base, uint16 &ct )
   {
      const Error *err = 0;

      ct = 0;
      for( int i=0; i<8; i++ )
      {
	 uint8 x;
	 err = sdo.Upld8( base++, 0, x );

	 if( err )
	    break;
	 ct += x;
      }

      if( !ct )
	 return err;

      return 0;
   }

#ifdef CML_ENABLE_IOMODULE_PDOS

   /***************************************************************************/
   /**
     Receive PDO for mapping digital output pins.  This class represents the 
     standard receive PDO into which up to 64 digital output pins may be 
     mapped.
     */
   /***************************************************************************/
   class DigOutPDO: public RPDO
   {
      /// These objects holds the raw output data
      Pmap8 out[8];

      /// This points to the IOModule object
      class IOModule *io;

      public:
      const Error *Init( class IOModule *io, uint32 cobID, uint8 ct, uint8 id[] );
      bool Update( uint8 id, uint8 value );
      bool UpdateBit( uint16 id, bool value );
      const Error *Transmit( void );
   };

   /***************************************************************************/
   /**
     Receive PDO for mapping analog outputs.  This class represents the standard
     receive PDO which can be used to transmit up to 4 16-bit analog outputs.
     */
   /***************************************************************************/
   class AlgOutPDO: public RPDO
   {
      /// These objects map the analog outputs
      Pmap16 out[4];

      /// This points to the IOModule object
      class IOModule *io;

      public:
      const Error *Init( class IOModule *io, uint32 cobID, uint8 ct, uint8 id[] );
      bool Update( uint8 id, int16 value );
      const Error *Transmit( void );
   };

   /***************************************************************************/
   /**
     Transmit PDO for mapping digital inputs.  This class represents the 
     standard transmit PDO into which up to 64 digital inputs may be 
     mapped.
     */
   /***************************************************************************/
   class DigInPDO: public TPDO
   {
      /// These objects map the raw digital input data
      Pmap8 in[8];

      /// This points to the IOModule object
      class IOModule *io;

      /// This holds the event mask that will be posted to the
      /// I/O modules event map when the PDO is received.
      IOMODULE_EVENTS eventMask;

      public:
      const Error *Init( class IOModule *io, uint32 cobID, uint8 ct, uint8 id[], IOMODULE_EVENTS event );
      bool GetInVal( uint8 id, uint8 &value );
      bool GetBitVal( uint16 id, bool &value );
      void Received( void );
   };

   /***************************************************************************/
   /**
     Transmit PDO for mapping analog inputs.  This class represents the standard
     transmit PDO which can be used to map up to 4 16-bit analog inputs.
     */
   /***************************************************************************/
   class AlgInPDO: public TPDO
   {
      /// These objects map the analog inputs
      Pmap16 in[4];

      /// This points to the IOModule object
      class IOModule *io;

      /// This holds the event mask that will be posted to the
      /// I/O modules event map when the PDO is received.
      IOMODULE_EVENTS eventMask;

      public:
      const Error *Init( class IOModule *io, uint32 cobID, uint8 ct, uint8 id[], IOMODULE_EVENTS event );
      bool GetInVal( uint8 id, int16 &value );
      void Received( void );
   };


   /// Default PDO used to transmit digital output info
   DigOutPDO doutPDO;
   DigInPDO  dinPDO;
   AlgOutPDO aoutPDO[3];
   AlgInPDO  ainPDO[3];

   /// Post an event condition to the I/O module's event map.
   /// This method is used internally by the various standard
   /// transmit PDOs when a new PDO message is received.
   /// @param event The event bit(s) to post
   virtual void PostIOEvent( IOMODULE_EVENTS event )
   {
      eventMap.setBits( (uint32)event );
      eventMap.clrBits( (uint32)event );
   }

   friend class DigOutPDO;
   friend class DigInPDO;
   friend class AlgOutPDO;
   friend class AlgInPDO;
#endif

private:
   /// Private copy constructor (not supported)
   IOModule( const IOModule& );

   /// Private assignment operator (not supported)
   IOModule& operator=( const IOModule& );
};


CML_NAMESPACE_END()

#endif

