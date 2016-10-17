/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/***************************************************************************/
/** \file
This file contains the AMP object methods used to upload / download various
amplifier parameters.
*/
/***************************************************************************/

#include "CML.h"

CML_NAMESPACE_USE();

#define VERSION( major, minor )    ((major<<8) | minor)

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
                         Amplifier state info
*/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Get the actual position used by the servo loop.  For dual encoder systems,
  this will be the load encoder position.  To get the motor encoder position
  on such a system, use Amp::GetPositionMotor.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetPositionActual( uunit &value )
{
   int32 v;
   const Error *err = sdo.Upld32( OBJID_POS_LOAD, 0, v );
   value = PosLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Get the actual motor position.  For single encoder systems, this value is
  identical to the value returned by Amp::GetPositionActual.

  For dual encoder systems, this function returns the actual motor position
  and Amp::GetPositionActual may be used to get the load encoder position.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetPositionMotor( uunit &value )
{
   int32 v;
   const Error *err = sdo.Upld32( OBJID_POS_MTR, 0, v );
   value = PosMtr2User( v );
   return err;
}

/***************************************************************************/
/**
  Set the actual position.  On dual encoder systems, this will set the load
  encoder position.  Amp::SetPositionMotor may be used to set the motor 
  position on such systems.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value The actual position of the motor.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetPositionActual( uunit value )
{
   return sdo.Dnld32( OBJID_POS_LOAD, 0, PosUser2Load(value) );
}

/***************************************************************************/
/**
  Set the actual motor position.  On dual encoder systems, this will set the 
  motor encoder position.  Amp::SetPositionActual may be used to set the load
  position on such systems.

  @param value The actual position of the motor.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetPositionMotor( uunit value )
{
   return sdo.Dnld32( OBJID_POS_MTR, 0, PosUser2Mtr(value) );
}

/***************************************************************************/
/**
  Get the instantaneous commanded position.  

  This position is the command input to the servo loop.  The commanded position
  is calculated by the trajectory generator and updated every servo cycle.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetPositionCommand( uunit &value )
{
   int32 v;
   const Error *err = sdo.Upld32( OBJID_POS_CMD, 0, v );
   value = PosLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Get the position error.  This is the difference between the position demand 
  value and the actual position.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetPositionError( uunit &value )
{ 
   int32 v;
   const Error *err = sdo.Upld32( OBJID_POS_ERR, 0, v );
   value = PosLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Get the actual motor velocity.  The motor velocity is estimated by the
  amplifier based on the change in position seen at the encoder.
  For dual encoder systems, the load encoder veloctiy can be queried using
  the function Amp::GetVelocityLoad.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetVelocityActual( uunit &value )
{ 
   int32 v;
   const Error *err = sdo.Upld32( OBJID_VEL_MTR, 0, v ); 
   value = VelMtr2User( v );
   return err;
}

/***************************************************************************/
/**
  Get the load encoder velocity.  The load velocity is estimated by the
  amplifier based on the change in position seen at the load encoder.
  For dual encoder systems, the motor encoder veloctiy can be queried using
  the function Amp::GetVelocityActual.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetVelocityLoad( uunit &value )
{ 
   int32 v;
   const Error *err = sdo.Upld32( OBJID_VEL_LOAD, 0, v ); 
   value = VelLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Get the commanded velocity.  The commanded velocity is the velocity value
  that is passed to the velocity limiter, and from there to the velocity 
  control loop.  If the amplifier is in position mode (i.e. the position loop
  is active), then this velocity is the output of the position control loop.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetVelocityCommand( uunit &value )
{ 
   int32 v;
   const Error *err = sdo.Upld32( OBJID_VEL_CMD, 0, v ); 
   value = VelMtr2User( v );
   return err;
}

/***************************************************************************/
/**
  Get the limited velocity.  This velocity is the result of applying the 
  velocity limiter to the commanded velocity (see GetVelocityCommand).

  When the velocity loop is being driven by the position loop, the velocity
  limiter consists of a maximum velocity value only.  When some other source
  is driving the velocity loop, the limiter also includes a maximum acceleration
  and deceleration value.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetVelocityLimited( uunit &value )
{ 
   int32 v;
   const Error *err = sdo.Upld32( OBJID_VEL_LIM, 0, v ); 
   value = VelMtr2User( v );
   return err;
}

/***************************************************************************/
/**
  Get the actual motor current.  This current is based on the amplifiers current
  sensors, and indicates the portion of current that is being used to generate
  torque in the motor.

  The current is returned in units of 0.01 amps.

  @param value A variable that will store the returned value.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetCurrentActual( int16 &value )
{
   return sdo.Upld16( OBJID_CRNT_ACT, 0, value );
}

/***************************************************************************/
/**
  Get the commanded motor current.  This current is the input to the current 
  limiter.  This value is also the output of the velocity loop when the 
  motor is in either position or velocity control mode.  When in current control 
  mode, the commanded current is derived from the control source (analog input, 
  PWM input, function generator, etc).

  The current is returned in units of 0.01 amps.

  @param value A variable that will store the returned value.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetCurrentCommand( int16 &value )
{
   return sdo.Upld16( OBJID_CRNT_CMD, 0, value );
}

/***************************************************************************/
/**
  Get the limited motor current.  The commanded current (GetCurrentCommand) is
  passed to a current limiter.  The output of the current limiter is the limited
  current which is passed as an input to the current loop.

  The current is returned in units of 0.01 amps.

  @param value A variable that will store the returned value.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetCurrentLimited( int16 &value )
{
   return sdo.Upld16( OBJID_CRNT_LIM, 0, value );
}

/***************************************************************************/
/**
  Get the instantaneous commanded velocity passed out of the trajectory 
  generator.  This velocity is used by the position loop to calculate it's 
  velocity feed forward term.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetTrajectoryVel( uunit &value )
{ 
   int32 v;
   const Error *err = sdo.Upld32( OBJID_TRJ_VEL, 0, v ); 
   value = VelLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Get the instantaneous commanded acceleration passed out of the trajectory 
  generator.  This acceleration is used by the position loop to calculate it's 
  acceleration feed forward term.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetTrajectoryAcc( uunit &value )
{ 
   int32 v;
   const Error *err = sdo.Upld32( OBJID_TRJ_ACC, 0, v ); 
   value = AccLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Get the motor phase angle.  The phase angle describes the motor's electrical 
  position with respect to it's windings.  It's an internal parameter used by 
  the amplifier to commutate a brushless motor.

  The angle is returned in degrees.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetPhaseAngle( int16 &value )
{ 
   return sdo.Upld16( OBJID_PHASE_ANG, 0, value ); 
}

/***************************************************************************/
/**
  Get the current digital hall sensor state.  The hall state is the value of 
  the hall sensors after any adjustments have been made to them based on the
  hallWiring parameter of the MtrInfo structure.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetHallState( int16 &value )
{ 
   return sdo.Upld16( OBJID_HALL_STATE, 0, value ); 
}

/***************************************************************************/
/**
  Get the analog reference input voltage.  If the amplifier has an analog
  reference input, it's value will be returned in millivolts.

  @param value The value will be returned in this variable
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetRefVoltage( int16 &value )
{
   return sdo.Upld16( OBJID_ANALOG_REF, 0, value );
}

/***************************************************************************/
/**
  Get the high voltage bus voltage in units of 0.1 volts.

  @param value The value will be returned in this variable
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetHighVoltage( int16 &value )
{
   return sdo.Upld16( OBJID_HVREF, 0, value );
}

/***************************************************************************/
/**
  Get the current amplifier temperature (degrees C).

  @param value The value will be returned in this variable
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetAmpTemp( int16 &value )
{
   return sdo.Upld16( OBJID_AMPTEMP, 0, value );
}

/***************************************************************************/
/**
  Get the raw voltage on the two analog encoder inputs (0.1 millivolt units).
  If the amplifier has analog encoder inputs, then they will be read and 
  returned.

  @param sin The sine input of the analog encoder will be returned here.
  @param cos The cosine input of the analog encoder will be returned here.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetAnalogEncoder( int16 &sin, int16 &cos )
{
   const Error *err = sdo.Upld16( OBJID_AENC_SIN, 0, sin );
   if( !err ) err = sdo.Upld16( OBJID_AENC_COS, 0, cos );
   return err;
}

/***************************************************************************/
/**
  Get the actual current values read directly from the amplifier's current
  sensors.

  Note that if the motor wiring is being swapped in software, the U and V
  reading will be swapped.

  @param u The U winding current will be returned here.
  @param v The V winding current will be returned here.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetMotorCurrent( int16 &u, int16 &v )
{
   const Error *err = sdo.Upld16( OBJID_CNRT_U, 0, u );
   if( !err ) err = sdo.Upld16( OBJID_CRNT_V, 0, v );
   return err;
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
   Misc configuration parameters
   */
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Set the PWM output mode configuration for the amplifier.
  @param mode The PWM output mode to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetPwmMode( AMP_PWM_MODE mode )
{
   return sdo.Dnld16( OBJID_PWM_MODE, 0, (uint16)mode );
}

/***************************************************************************/
/**
  Get the current PWM output mode configuration from the amplifier.
  @param mode The mode information will be returned here.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetPwmMode( AMP_PWM_MODE &mode )
{
   uint16 m;
   const Error *err = sdo.Upld16( OBJID_PWM_MODE, 0, m );
   mode = (AMP_PWM_MODE)m;
   return err;
}

/***************************************************************************/
/**
  Set the phasing mode configuration for the amplifier.
  @param mode The phasing mode to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetPhaseMode( AMP_PHASE_MODE mode )
{
   return sdo.Dnld16( OBJID_PHASE_MODE, 0, (uint16)mode );
}

/***************************************************************************/
/**
  Get the current phasing mode configuration from the amplifier.
  @param mode The mode information will be returned here.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetPhaseMode( AMP_PHASE_MODE &mode )
{
   uint16 m;
   const Error *err = sdo.Upld16( OBJID_PHASE_MODE, 0, m );
   mode = (AMP_PHASE_MODE)m;
   return err;
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
   I/O pins and status registers
   */
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/


/***************************************************************************/
/**
  Get the present value of the general purpose input pins.

  The input pin values are returned one per bit.  The value of input pin
  1 will be returned in bit 0 (1 if high, 0 if low), pin 2 will be in bit 1, 
  etc.

  @param value variable that will store the returned value
  @param viaSDO If true, an SDO will be used to read the input pins.
  If false (default), the most recent input value received from
  the amplifier via PDO will be returned.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetInputs( uint16 &value, bool viaSDO )
{
   if( viaSDO )
      return sdo.Upld16( OBJID_INPUTS, 0, value ); 

   else
   {
      value = (uint16)inputStateMap.getMask();
      return 0;
   }
}

/***************************************************************************/
/**
  Set the current state of the input pin pull up/down resisters.

  Pull up/down resisters control how an undriven input pin will be interpreted
  by the amplifier.  Depending on the model of amplifier being controlled, there
  may be zero or more groups of pull up/down resisters attached to some the input
  pins.

  Each bit of this register is used to control one group of pull up/down resisters.
  Bit 0 controls group 0, etc. 

  Please refer to the amplifier data sheet for details on the number of groups
  of pull up/down resisters, and which input pins are included in each group.

  @param value The new value to write to the pull up/down control register.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetIoPullup( uint16 value )
{
   return sdo.Dnld16( OBJID_PULLUPS, 0, value );
}

/***************************************************************************/
/**
  Get the current state of the input pin pull up/down resisters.

  Pull up/down resisters control how an undriven input pin will be interpreted
  by the amplifier.  Depending on the model of amplifier being controlled, there
  may be zero or more groups of pull up/down resisters attached to some the input
  pins.

  Each bit of this register is used to control one group of pull up/down resisters.
  Bit 0 controls group 0, etc. 

  Please refer to the amplifier data sheet for details on the number of groups
  of pull up/down resisters, and which input pins are included in each group.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetIoPullup( uint16 &value )
{
   return sdo.Upld16( OBJID_PULLUPS, 0, value );
}

/***************************************************************************/
/**
  Set the input pin configuration for the specified input pin.

  Each of the amplifier input pins can be configured to perform some
  function.  This function configures the specified input to perform
  the specified function.

  @param pin The input pin to configure.  Input pins are numbered starting
  from 0.  Check the amplifier datasheet for the number of input pins
  available.
  @param cfg The input pin function to be assigned to this pin.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetInputConfig( int8 pin, INPUT_PIN_CONFIG cfg )
{
   if( pin < 0 ) return &SDO_Error::Subindex;
   return sdo.Dnld16( OBJID_INPUT_CFG, pin+1, (uint16)cfg );
}

/***************************************************************************/
/**
  Get the input pin configuration for the specified input pin.

  Each of the amplifier input pins can be configured to perform some
  function.  This function configures the specified input to perform
  the specified function.

  @param pin The input pin to check.  Input pins are numbered starting
  from 0.  Check the amplifier datasheet for the number of input pins
  available.
  @param cfg The input pin function will be returned in this variable.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetInputConfig( int8 pin, INPUT_PIN_CONFIG &cfg )
{
   uint16 c;
   if( pin < 0 ) return &SDO_Error::Subindex;
   const Error *err = sdo.Upld16( OBJID_INPUT_CFG, pin+1, c );
   cfg = (INPUT_PIN_CONFIG)c;
   return err;
}

/***************************************************************************/
/**
  Set the input pin debounce time for the specified input pin.

  Each of the amplifier input pins can be configured ignore transient 
  states that last less then the debounce time.  This function configures 
  the debounce time for a specific pin.

  @param pin The input pin to configure.  Input pins are numbered starting
  from 0.  Check the amplifier datasheet for the number of input pins
  available.
  @param value The debounce time to use (milliseconds)
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetInputDebounce( int8 pin, int16 value )
{
   if( pin < 0 ) return &SDO_Error::Subindex;
   return sdo.Dnld16( OBJID_DEBOUNCE, pin+1, value );
}

/***************************************************************************/
/**
  Get the input pin debounce time for the specified input pin.

  @param pin The input pin to configure.  Input pins are numbered starting
  from 0.  Check the amplifier datasheet for the number of input pins
  available.
  @param value The pins debounce time (milliseconds) is returned here.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetInputDebounce( int8 pin, int16 &value )
{
   if( pin < 0 ) return &SDO_Error::Subindex;
   return sdo.Upld16( OBJID_DEBOUNCE, pin+1, value );
}

/***************************************************************************/
/**
  Set the output pin configuration for the specified pin.

  Each of the amplifier output pins can be configured to perform some
  function.  These functions break down into several basic modes:

  - manual mode: In this mode, the output pin will be controlled through the
    CANopen network using the Amp::SetOutputs function.  Output pins can be
    configured to be either active high or active low in this mode.

  - Status word tracking:  In this mode, the output pin is configured to track
    one or more bits of one of the amplifier's internal status words.  A 32-bit
    mask is also supplied which identifies which bits are to be tracked.  If any
    of the selected bits are set in the status word, the output pin will go 
    active.

  - Position trigger.  In this mode the output pin will be configured to go active
    based on the position of the motor.  In some cases the output will go active between
    two programmed positions.  In other cases the output will be triggered by crossing 
    a position and will stay active for a programmed duration.

  @param pin     The output pin to configure.  Output pins are numbered starting
                 from 0.  Check the amplifier datasheet for the number of output pins
                 available.

  @param cfg     The pin function to be assigned to this pin.

  @param param1  A 32-bit parameter used in conjunction with the output pin configuration
                 to define the pin behavior.  For most simple output pin modes this parameter
		 is a bitmask that selects bits in a status register that the output should
		 track.  If the output pin is being configured for manual mode, then the mask 
		 is not used and does not need to be specified.

  @param param2  A second 32-bit parameter used in a few output pin configurations.

  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetOutputConfig( int8 pin, OUTPUT_PIN_CONFIG cfg, uint32 param1, uint32 param2 )
{
   if( pin < 0 ) return &SDO_Error::Subindex;

   uint16 c = (uint16)cfg;
   byte buff[10];

   buff[0] = ByteCast(c);
   buff[1] = ByteCast(c>>8);
   buff[2] = ByteCast(param1);
   buff[3] = ByteCast(param1>>8);
   buff[4] = ByteCast(param1>>16);
   buff[5] = ByteCast(param1>>24);

   // In version 4.77 firmware we added additional data to the output
   // configuration settings.  This allows us to program the outputs
   // with more complex functions
   if( SwVersionNum < VERSION(4,77) )
      return sdo.Download( OBJID_OUTPUT_CFG, pin+1, 6, buff );

   buff[6] = ByteCast(param2);
   buff[7] = ByteCast(param2>>8);
   buff[8] = ByteCast(param2>>16);
   buff[9] = ByteCast(param2>>24);
   return sdo.Download( OBJID_OUTPUT_CFG, pin+1, 10, buff );
}

/***************************************************************************/
/**
  Get the output pin configuration for the specified pin.
  This function supports output pin configurations that require two 
  32-bit parameters.

  @param pin    The output pin to check.
  @param cfg    The pin configuration value will be returned here.
  @param param1 The pin's first 32-bit parameter will be returned here
  @param param2 The pin's second 32-bit parameter will be returned here
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetOutputConfig( int8 pin, OUTPUT_PIN_CONFIG &cfg, uint32 &param1, uint32 &param2 )
{
   if( pin < 0 ) return &SDO_Error::Subindex;

   int32 size = 10;
   byte buff[10];

   const Error *err = sdo.Upload( OBJID_OUTPUT_CFG, pin+1, size, buff );

   cfg = (OUTPUT_PIN_CONFIG)bytes_to_uint16(buff);
   param1 = bytes_to_uint32(&buff[2]);

   if( size > 6 )
      param2 = bytes_to_uint32(&buff[6]);
   else
      param2 = 0;

   return err;
}

/***************************************************************************/
/**
  Get the output pin configuration for the specified pin.

  @param pin The output pin to check.
  @param cfg The pin configuration value will be returned here.
  @param mask The pin's status bit selection mask will be returned here.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetOutputConfig( int8 pin, OUTPUT_PIN_CONFIG &cfg, uint32 &mask )
{
   uint32 mask2;
   return GetOutputConfig( pin, cfg, mask, mask2 );
}

/***************************************************************************/
/**
  Get the output configuration for the specified pin.

  @param pin The output pin to check.
  @param cfg The pin configuration value will be returned here.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetOutputConfig( int8 pin, OUTPUT_PIN_CONFIG &cfg )
{
   uint32 mask1, mask2;
   return GetOutputConfig( pin, cfg, mask1, mask2 );
}

/***************************************************************************/
/**
  Update the state of the manual output pins.  The passed value will
  be written to the output pin control register.  Any of the output pins that 
  have been configured as manual outputs will be updated based on the
  value of this register.

  Bit 0 controls output pin 0, bit 1 sets output pin 1, etc.

  Note that only those output pins that have been configured as manual 
  outputs are effected by this command.  Output pins that are configured to 
  perform some other function (such as tracking bits in the event status register)
  are not effected.  See Amp::SetOutputConfig for details on configuring the 
  amplifier output pins.

  Also note that this pin controls the active/inactive state of the outputs, not
  the high/low state.  Each output pin can be individually configured as active
  high or active low.  Setting a bit in the register to 1 sets the corresponding
  output pin active.  Active high/low configuration is set using Amp::SetOutputConfig.

  @param value The new value to write to the output pin control register.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetOutputs( uint16 value )
{
   return sdo.Dnld16( OBJID_OUTPUTS, 0, value ); 
}

/***************************************************************************/
/**
  Get the present value of the output pin control register.  

  This register shows the current state of all digital output pins.  For each 
  pin, the corresponding bit in the register will be 1 if the pin is active, and
  0 if the pin is inactive.  Bit 0 follows output pin 0, bit 1 follows pin 1, etc.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetOutputs( uint16 &value )
{
   return sdo.Upld16( OBJID_OUTPUTS, 0, value ); 
}

/***************************************************************************/
/**
  Get any active amplifier faults.

  @param value A bit mask identifying the active faults will be returned here.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetFaults( AMP_FAULT &value )
{
   uint32 v;
   const Error *err = sdo.Upld32( OBJID_FAULTS, 0, v );
   value = (AMP_FAULT)v;
   return err;
}

/***************************************************************************/
/**
  Clear amplifier faults.  This function can be used to clear any latching
  faults on the amplifier (see Amp::SetFaultMask for details on latching 
  fault conditions).  It also clears tracking error conditions.

  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::ClearFaults( AMP_FAULT value )
{
   const Error *err = sdo.Dnld32( OBJID_FAULTS, 0, (uint32)value );
   if( err ) return err;

   // I expect the amplifier status to be updated when the faults are
   // cleared.  I'll wait for that update here to make sure my local
   // status is consistant with the drives.
   EventNone e( AMPEVENT_FAULT );
   WaitEvent( e, 100 );
   return 0;
}

/***************************************************************************/
/**
  Set the amplifier's fault mask.  The fault mask identifies which conditions
  will be treated as latching faults by the amplifier.  If such a condition
  occurs, the amplifier's output will be disabled immediately, and will not 
  be enabled until the fault condition is cleared.

  @param value A bit mask identifying which fault conditions to latch.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetFaultMask( AMP_FAULT &value )
{
   return sdo.Dnld32( OBJID_FAULT_MASK, 0, (uint32)value );
}

/***************************************************************************/
/**
  Get the current value of the amplifier's fault mask.  This mask identifies
  which error conditions will be treated as latching faults by the amplifier.

  @param value The fault mask will be returned here
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetFaultMask( AMP_FAULT &value )
{
   uint32 v;
   const Error *err = sdo.Upld32( OBJID_FAULT_MASK, 0, v );
   value = (AMP_FAULT)v;
   return err;
}

/***************************************************************************/
/**
  Get the amplifier's 'event status' register.  This is the main register 
  used internal to the amplifier to describe it's current state.

  @param stat The register status is returned here.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetEventStatus( EVENT_STATUS &stat )
{
   uint32 s;
   const Error *err = sdo.Upld32( OBJID_EVENT_STAT, 0, s );
   stat = (EVENT_STATUS)s;
   return err;
}

/***************************************************************************/
/**
  Get the amplifier's 'sticky' event status register.  
  This register is a copy of the amplifiers event status register in which 
  bits are set normally, but only cleared when the register is read (i.e. 
  the bits are 'sticky').  It's useful for checking for transitory events
  which might me missed by reading the standard event status register.

  @param stat The register status is returned here.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetEventSticky( EVENT_STATUS &stat )
{
   uint32 s;
   const Error *err = sdo.Upld32( OBJID_EVENT_STAT_STICKY, 0, s );
   stat = (EVENT_STATUS)s;
   return err;
}

/***************************************************************************/
/**
  Get the amplifier's latched event status register.
  This register is a copy of the normal event status register in which bits 
  are latched, i.e. they are set bit not cleared.  Bits in this register 
  are only cleared in response to a Amp::ClearEventLatch function call (which
  is protected by the Amp object).  This register is primarily used internally
  by the Amp object to detect reset conditions on the amplifier.

  @param stat The register status is returned here.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetEventLatch( EVENT_STATUS &stat )
{
   uint32 s;
   const Error *err = sdo.Upld32( OBJID_EVENT_STAT_LATCH, 0, s );
   stat = (EVENT_STATUS)s;
   return err;
}

/***************************************************************************/
/**
  Clear the latched version of the amplifier's event status register.  This
  function is protected and generally only intended for internal use.

  @param stat Identifies which bits to clear.  Any bit set in this parameter
  will be cleared in the latched event status register.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::ClearEventLatch( EVENT_STATUS stat )
{
   return sdo.Dnld32(  OBJID_EVENT_STAT_LATCH, 0, (uint32)stat );
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
   HOMING MODE
   */
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Set the method used for homing the drive.

  @param method The home method to set
  @param extended If the 'method' parameter is set to CHM_EXTENDED, then 
         this value will be written to the extended homing parameter on the
	 amplifier.  For any other homing method this parameter is ignored.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetHomeMethod( COPLEY_HOME_METHOD method, uint16 extended )
{
   const Error *err;

   if( method == CHM_EXTENDED )
      err = sdo.Dnld16( OBJID_HOME_METHOD_EXT, 0, extended );
   else
      err = sdo.Dnld8( OBJID_HOME_METHOD, 0, (int8)method );

   if( !err ) lastHomeMethod = method;
   return err;
}

/***************************************************************************/
/**
  Get the selected homing method.
  @param method The home method will be returned here.
  @param extended If this pointer is non-null, then the extended homing 
           method value will be returned here.  If this pointer is null,
	   then it will be ignored.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetHomeMethod( COPLEY_HOME_METHOD &method, uint16 *extended )
{
   int8 i;
   const Error *err = sdo.Upld8( OBJID_HOME_METHOD, 0, i );
   method = (COPLEY_HOME_METHOD)i;
   if( err || !extended ) return err;

   return sdo.Upld16( OBJID_HOME_METHOD_EXT, 0, *extended );
}

/***************************************************************************/
/**
  Set the home offset value.  This offset is the difference between the location
  of the homing sensor (as defined by the homing method), and the actual zero
  position.  Once the home location has been found, the amplifier will use this 
  offset to determine where the zero position location is.  

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value the value to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetHomeOffset( uunit value )
{
   return sdo.Dnld32( OBJID_HOME_OFFSET, 0, PosUser2Load(value) );
}

/***************************************************************************/
/**
  Get the home offset value.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetHomeOffset( uunit &value )
{ 
   int32 v;
   const Error *err = sdo.Upld32( OBJID_HOME_OFFSET, 0, v ); 
   value = PosLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Set the home velocity used to move to a home switch.  This velocity will be used
  for any move in the home routine that can be done at relatively high speed.  A 
  second slower velocity can also be programed for the parts of the home routine
  that are speed sensitive.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value the value to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetHomeVelFast( uunit value )
{
   int32 v = VelUser2Load(value);
   if( v < 0 ) return &AmpError::badHomeParam;

   return sdo.Dnld32( OBJID_HOME_VEL, 1, v ); 
}

/***************************************************************************/
/**
  Get the home velocity used to move to a home switch.  This velocity is used
  for any home moves that may be made at a high velocity without effecting the
  quality of the home sensor detection.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetHomeVelFast( uunit &value )
{
   int32 v;
   const Error *err = sdo.Upld32( OBJID_HOME_VEL, 1, v );
   value = VelLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Set the home velocity used to find a switch edge.  This velocity will be used 
  for any move in the home routine which is speed sensitive.  This typically is a 
  move in which the edge of a sensor is being searched for.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value the value to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetHomeVelSlow( uunit value )
{
   int32 v = VelUser2Load(value);
   if( v < 0 ) return &AmpError::badHomeParam;

   return sdo.Dnld32( OBJID_HOME_VEL, 2, v ); 
}

/***************************************************************************/
/**
  Get the home velocity used to find a switch edge.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetHomeVelSlow( uunit &value )
{
   int32 v;
   const Error *err = sdo.Upld32( OBJID_HOME_VEL, 2, v );
   value = VelLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Set the home acceleration.  This acceleration value will be used for all 
  moves that are part of the home routine.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value the value to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetHomeAccel( uunit value  )
{ 
   int32 v = AccUser2Load(value);
   if( v < 0 ) return &AmpError::badHomeParam;

   return sdo.Dnld32( OBJID_HOME_ACC, 0, v ); 
}

/***************************************************************************/
/**
  Get the home acceleration.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetHomeAccel( uunit &value )
{
   int32 v;
   const Error *err = sdo.Upld32( OBJID_HOME_ACC, 0, v );
   value = AccLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Get the last home adjustment amount.

  The value returned is distance that the home position was adjusted by on the last
  successful home opperation.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetHomeAdjustment( uunit &value )
{
   int32 v;
   const Error *err = sdo.Upld32( OBJID_HOME_ADJUST, 0, v );
   value = PosLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Set the home current.  The home current value is only used when homing to a
  hard stop.  This parameter is specified in units of 0.01 Amps (i.e. a value
  of 123 would be 1.23 Amps).

  @param value the value to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetHomeCurrent( int16 value  )
{ 
   return sdo.Dnld16( OBJID_HOME_CRNT, 0, value ); 
}

/***************************************************************************/
/**
  Get the home current.  The homing current is returned in 0.01 Amp units
  (i.e. a value of 123 would be 1.23 Amps).

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetHomeCurrent( int16 &value )
{
   return sdo.Upld16( OBJID_HOME_CRNT, 0, value );
}

/***************************************************************************/
/**
  Set the home delay.  The home delay value is only used when homing to a
  hard stop.  This parameter is specified in units of milliseconds.

  @param value the value to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetHomeDelay( int16 value  )
{ 
   return sdo.Dnld16( OBJID_HOME_DELAY, 0, value ); 
}

/***************************************************************************/
/**
  Get the home current.  The homing delay is returned in units of milliseconds.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetHomeDelay( int16 &value )
{
   return sdo.Upld16( OBJID_HOME_DELAY, 0, value );
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
                         Position capture control
*/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/**
  Set the position capture configuration.

  The position capture mechanism in the amplifier allows the motor 
  position to be captured by some event.  The position can be captured
  by a transition on the encoder index signal, or by a transition on 
  a general purpose input pin which has been configured as a 'home' 
  input.

  Note that support for capturing position on a home input pin was
  added in version 4.77 firmware.  Earlier versions of firmware will
  return an error if an attempt is made to configure that capture.
 */
const Error *Amp::SetPosCaptureCfg( POS_CAPTURE_CFG cfg )
{
   return sdo.Dnld16( OBJID_CAP_CTRL, 0, (uint16)cfg );
}

/**
  Read the current configuration of the position capture mechanism.
  */
const Error *Amp::GetPosCaptureCfg( POS_CAPTURE_CFG &cfg )
{
   uint16 value;
   const Error *err = sdo.Upld16( OBJID_CAP_CTRL, 0, value );
   cfg = (POS_CAPTURE_CFG)value;
   return err;
}

/**
  Read the current status of the position capture mechanism.
  */
const Error *Amp::GetPosCaptureStat( POS_CAPTURE_STAT &stat )
{
   uint16 value;
   const Error *err = sdo.Upld16( OBJID_CAP_STAT, 0, value );
   stat = (POS_CAPTURE_STAT)value;
   return err;
}

/**
  Get the most recently captured encoder index position.
  */
const Error *Amp::GetIndexCapture( int32 &value )
{
   return sdo.Upld32( OBJID_CAP_NDX, 0, value );
}

/**
  Get the most recently captured home sensor position.

  Note that support for capturing position on a home input pin was
  added in version 4.77 firmware.  Earlier versions of firmware will
  return an error if an attempt is made to read this register.
  */
const Error *Amp::GetHomeCapture( int32 &value )
{
   return sdo.Upld32( OBJID_CAP_HOME, 0, value );
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
   Position profile mode (point to point moves)
   */
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Set the motion profile type.

  The motion profile type is only used when running in 'position profile' mode.
  In this mode, the drive performs point to point moves using it's internal 
  trajectory generator.

  The motion profile type defines the type of trajectory profile that the 
  drive will generate.

  @param type The profile type to use
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetProfileType( PROFILE_TYPE type )
{
   return sdo.Dnld16( OBJID_PROFILE_TYPE, 0, (int16)type );
}

/***************************************************************************/
/**
  Get the currently selected motion profile type.  This profile type is used
  for point to point moves in which the amplifier calculates it's own 
  trajectory.

  @param type variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetProfileType( PROFILE_TYPE &type )
{
   int16 value;
   const Error *err = sdo.Upld16( OBJID_PROFILE_TYPE, 0, value );
   type = (PROFILE_TYPE)value;
   return err;
}

/***************************************************************************/
/**
  Set the profile target position (i.e. the position to which the motor should move).

  For relative moves, this function sets the distance to move.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value The position to move to
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetTargetPos( uunit value )
{
   return sdo.Dnld32( OBJID_PROFILE_POS, 0, PosUser2Load(value) );
}

/***************************************************************************/
/**
  Get the profile target position.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetTargetPos( uunit &value )
{
   int32 v;
   const Error *err = sdo.Upld32( OBJID_PROFILE_POS, 0, v ); 
   value = PosLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Set the target velocity used in profile velocity mode.  

  This parameter is only used when the amplifier is set to profile velocity 
  mode (AMPMODE_CAN_VELOCITY).  When in this mode, this parameter defines the
  target velocity for motion.  

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value The new target velocity.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetTargetVel( uunit value )
{
   return sdo.Dnld32( OBJID_TARGET_VEL, 0, VelUser2Load(value) );
}

/***************************************************************************/
/**
  Get the target velocity used in profile velocity mode.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetTargetVel( uunit &value )
{
   int32 v;
   const Error *err = sdo.Upld32( OBJID_TARGET_VEL, 0, v ); 
   value = VelLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Set the profile velocity value (i.e. the velocity that the motor will normally
  attain during the move).

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value The value to set.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetProfileVel( uunit value )
{ 
   int32 v = VelUser2Load(value);
   if( v < 0 ) return &AmpError::badMoveParam;

   return sdo.Dnld32( OBJID_PROFILE_VEL, 0, v );
}

/***************************************************************************/
/**
  Get the profile velocity value

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetProfileVel( uunit &value )
{ 
   int32 v;
   const Error *err = sdo.Upld32( OBJID_PROFILE_VEL, 0, v );
   value = VelLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Set the profile acceleration value (i.e. the acceleration that the motor will normally
  attain when starting the move).

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value The value to set.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetProfileAcc( uunit value )
{ 
   int32 v = AccUser2Load(value);
   if( v < 0 ) return &AmpError::badMoveParam;

   return sdo.Dnld32( OBJID_PROFILE_ACC, 0, v ); 
}

/***************************************************************************/
/**
  Get the profile acceleration value

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetProfileAcc( uunit &value )
{ 
   int32 v;
   const Error *err = sdo.Upld32( OBJID_PROFILE_ACC, 0, v ); 
   value = AccLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Set the profile deceleration value (i.e. the acceleration that the motor will normally
  attain when ending the move).

  Note that S-curve profiles don't use a separate deceleration value.  For S-curve moves,
  the value programmed in SetProfileAcc is also used for the deceleration segment at the
  end of the move.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value The value to set.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetProfileDec( uunit value )
{ 
   int32 v = AccUser2Load(value);
   if( v < 0 ) return &AmpError::badMoveParam;

   return sdo.Dnld32( OBJID_PROFILE_DEC, 0, v ); 
}

/***************************************************************************/
/**
  Get the profile deceleration value.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetProfileDec( uunit &value )
{ 
   int32 v;
   const Error *err = sdo.Upld32( OBJID_PROFILE_DEC, 0, v ); 
   value = AccLoad2User( v );
   return err;
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
   Profile torque mode
   */
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Set the amplifier target torque value.  This parameter is used in profile 
  torque mode (AMPMODE_CAN_TORQUE) to specify the desired target torque value.
  The actual torque commanded by the amplifier will ramp up/down to this value
  based on the programmed torque slope (see Amp::SetTorqueSlope).  

  @param value The torque value to be set.  This is specified in thousandths
  of the motor rated torque (see Amp::GetTorqueRated).
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetTorqueTarget( int16 value )
{
   return sdo.Dnld16( OBJID_TORQUE_TARGET, 0, value ); 
}

/***************************************************************************/
/**
  Get the current target torque value.

  This parameter is used in profile torque mode (AMPMODE_CAN_TORQUE) to specify 
  the torque that should be applied by the motor.

  @param value The torque value is returned here.  This is specified in thousandths
  of the motor rated torque (see Amp::GetTorqueRated).
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetTorqueTarget( int16 &value )
{
   return sdo.Upld16( OBJID_TORQUE_TARGET, 0, value ); 
}

/***************************************************************************/
/**
  Get the torque demand value.  This is the torque that the amplifier is attempting
  to apply at the moment.

  @param value The torque value is returned here.  This is specified in thousandths
  of the motor rated torque (see Amp::GetTorqueRated).
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetTorqueDemand( int16 &value )
{
   return sdo.Upld16( OBJID_TORQUE_DEMAND, 0, value ); 
}

/***************************************************************************/
/**
  Get the actual torque being applied by the motor at the moment.

  @param value The torque value is returned here.  This is specified in thousandths
  of the motor rated torque (see Amp::GetTorqueRated).
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetTorqueActual( int16 &value )
{
   return sdo.Upld16( OBJID_TORQUE_ACTUAL, 0, value ); 
}

/***************************************************************************/
/**
  Set the rate of change of torque for use in profile torque mode (AMPMODE_CAN_TORQUE).
  Setting this parameter to zero will cause the rate of change to be unlimited.

  @param value The rate of change specified in thousandths of the total rated torque
  per second.  For example, setting to 1000 would specify a slope of the full
  rated torque of the motor every second.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetTorqueSlope( int32 value )
{
   return sdo.Dnld32( OBJID_TORQUE_SLOPE, 0, value ); 
}

/***************************************************************************/
/**
  Get the rate of change of torque for use in profile torque mode (AMPMODE_CAN_TORQUE).

  @param value The rate of change specified in thousandths of the total rated torque
  per second.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetTorqueSlope( int32 &value )
{
   return sdo.Upld32( OBJID_TORQUE_SLOPE, 0, value ); 
}

/***************************************************************************/
/**
  Set the motor rated torque parameter.  The motor's rated torque is the amount
  of torque that the motor can continuously output without damage.  

  @param value The motor's rated torque in 0.001 Nm units.  For linear motors 
  the units are 0.001 N.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetTorqueRated( int32 value )
{
   return sdo.Dnld32( OBJID_TORQUE_RATED, 0, value ); 
}

/***************************************************************************/
/**
  Get the motor rated torque parameter.  The motor's rated torque is the amount
  of torque that the motor can continuously output without damage.  

  @param value The motor's rated torque in 0.001 Nm units.  For linear motors 
  the units are 0.001 N.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetTorqueRated( int32 &value )
{
   return sdo.Upld32( OBJID_TORQUE_RATED, 0, value ); 
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
   Quick stop, halt, etc
   */
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Set the halt mode.  When the amplifier's halt command is issued (Amp::HaltMove)
  the amplifier will attempt to stop the move in progress using the method 
  defined by it's halt mode.

  @param mode The mode to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetHaltMode( HALT_MODE mode )
{
   return sdo.Dnld16( OBJID_HALT_MODE, 0, (int16)mode );
}

/***************************************************************************/
/**
  Get the halt mode.  This mode defines what happens when a halt command is
  issued to the amplifier.

  @param mode The mode will be returned here.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetHaltMode( HALT_MODE &mode )
{
   int16 m;
   const Error *err = sdo.Upld16( OBJID_HALT_MODE, 0, m );
   mode = (HALT_MODE)m;
   return err;
}

/***************************************************************************/
/**
  Set the quick stop mode.  When the amplifier's quick stop command is issued 
  (Amp::QuickStop), the amplifier will attempt to stop the move in progress 
  using the method defined by it's quick stop mode.

  @param mode The mode to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetQuickStop( QUICK_STOP_MODE mode )
{
   return sdo.Dnld16( OBJID_QSTOP_MODE, 0, (int16)mode );
}

/***************************************************************************/
/**
  Get the quick stop mode.  This mode defines what happens when a quick stop 
  command is issued to the amplifier.

  @param mode The mode will be returned here.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetQuickStop( QUICK_STOP_MODE &mode )
{
   int16 m;
   const Error *err = sdo.Upld16( OBJID_QSTOP_MODE, 0, m );
   mode = (QUICK_STOP_MODE)m;
   return err;
}

/***************************************************************************/
/**
  Set the quick stop deceleration value (i.e. the acceleration that the motor will 
  use when doing a quick stop).

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value The value to set.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetQuickStopDec( uunit value )
{ 
   return sdo.Dnld32( OBJID_PROFILE_QSTOP, 0, AccUser2Load(value) ); 
}

/***************************************************************************/
/**
  Get the quick stop deceleration value.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetQuickStopDec( uunit &value )
{ 
   int32 v;
   const Error *err = sdo.Upld32( OBJID_PROFILE_QSTOP, 0, v ); 
   value = AccLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Set the jerk limit used with S-curve profiles.  Jerk is the rate of change 
  of acceleration.

  Note that this value is only used with S-curve profiles.  Trapezoidal profiles
  do not limit jerk (i.e. they allow instantaneous changes in acceleration).

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value The jerk value to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetProfileJerk( uunit value )
{ 
   int32 v = JrkUser2Load(value);
   if( v < 0 ) return &AmpError::badMoveParam;

   return sdo.Dnld32( OBJID_PROFILE_JRK, 0, v ); 
}

/***************************************************************************/
/**
  Get the currently programmed jerk limit for S-curve profiles.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value The location in which the jerk value will be returned.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetProfileJerk( uunit &value  )
{ 
   int32 v;
   const Error *err = sdo.Upld32( OBJID_PROFILE_JRK, 0, v ); 
   value = JrkLoad2User( v );
   return err;
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
   Interpolated position mode (PVT moves)
   */
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/


/***************************************************************************/
/**
  Get the number of free positions in the PVT segment buffer.
  @param n A reference used to return the number of free positions
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetPvtBuffFree( int16 &n )
{
   return sdo.Upld16( OBJID_PVT_BUFF_CT, 0, n );
}

/***************************************************************************/
/**
  Get the segment ID that the amplifier expects for the next PVT
  segment.  This starts at zero when the amp is reset, and is 
  increased for every segment received.
  @param id A reference to the variable where the ID will be written
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetPvtSegID( uint16 &id )
{
   return sdo.Upld16( OBJID_PVT_SEG_ID, 0, id ); 
}

/***************************************************************************/
/**
  Get the amplifier's PVT buffer status word.
  @param stat A reference to a variable where the status will be returned
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetPvtBuffStat( uint32 &stat )
{
   return sdo.Upld32( OBJID_PVT_BUFF_STAT, 0, stat ); 
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
   Position windows (error, warning, settling, etc)
   */
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Set the position error window.  If the absolute value of the motor's position
  error ever exceeds this value, then a tracking error will occur.

  A tracking error causes the amplifier to abort any move in progress, and 
  attempt to bring the motor to a stop using it's velocity loop.  The commanded
  velocity input to the velocity loop will be driven to zero, subject to the
  velocity loop acceleration and deceleration limits.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value The value to use for the position error window.  
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetPositionErrorWindow( uunit value )
{ 
   return sdo.Dnld32( OBJID_POSERR_WIN, 0, PosUser2Load(value) ); 
}

/***************************************************************************/
/**
  Get the position error window.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value The position error window value will be returned here.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetPositionErrorWindow( uunit &value )
{ 
   int32 v;
   const Error *err = sdo.Upld32( OBJID_POSERR_WIN, 0, v ); 
   value = PosLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Set the position warning window.  

  If the absolute value of the position error ever exceeds this value, 
  then a tracking warning will result.  A tracking warning causes a bit
  in the drives status to be set.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value the value to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetPositionWarnWindow( uunit value )
{ 
   return sdo.Dnld32( OBJID_POSWARN_WIN, 0, PosUser2Load(value) ); 
}

/***************************************************************************/
/**
  Get the position warning window.  

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetPositionWarnWindow( uunit &value )
{ 
   int32 v;
   const Error *err = sdo.Upld32( OBJID_POSWARN_WIN, 0, v ); 
   value = PosLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Set the position settling window.  

  The drive will be considered to be settled in position
  after a move when it's absolute position error value has been within the position
  window for an amount of time greater then the position window time value.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value the value to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetSettlingWindow( uunit value )
{
   return sdo.Dnld32( OBJID_SETTLE_WIN, 0, PosUser2Load(value) );
}

/***************************************************************************/
/**
  Get the position window value.  This window, along with the position window time value, 
  is used to identify when the motor has come to rest at the desired position.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetSettlingWindow( uunit &value )
{ 
   int32 v;
   const Error *err = sdo.Upld32( OBJID_SETTLE_WIN, 0, v ); 
   value = PosLoad2User( v );
   return err;
}

/***************************************************************************/
/**
  Set the position window time value (milliseconds). 
  The drive will be considered to be settled in position after a move when it's 
  absolute position error value has been within the position window for an amount 
  of time greater then the position window time value.

  @param value the value to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetSettlingTime( uint16 value )
{ 
   return sdo.Dnld16( OBJID_SETTLE_TIME, 0, value ); 
}

/***************************************************************************/
/**
  Get the position window time value (milliseconds).  This timeout is used in conjunction 
  with the position window  value to identify when a motor has come to rest at the desired
  position.
  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetSettlingTime( uint16 &value )
{ 
   return sdo.Upld16( OBJID_SETTLE_TIME, 0, value ); 
}

/***************************************************************************/
/**
  Set the velocity warning window.  

  If the absolute value of the velocity error exceeds this value, 
  then a velocity warning will result.  A velocity warning causes a bit
  in the drives status to be set.

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value the value to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetVelocityWarnWindow( uunit value )
{
   return sdo.Dnld32( OBJID_VELWARN_WIN, 0, VelUser2Mtr(value) ); 
}

/***************************************************************************/
/**
  Get the velocity warning window.  

  This parameter is specified in "user units".  See Amp::SetCountsPerUnit for details.

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetVelocityWarnWindow( uunit &value )
{ 
   int32 v;
   const Error *err = sdo.Upld32( OBJID_VELWARN_WIN, 0, v ); 
   value = VelMtr2User( v );
   return err;
}

/***************************************************************************/
/**
  Set the velocity warning window time value (milliseconds). 

  If the velocity error exceeds the velocity warning window, then a bit 
  will be set in the amplifier status word.  This bit will not be cleared
  until the velocity error has been within the warning window for at 
  least this long.

  @param value the value to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetVelocityWarnTime( uint16 value )
{ 
   return sdo.Dnld16( OBJID_VELWARN_TIME, 0, value ); 
}

/***************************************************************************/
/**
  Get the position window time value (milliseconds).

  @param value variable that will store the returned value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetVelocityWarnTime( uint16 &value )
{ 
   return sdo.Upld16( OBJID_VELWARN_TIME, 0, value ); 
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
   CANopen device profile 
   */
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Get the current value of the drive's status word.  The drive status word 
  indicates the drives current state in it's internal state machine.  The
  drives state in turn identifies if the drive is enabled / disabled, whether
  a fault is present on the drive, whether it is in motion, etc.

  This status word is part of the CANopen device profile (DSP-402).  It's 
  used internally by the amplifier object.

  @param value Returns the current status word value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetStatusWord( uint16 &value )
{
   return sdo.Upld16( OBJID_STATUS, 0, value );
}

/***************************************************************************/
/**
  Set the amplifier's control word.

  The control word is part of the CANopen device profile (DSP-402).  It's used
  to enable/disable the amplifier, start moves, etc.  This function is used
  internally by the Amp object.

  @param value The control word value to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetControlWord( uint16 value )
{
   cml.Debug( "Amp %d control 0x%04x\n", GetNodeID(), value );
   const Error *err = sdo.Dnld16( OBJID_CONTROL, 0, value );
   if( !err ) lastCtrlWord = value;
   else
      cml.Warn( "Amp %d, Error setting control 0x%04x: %s\n", GetNodeID(), value, err->toString() );

   return err;
}

/***************************************************************************/
/**
  Returns the present value of the CANopen device profile control word.

  @param value Returns the control word value
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetControlWord( uint16 &value )
{
   return sdo.Upld16( OBJID_CONTROL, 0, value );
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
   Low level control modes 
   */
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Set the programmed current value in 0.01 Amp units.  This parameter is only 
  used when running in the mode AMPMODE_PROG_CRNT.  The value programmed through
  this variable is the current that the amplifier will attempt to output.

  @param crnt The current to output (0.01 Amp units).
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetCurrentProgrammed( int16 crnt )
{
   return sdo.Dnld16( OBJID_PROG_CRNT, 0, crnt );
}

/***************************************************************************/
/**
  Get the programmed current value.  This parameter is the current that the 
  amplifier will attempt to maintain when set to the mode AMPMODE_PROG_CRNT.
  @param crnt The current will be returned here (0.01 Amp units).
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetCurrentProgrammed( int16 &crnt )
{
   return sdo.Upld16( OBJID_PROG_CRNT, 0, crnt );
}

/***************************************************************************/
/**
  Set the programmed velocity value.  This parameter is only used when running 
  in the mode AMPMODE_PROG_VEL.  The value programmed through this variable is 
  the velocity that the amplifier will attempt to output.

  @param vel The velocity to output.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetVelocityProgrammed( uunit vel )
{
   return sdo.Dnld32( OBJID_PROG_VEL, 0, VelUser2Mtr(vel) ); 
}

/***************************************************************************/
/**
  Get the programmed velocity value.  This parameter is the velocity that the 
  amplifier will attempt to maintain when set to the mode AMPMODE_PROG_VEL.
  @param vel The velocity will be returned here.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetVelocityProgrammed( uunit &vel )
{
   int32 v;
   const Error *err = sdo.Upld32( OBJID_PROG_VEL, 0, v ); 
   vel = VelMtr2User( v );
   return err;
}

/***************************************************************************/
/**
  Set the amplifier microstepping rate.  This parameter is only used in the 
  diagnostic microstepping mode (AMPMODE_DIAG_USTEP).
  @param rate The microstepping rate in degrees / second
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetMicrostepRate( int16 rate )
{
   return sdo.Dnld16( OBJID_USTEP_RATE, 0, rate );
}

/***************************************************************************/
/**
  Get the amplifier microstepping rate.  This parameter is only used in the 
  diagnostic microstepping mode (AMPMODE_DIAG_USTEP).
  @param rate The microstepping rate will be returned here (degrees / second).
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetMicrostepRate( int16 &rate )
{
   return sdo.Upld16( OBJID_USTEP_RATE, 0, rate );
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
   Trace related
   */
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/


/***************************************************************************/
/**
  Get the current status of the amplifier's trace system.
  @param stat Information on whether the trace is currently running is returned
  in this parameter.
  @param samp The total number of trace samples collected is returned here.
  @param sampMax The maximum number of trace samples that will fit in the internal
  buffer is returned here.  This value will change depending on how
  many trace channels are active and which variables are selected.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetTraceStatus( AMP_TRACE_STATUS &stat, int16 &samp, int16 &sampMax )
{
   const Error *err;
   uint16 s;

   err = sdo.Upld16( OBJID_TRACE_STATUS, 0, s );
   if( !err ) err = sdo.Upld16( OBJID_TRACE_SAMP_CT,  0, samp );
   if( !err ) err = sdo.Upld16( OBJID_TRACE_SAMP_MAX, 0, sampMax );

   stat = (AMP_TRACE_STATUS)s;
   return err;
}

/***************************************************************************/
/**
  Get the 'reference period' used with the amplifiers trace mechanism. 
  The amplifier internally samples it's trace channels at integer multiples
  of this time. 

  For example, if the amplifier's reference period is 100,000 nanoseconds, then 
  setting the trace period to 12 would indicate that the amplifier should 
  sample it's internal variables every 1.2 milliseconds.

  @param per The reference period is returned here in units of nanoseconds.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetTraceRefPeriod( int32 &per )
{
   return sdo.Upld32( OBJID_TRACE_REF_PER, 0, per );
}

/***************************************************************************/
/**
  Get the period of time between trace samples.  When the trace system is running,
  the amplifier will sample and store it's internal variables this often.

  Note that this parameter specifies time in units of the amplifier's 'reference
  period'.  See Amp::GetTraceRefPeriod for more information.

  @param per The trace period is returned here.

  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetTracePeriod( int16 &per )
{
   return sdo.Upld16( OBJID_TRACE_PERIOD, 0, per );
}

/***************************************************************************/
/**
  Set the period of time between trace samples.  When the trace system is running,
  the amplifier will sample and store it's internal variables this often.

  Note that this parameter specifies time in units of the amplifier's 'reference
  period'.  See Amp::GetTraceRefPeriod for more information.

  @param per The trace period to be set.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetTracePeriod( int16 per )
{
   return sdo.Dnld16( OBJID_TRACE_PERIOD, 0, per );
}

/***************************************************************************/
/**
  Get the current configuration of the amplifier's trace trigger.

  See Amp::SetTraceTrigger for more information about the trigger.

  @param type The type of trigger to be used.
  @param chan Which trace channel to trigger off of.
  @param level The trigger level
  @param delay The delay between the occurance of the trigger and the
  start of data collection.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetTraceTrigger( AMP_TRACE_TRIGGER &type, uint8 &chan, int32 &level, int16 &delay )
{
   int32 size = 6;
   byte buff[6];

   const Error *err = sdo.Upload( OBJID_TRACE_TRIGGER, 0, size, buff );
   if( !err ) err = sdo.Upld16( OBJID_TRACE_DELAY, 0, delay );

   if( err ) return err;

   uint16 t = bytes_to_uint16(buff);
   level = bytes_to_int32(&buff[2]);

   chan = t & TRACETRIG_CHANNEL;
   type = (AMP_TRACE_TRIGGER)(t & ~TRACETRIG_CHANNEL);

   return 0;
}

/***************************************************************************/
/**
  Configure the amplifier's trace trigger.

  The trigger acts something like the trigger on an oscilloscope.  It allows some
  event to be specified which will cause the trace subsystem to start collecting 
  data.  Most trigger types watch one of the trace channels and constantly compare
  it's value to a level.  The type of comparison made will depend on the type of
  trigger.  For example, the trace can be triggered on the rising edge of a signal, 
  on the falling edge, etc.

  The trigger also allows a delay value to be specified.  Trace data will start to
  be collected N trace periods after the trigger, where N is the delay value.  The
  delay can also be negative, in which case the data will start to be collected 
  before the trigger event.

  @param type  The trigger type.
  @param chan  The trace channel to watch.  This parameter defaults to 0 if not specified.
  @param level The trigger level.  This parameter defaults to 0 if not specified.
  @param delay The trigger delay in trace sample periods.  Defaults to 0 if not specified.

  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetTraceTrigger( AMP_TRACE_TRIGGER type, uint8 chan, int32 level, int16 delay )
{
   uint16 t = (uint16)type;
   t &= ~TRACETRIG_CHANNEL;
   t |= (chan & TRACETRIG_CHANNEL);

   byte buff[6];

   buff[0] = ByteCast(t);
   buff[1] = ByteCast(t>>8);
   buff[2] = ByteCast(level);
   buff[3] = ByteCast(level>>8);
   buff[4] = ByteCast(level>>16);
   buff[5] = ByteCast(level>>24);

   const Error *err = sdo.Download( OBJID_TRACE_TRIGGER, 0, 6, buff );
   if( !err ) err = sdo.Dnld16( OBJID_TRACE_DELAY, 0, delay );
   return err;
}

/***************************************************************************/
/**
  Return the maximum number of trace channels supported by the amplifier.
  @param max The number of channels is returned here.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetTraceMaxChannel( uint8 &max )
{
   return sdo.Upld8( OBJID_TRACE_CHANNELS, 0, max );
}

/***************************************************************************/
/**
  Get the amplifier variable current selected on one of the trace channels.

  @param ndx The trace channel to get
  @param value The trace variable assigned to this channel will be returned here.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetTraceChannel( uint8 ndx, AMP_TRACE_VAR &value )
{
   if( ndx == 255 ) return &SDO_Error::Subindex;
   uint16 v;
   const Error *err = sdo.Upld16( OBJID_TRACE_CHANNELS, ndx+1, v );
   value = (AMP_TRACE_VAR)v;
   return err;
}

/***************************************************************************/
/**
  Select an amplifier trace variable to be sampled.

  @param ndx The trace channel that the variable will be assigned to.
  @param value The trace variable to sample.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetTraceChannel( uint8 ndx, AMP_TRACE_VAR value )
{
   if( ndx == 255 ) return &SDO_Error::Subindex;
   return sdo.Dnld16( OBJID_TRACE_CHANNELS, ndx+1, (uint16)value );
}

/***************************************************************************/
/**
  Start collecting trace data on the amplifier.  The trace will automatically
  stop once the amplifier's internal trace buffer fills up.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::TraceStart( void )
{
   return sdo.Dnld16( OBJID_TRACE_START, 0, (uint16)1 );
}

/***************************************************************************/
/**
  Stop collecting trace data on the amplifier.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::TraceStop( void )
{
   return sdo.Dnld16( OBJID_TRACE_START, 0, (uint16)0 );
}

/***************************************************************************/
/**
  Upload any trace data captured in the amplifier.  Trace data should only be
  uploaded when the traces are stopped.  Uploading data while it is currently 
  being collected in the amplifier can cause corrupt data to be uploaded.

  The trace data is returned as an array of 32-bit integer values.  If there are
  N currently active trace channels, and M samples of data have been collected,
  then a total of N x M integer values will be returned.  In this case, the samples 
  for channel n (0 <= n < N) will be located at postion n + m*N for 0 <= m < M.

  @param data An array where the trace data will be returned.
  @param max On entry to this call, this parameter must hold the maximum number of
  32-bit integer values to upload.  On successful return this parameter
  will be filled with the total number of integers uploaded.

  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetTraceData( int32 *data, int32 &max )
{
   byte *bptr = (byte *)data;

   int32 size = max * 4;

   const Error *err = sdo.Upload( OBJID_TRACE_DATA, 0, size, bptr );
   if( err ) return err;

   max = size/4;

   for( int i=0; i<max; i++ )
      data[i] = bytes_to_int32( &bptr[4*i] );

   return 0;
}

