/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

*/

#ifndef _DEF_INC_CML_AMPDEF
#define _DEF_INC_CML_AMPDEF

#include "CML_Settings.h"

CML_NAMESPACE_START()

/***************************************************************************/
/**
This enumeration holds the object identifiers of all of the objects in the
amplifier's object dictionary.
*/
/***************************************************************************/
enum AMP_OBJID
{
   OBJID_CONTROL               = 0x6040,
   OBJID_STATUS                = 0x6041,
   OBJID_AMP_MODE              = 0x2300,
   OBJID_PWM_MODE              = 0x2140,
   OBJID_OP_MODE               = 0x6060,
   OBJID_OP_MODE_DISP          = 0x6061,
   OBJID_EVENT_STAT            = 0x1002,
   OBJID_EVENT_STAT_STICKY     = 0x2180,
   OBJID_EVENT_STAT_LATCH      = 0x2181,
   OBJID_FAULT_MASK            = 0x2182,
   OBJID_FAULTS                = 0x2183,

   OBJID_POS_ACT               = 0x6064,
   OBJID_POS_LOAD              = 0x6064,
   OBJID_POS_CMD               = 0x6062,
   OBJID_POS_ERR               = 0x60F4,
   OBJID_POS_MTR               = 0x2240,

   OBJID_VEL_ACT               = 0x6069,
   OBJID_VEL_MTR               = 0x6069,
   OBJID_VEL_CMD               = 0x606B,
   OBJID_VEL_LIM               = 0x2230,
   OBJID_VEL_LOAD              = 0x2231,
   OBJID_TARGET_VEL            = 0x60FF,

   OBJID_CRNT_ACT              = 0x221C,
   OBJID_CRNT_CMD              = 0x221D,
   OBJID_CRNT_LIM              = 0x221E,

   OBJID_INPUTS                = 0x2190,
   OBJID_PULLUPS               = 0x2191,
   OBJID_INPUT_CFG             = 0x2192,
   OBJID_OUTPUT_CFG            = 0x2193,
   OBJID_OUTPUTS               = 0x2194,
   OBJID_DEBOUNCE              = 0x2195,

   OBJID_PHASE_ANG             = 0x2260,
   OBJID_HALL_STATE            = 0x2261,
   OBJID_PHASE_MODE            = 0x21C0,
   OBJID_USTEP_RATE            = 0x21C1,

   OBJID_ANALOG_REF            = 0x2200,
   OBJID_HVREF                 = 0x2201,
   OBJID_AMPTEMP               = 0x2202,
   OBJID_CNRT_U                = 0x2203,
   OBJID_CRNT_V                = 0x2204,
   OBJID_AENC_SIN              = 0x2205,
   OBJID_AENC_COS              = 0x2206,

   OBJID_HOME_METHOD           = 0x6098,
   OBJID_HOME_VEL              = 0x6099,
   OBJID_HOME_ACC              = 0x609A,
   OBJID_HOME_OFFSET           = 0x607C,
   OBJID_HOME_CRNT             = 0x2350,
   OBJID_HOME_DELAY            = 0x2351,
   OBJID_HOME_METHOD_EXT       = 0x2352,
   OBJID_HOME_ADJUST           = 0x2353,

   OBJID_PROFILE_TYPE          = 0x6086,
   OBJID_PROFILE_POS           = 0x607A,
   OBJID_PROFILE_VEL           = 0x6081,
   OBJID_PROFILE_ACC           = 0x6083,
   OBJID_PROFILE_DEC           = 0x6084,
   OBJID_PROFILE_JRK           = 0x2121,

   OBJID_PROFILE_QSTOP         = 0x6085,
   OBJID_QSTOP_MODE            = 0x605A,
   OBJID_HALT_MODE             = 0x605D,

   OBJID_PVT_DATA              = 0x2010,
   OBJID_PVT_BUFF_CT           = 0x2011,
   OBJID_PVT_BUFF_STAT         = 0x2012,
   OBJID_PVT_SEG_ID            = 0x2013,

   OBJID_TRJ_VEL               = 0x2250,
   OBJID_TRJ_ACC               = 0x2251,
   OBJID_TRJ_STATUS            = 0x2252,

   OBJID_POSERR_WIN            = 0x2120,
   OBJID_POSWARN_WIN           = 0x6065,
   OBJID_SETTLE_WIN            = 0x6067,
   OBJID_SETTLE_TIME           = 0x6068,

   OBJID_VELWARN_WIN           = 0x2104,
   OBJID_VELWARN_TIME          = 0x2105,
   OBJID_VEL_OUTFILT           = 0x2106,
   OBJID_HALLVEL_SHIFT         = 0x2107,
   OBJID_VEL_CMDFILT           = 0x2108,

   OBJID_CRNTLOOP              = 0x60F6,
   OBJID_VELLOOP               = 0x60F9,
   OBJID_POSLOOP               = 0x60FB,

   OBJID_VELLIM_MAXACC         = 0x2100,
   OBJID_VELLIM_MAXDEC         = 0x2101,
   OBJID_VELLIM_ESTOP          = 0x2102,
   OBJID_VELLIM_MAXVEL         = 0x2103,

   OBJID_CRNTLIM_PEAK          = 0x2110,
   OBJID_CRNTLIM_CONT          = 0x2111,
   OBJID_CRNTLIM_TIME          = 0x2112,
   OBJID_CRNT_SLOPE            = 0x2113,

   OBJID_SOFTLIM               = 0x607D,
   OBJID_SOFTLIM_ACCEL         = 0x2253,

   OBJID_MOTOR_MODEL           = 0x6403,
   OBJID_MOTOR_MFG             = 0x6404,
   OBJID_MOTOR_INFO            = 0x6410,

   OBJID_AMP_MODES             = 0x6502,
   OBJID_AMP_MODEL             = 0x6503,
   OBJID_AMP_MFG               = 0x6504,
   OBJID_AMP_WEB               = 0x6505,
   OBJID_AMP_INFO              = 0x6510,

   OBJID_REGEN_RES             = 0x2150,
   OBJID_REGEN_CONT            = 0x2151,
   OBJID_REGEN_PEAK            = 0x2152,
   OBJID_REGEN_TIME            = 0x2153,
   OBJID_REGEN_VON             = 0x2154,
   OBJID_REGEN_VOFF            = 0x2155,
   OBJID_REGEN_MODEL           = 0x2156,
   OBJID_REGEN_FLAGS           = 0x2157,

   OBJID_REF_SCALE             = 0x2310,
   OBJID_REF_OFFSET            = 0x2311,
   OBJID_REF_CALOFF            = 0x2312,
   OBJID_REF_DEADBAND          = 0x2313,

   OBJID_PWMIN_CFG             = 0x2320,
   OBJID_PWMIN_SCALE           = 0x2321,
   OBJID_PWMIN_FREQ            = 0x2322,

   OBJID_FGEN_CFG              = 0x2330,
   OBJID_FGEN_FREQ             = 0x2331,
   OBJID_FGEN_AMP              = 0x2332,
   OBJID_FGEN_DUTY             = 0x2333,

   OBJID_PROG_CRNT             = 0x2340,
   OBJID_PROG_VEL              = 0x2341,

   OBJID_AMP_NAME              = 0x21A0,

   OBJID_CANID_CFG             = 0x21B0,

   OBJID_CANMASK_LIMIT         = 0x2184,

   OBJID_CAP_CTRL              = 0x2400,
   OBJID_CAP_STAT              = 0x2401,
   OBJID_CAP_NDX               = 0x2402,
   OBJID_CAP_HOME              = 0x2403,
   OBJID_CAP_HSTIME            = 0x2404,
   OBJID_CAP_HSPOS             = 0x2405,

   OBJID_MISC_OPTIONS          = 0x2420,
   OBJID_CME2_CONFIG           = 0x2421,
   OBJID_ENCOUT_CONFIG         = 0x2241,

   OBJID_FIRMWARE              = 0x2001,
   OBJID_PDOREQUEST            = 0x2002,

   OBJID_USTEP_HOLDCRNT        = 0x21D0,
   OBJID_USTEP_HOLDTIME        = 0x21D1,
   OBJID_USTEP_VOLTIME         = 0x21D5,

   OBJID_CAM_CONFIG            = 0x2360,
   OBJID_CAM_DELAYF            = 0x2361,
   OBJID_CAM_DELAYR            = 0x2362,
   OBJID_CAM_VEL               = 0x2363,

   OBJID_TRACE_CHANNELS        = 0x2500,
   OBJID_TRACE_STATUS          = 0x2501,
   OBJID_TRACE_REF_PER         = 0x2502,
   OBJID_TRACE_SAMP_CT         = 0x2503,
   OBJID_TRACE_SAMP_MAX        = 0x2504,
   OBJID_TRACE_PERIOD          = 0x2505,
   OBJID_TRACE_TRIGGER         = 0x2506,
   OBJID_TRACE_DELAY           = 0x2507,
   OBJID_TRACE_START           = 0x2508,
   OBJID_TRACE_DATA            = 0x2509,
   OBJID_TRACE_RESERVE         = 0x250A,
   OBJID_TRACE_ADDR            = 0x250B,
   OBJID_TRACE_MEM             = 0x250C,

   OBJID_TORQUE_TARGET         = 0x6071,
   OBJID_TORQUE_DEMAND         = 0x6074,
   OBJID_TORQUE_RATED          = 0x6076,
   OBJID_TORQUE_ACTUAL         = 0x6077,
   OBJID_TORQUE_SLOPE          = 0x6087
};

/***************************************************************************/
/**
Input pin configuration settings.  The digital input pins located on an 
amplifier can be programmed to perform some action.  This enumeration 
provides a list of the possible settings for an input pin.

Note that it is perfectly legal to program more then one input pin to 
perform the same action.  It's often useful to have two hardware disable
inputs for example.  If either of these inputs becomes active, the amplifier
will be disabled.

Whether the inputs are configured to perform some action or not, it's still
possible to read them directly using the Amp::GetInputs function.
*/
/***************************************************************************/
enum INPUT_PIN_CONFIG
{
   INCFG_NONE                  = 0x0000, ///< No function assigned to the input
   INCFG_RESET_R               = 0x0002, ///< Reset the amplifier on the Rising edge of the input
   INCFG_RESET_F               = 0x0003, ///< Reset the amplifier on the Falling edge of the input
   INCFG_POSLIM_H              = 0x0004, ///< Positive limit switch, active High.
   INCFG_POSLIM_L              = 0x0005, ///< Positive limit switch, active Low.
   INCFG_NEGLIM_H              = 0x0006, ///< Negative limit switch, active High. 
   INCFG_NEGLIM_L              = 0x0007, ///< Negative limit switch, active Low.  
   INCFG_MOTOR_TEMP_H          = 0x0008, ///< Motor temp sensor active high            
   INCFG_MOTOR_TEMP_L          = 0x0009, ///< Motor temp sensor active low             
   INCFG_CLR_FAULTS_H          = 0x000A, ///< Clear faults on edge, disable while high 
   INCFG_CLR_FAULTS_L          = 0x000B, ///< Clear faults on edge, disable while low  
   INCFG_RESET_DISABLE_R       = 0x000C, ///< Reset on rising edge, disable while high 
   INCFG_RESET_DISABLE_F       = 0x000D, ///< Reset on falling edge, disable while low  
   INCFG_HOME_H                = 0x000E, ///< Home switch, active high     
   INCFG_HOME_L                = 0x000F, ///< Home switch, active low      
   INCFG_DISABLE_H             = 0x0010, ///< Amplifier disable active high
   INCFG_DISABLE_L             = 0x0011, ///< Amplifier disable active low 
   INCFG_PWM_SYNC_H            = 0x0013, ///< Sync input on falling edge, valid only on high speed inputs
   INCFG_MOTION_ABORT_H        = 0x0014, ///< Abort motion active high 
   INCFG_MOTION_ABORT_L        = 0x0015, ///< Abort motion active low 
   INCFG_SCALE_ADC_H           = 0x0016, ///< Scale analog reference input by a factor of 8 when high
   INCFG_SCALE_ADC_L           = 0x0017, ///< Scale analog reference input by a factor of 8 when low
   INCFG_HIGHSPEED_CAPTURE_R   = 0x0018, ///< High speed position capture on rising edge
   INCFG_HIGHSPEED_CAPTURE_F   = 0x0019  ///< High speed position capture on falling edge
};

/***************************************************************************/
/**
Output pin configuration settings.  The digital output pins located on the
amplifier can be programmed to follow one or more bits in one of the 
amplifier's status words.

This enumeration is used to specify which status word a particular output 
pin will follow, and whether the output will be active high or active low.

Each output pin has a configuration value associated with it (which should 
be programmed using one of the values of this enumeration), and a 32-bit 
mask value.  If the output pin is configured to follow a status register, 
the mask identifies which bit(s) of the status register should be used to 
control the output pin.  If any of the masked bits in the status register 
are set, then the output pin will go active.
*/
/***************************************************************************/
enum OUTPUT_PIN_CONFIG
{
   /// The output pin follows the amplifier's event status register 
   /// and is active Low.
   OUTCFG_EVENT_STATUS_L       = 0x0000,

   /// The output pin follows the amplifier's event status register 
   /// and is active High.
   OUTCFG_EVENT_STATUS_H       = 0x0100,

   /// The output pin follows the latched version of the amplifier's 
   /// event status register and is active Low.
   OUTCFG_EVENT_LATCH_L        = 0x0001,

   /// The output pin follows the latched version of the amplifier's 
   /// event status register and is active High.
   OUTCFG_EVENT_LATCH_H        = 0x0101,

   /// The output pin is manually controlled using the Amp::SetOutputs
   /// function, and the output is active Low.
   OUTCFG_MANUAL_L             = 0x0002,

   /// The output pin is manually controlled using the Amp::SetOutputs
   /// function, and the output is active High.
   OUTCFG_MANUAL_H             = 0x0102,

   /// The output pin follows bits in the trajectory status register
   OUTCFG_TRJ_STATUS           = 0x0003,

   /// The output pin will go active when the actual motor position is
   /// greater then the first output parameter, and less then the 
   /// second output parameter
   OUTCFG_POSITION_WINDOW      = 0x0004,

   /// The output pin will go active when the motor actual position crosses
   /// through a programmed value in the low to high direction.
   /// The pin will stay active for a programmed amount of time.
   /// The first output parameter specifies the position, and the second 
   /// output parameter specifies the time to remain active in milliseconds.
   OUTCFG_POSITION_TRIG_LOW2HIGH = 0x0005,

   /// The output pin will go active when the motor actual position crosses
   /// through a programmed value in the high to low direction.
   /// The pin will stay active for a programmed amount of time.
   /// The first output parameter specifies the position, and the second 
   /// output parameter specifies the time to remain active in milliseconds.
   OUTCFG_POSITION_TRIG_HIGH2LOW = 0x0006,

   /// The output pin will go active when the motor actual position crosses
   /// through a programmed value in either direction.
   /// The pin will stay active for a programmed amount of time.
   /// The first output parameter specifies the position, and the second 
   /// output parameter specifies the time to remain active in milliseconds.
   OUTCFG_POSITION_TRIG        = 0x0007,

   /// The output pin will go active when the motor actual position crosses
   /// through any one of a series of programmed values in either direction.
   /// The pin will stay active for a programmed amount of time.
   /// The list of output positions to trigger on must be uploaded into an 
   /// area of trace memory.  
   /// The first output parameter gives a 16-bit word offset into trace 
   /// memory in it's upper half, and the number of positions in it's 
   /// lower half.
   /// The second parameter specifies the time to remain active in milliseconds.
   OUTCFG_POSITION_TRIG_LIST   = 0x0009,

   /// If set the output pin is used as Sync output. This bit can only
   /// be used with  output pin 0. 
   OUTCFG_SYNC_OUTPUT	       = 0x0200,

   /// This bit may be ORed with any of the other output pin configuration 
   /// values to make them active high.
   OUTCFG_ACTIVE_HIGH          = 0x0100
};

/***************************************************************************/
/**
This enumeration is used to specify the mode of operation of the amplifier.

The amplifier mode of operation specifies the control method to be used by 
the amplifier, as well as the source of input to that control structure.

The amplifier can be controlled in servo mode or in microstepping mode.  When
running in servo mode the amplifier uses up to three nested control loops.
These loops control current, velocity and position.  In microstepping mode
the low level current loop is retained, but the upper level loops are replaced
with a simple position command.

The command source of the amplifier will normally be the CANopen network itself.
However, the amplifier also supports several low level control methods in which
commands are received through analog or digital input pins, or even from an 
internal function generator.

Normally, only the CANopen modes of operation will be used when running over
the CANopen network.  These modes are AMPMODE_CAN_PROFILE, AMPMODE_CAN_VELOCITY,
AMPMODE_CAN_HOMING, and AMPMODE_CAN_PVT.  Each of these modes can be used on 
either servo or microstepping drives.  It's typically not necessary to 
specify the type of control method (servo or microstepping) to be used with 
these modes as it can be determined by the type of amplifier being used.  Servo
amplifier's (such as Accelnet) default to servo mode, and microstepping amplifiers
(such as Stepnet) will default to microstepping mode.  If this default is not
appropriate for the application, then the control method may be forced by ORing
in one of the following two values; AMPMODE_CAN_SERVO and AMPMODE_CAN_USTEP.

*/
/***************************************************************************/
enum AMP_MODE
{

   /// In this mode the CANopen network sends move commands to the amplifier,
   /// and the amplifier uses it's internal trajectory generator to perform 
   /// the moves.
   /// This mode conforms to the CANopen device profile for motion control
   /// (DSP-402) profile position mode.
   AMPMODE_CAN_PROFILE      = 0x0001,

   /// In this mode the CANopen network commands target velocity values to the 
   /// amplifier.  The amplifier uses it's programmed acceleration and deceleration
   /// values to ramp the velocity up/down to the target.
   ///
   /// Note that support for profile velocity mode was added in amplifier firmware
   /// version 3.06.  Earlier versions of firmware will report an error if this
   /// mode is selected.
   AMPMODE_CAN_VELOCITY     = 0x0003,

   /// In this mode the CANopen network commands torque values to the amplifier.
   ///
   /// Note that support for profile torque mode was added in amplifier firmware
   /// version 3.34.  Earlier versions of firmware will report an error if this
   /// mode is selected.
   AMPMODE_CAN_TORQUE       = 0x0004,

   /// This mode is used to home the motor (i.e. find the motor zero position)
   /// under the control of the CANopen network.
   /// This mode conforms to the CANopen device profile for motion control
   /// (DSP-402) homing mode.
   AMPMODE_CAN_HOMING       = 0x0006,

   /// In this mode the CANopen master calculates the motor trajectory and
   /// streams it over the CANopen network as a set of points that the 
   /// amplifier interpolated between.
   /// This mode conforms to the CANopen device profile for motion control
   /// (DSP-402) interpolated position mode.
   AMPMODE_CAN_PVT          = 0x0007,

   /// This value may be combined with one of the standard CAN control modes to
   /// specify that servo control should be used.  This is most often specified
   /// when a microstepping amplifier (such as the Stepnet) is to be used in 
   /// servo mode.
   AMPMODE_CAN_SERVO        = 0x1E00,

   /// This value may be combined with one of the standard CAN control modes to
   /// specify that microstepping control should be used.  This is most often specified
   /// when a servo amplifier (such as the Accelnet) is to be used in 
   /// microstepping mode.
   AMPMODE_CAN_USTEP        = 0x2800,

   /// Disable the amplifier.  In this mode, none of the controls loops are
   /// running, and no voltage will be applied across the motor windings.
   AMPMODE_DISABLED         = 0x0000,

   /// Current mode in which the command to the current loop is simply a 
   /// static value that may be programmed over the serial port or CANopen
   /// network.
   /// The programmed current value can be set with the function 
   /// Amp::SetCurrentProgrammed
   AMPMODE_PROG_CRNT        = 0x0100,

   /// Current mode in which the command to the current loop is derived from
   /// the analog input.  Note that some amplifier models do not support an
   /// analog input.  Please refer to the amplifier datasheet to determine if
   /// this mode is applicable.
   AMPMODE_AIN_CRNT         = 0x0200,

   /// Current mode in which the command to the current loop is derived from
   /// the digital input pins.  One or two of the digital inputs are used as
   /// a PWM input command which is interpreted as a current command.
   /// Please refer to the amplifier data sheet to determine which input(s)
   /// should be used in this mode.
   AMPMODE_DIN_CRNT         = 0x0300,

   /// Current mode in which the command to the current loop is derived from
   /// the internal function generator.
   AMPMODE_FGEN_CRNT        = 0x0400,

   /// Velocity mode in which the command to the velocity loop is simply a
   /// static value that may be programmed over the serial or CANopen 
   /// network.
   /// The programmed velocity value can be set with the function 
   /// Amp::SetVelocityProgrammed
   AMPMODE_PROG_VEL         = 0x0B00,

   /// Velocity mode in which the command to the velocity loop is derived from
   /// the analog input.  Note that some amplifier models do not support an
   /// analog input.  Please refer to the amplifier datasheet to determine if
   /// this mode is applicable.
   AMPMODE_AIN_VEL          = 0x0C00,

   /// Velocity mode in which the command to the velocity loop is derived from
   /// the digital input pins.  One or two of the digital inputs are used as
   /// a PWM input command which is interpreted as a velocity command.
   /// Please refer to the amplifier data sheet to determine which input(s)
   /// should be used in this mode.
   AMPMODE_DIN_VEL          = 0x0D00,

   /// Velocity mode in which the command to the velocity loop is derived from
   /// the internal function generator.
   AMPMODE_FGEN_VEL         = 0x0E00,

   /// Position mode in which the command to the position loop is derived from
   /// the digital input pins.  Two of the digital inputs can be configured as
   /// either a master encoder input (quadrature input), a step & direction 
   /// input, or a step up / step down input.
   /// Please refer to the amplifier data sheet to determine which inputs
   /// should be used in this mode.
   AMPMODE_DIN_POS          = 0x1700,

   /// Position mode in which the command to the position loop is derived from
   /// the internal function generator.
   AMPMODE_FGEN_POS         = 0x1800,

   /// Position mode in which the command to the position loop is derived from
   /// CAM tables located in the amplifiers memory.
   AMPMODE_CAM_POS          = 0x1900,

   /// Microstepping mode in which the commanded position is derived from
   /// the digital input pins.  Two of the digital inputs can be configured as
   /// either a master encoder input (quadrature input), a step & direction 
   /// input, or a step up / step down input.
   /// Please refer to the amplifier data sheet to determine which inputs
   /// should be used in this mode.
   AMPMODE_DIN_USTEP        = 0x2100,

   /// Microstepping mode in which the commanded position is derived from
   /// the internal function generator.
   AMPMODE_FGEN_USTEP       = 0x2200,

   /// Diagnostic microstepping mode.  This is a very simple microstepping
   /// mode that can be used for motor setup and testing.  A constant motor 
   /// current is set by the programmed current value, and the motor phase
   /// is microstepped at a fixed rate.  The position and velocity loops are
   /// not used in this mode.
   AMPMODE_DIAG_USTEP       = 0x2A00
};

/***************************************************************************/
/**
Amplifier event status word bit definitions.
*/
/***************************************************************************/
enum EVENT_STATUS
{
   ESTAT_SHORT_CRCT    = 0x00000001,  ///< Amplifier short circuit 
   ESTAT_AMP_TEMP      = 0x00000002,  ///< Amplifier over temperature
   ESTAT_OVER_VOLT     = 0x00000004,  ///< Amplifier over voltage
   ESTAT_UNDER_VOLT    = 0x00000008,  ///< Amplifier under voltage
   ESTAT_MTR_TEMP      = 0x00000010,  ///< Motor over temperature
   ESTAT_ENCODER_PWR   = 0x00000020,  ///< Encoder power error
   ESTAT_PHASE_ERR     = 0x00000040,  ///< Phasing error
   ESTAT_CRNT_LIM      = 0x00000080,  ///< Current limited
   ESTAT_VOLT_LIM      = 0x00000100,  ///< Voltage limited
   ESTAT_POSLIM        = 0x00000200,  ///< Positive limit switch triggered
   ESTAT_NEGLIM        = 0x00000400,  ///< Negative limit switch triggered
   ESTAT_DISABLE_INPUT = 0x00000800,  ///< Enable input pin not set
   ESTAT_SOFT_DISABLE  = 0x00001000,  ///< Disabled due to software request
   ESTAT_STOP          = 0x00002000,  ///< Try to stop motor (after disable, before brake)
   ESTAT_BRAKE         = 0x00004000,  ///< Brake actuated
   ESTAT_PWM_DISABLE   = 0x00008000,  ///< PWM outputs disabled
   ESTAT_SOFTLIM_POS   = 0x00010000,  ///< Positive software limit reached
   ESTAT_SOFTLIM_NEG   = 0x00020000,  ///< Negative software limit reached
   ESTAT_TRK_ERR       = 0x00040000,  ///< Tracking error
   ESTAT_TRK_WARN      = 0x00080000,  ///< Tracking warning
   ESTAT_RESET         = 0x00100000,  ///< Amplifier has been reset
   ESTAT_POSWRAP       = 0x00200000,  ///< Encoder position wrapped (rotory) or hit limit (linear).
   ESTAT_FAULT         = 0x00400000,  ///< Latching fault in effect
   ESTAT_VEL_LIMIT     = 0x00800000,  ///< Velocity is at limit
   ESTAT_ACC_LIMIT     = 0x01000000,  ///< Acceleration is at limit
   ESTAT_TRK_WIN       = 0x02000000,  ///< Not in tracking window if set
   ESTAT_HOME          = 0x04000000,  ///< Home switch is active
   ESTAT_MOVING        = 0x08000000,  ///< Trajectory generator active OR not yet settled
   ESTAT_VEL_WIN       = 0x10000000,  ///< Velocity error outside of velocity window when set.
   ESTAT_PHASE_INIT    = 0x20000000   ///< Set when using algorithmic phase init mode & phase not initialized.
};

/***************************************************************************/
/**
Amplifier events.  This enumeration provides a list of events that can be
used to wait on amplifier conditions.
*/
/***************************************************************************/
enum AMP_EVENT
{
   /// Set when a move is finished and the amplifier has settled in to position 
   /// at the end of the move.  Cleared when a new move is started.
   AMPEVENT_MOVEDONE      = 0x00000001,

   /// Set when the trajectory generator finishes a move.  The motor may not 
   /// have settled into position at this point.  Cleared when a new move is
   /// started.
   AMPEVENT_TRJDONE       = 0x00000002,

   /// A node guarding (or heartbeat) error has occurred.  This indicates that
   /// the amplifier failed to respond within the expected amount of time for
   /// either a heartbeat or node guarding message.  This could be caused by 
   /// a network wiring problem, amplifier power down, amp reset, etc.
   /// This bit is set when the error occurs, and is cleared by a call to the
   /// function Amp::ClearNodeGuardEvent.
   AMPEVENT_NODEGUARD     = 0x00000004,

   /// This event bit is used internally by the amplifier object.  It is set
   /// when the amp acknowledges a new move start.
   AMPEVENT_SPACK         = 0x00000008,

   /// A latching amplifier fault has occurred.  The specifics of what caused
   /// the fault can be obtained by calling Amp::GetFaults, and the fault conditions
   /// can be cleared by calling Amp::ClearFaults.
   AMPEVENT_FAULT         = 0x00000010,

   /// A non-latching amplifier error has occurred.  
   AMPEVENT_ERROR         = 0x00000020,

   /// The amplifier's absolute position error is greater then the window
   /// set with Amp::SetPositionWarnWindow.
   AMPEVENT_POSWARN       = 0x00000040,

   /// The amplifier's absolute position error is greater then the window
   /// set with Amp::SetSettlingWindow.
   AMPEVENT_POSWIN        = 0x00000080,

   /// The amplifier's absolute velocity error is greater then the window
   /// set with Amp::SetVeliocityWarnWindow
   AMPEVENT_VELWIN        = 0x00000100,

   /// The amplifier's outputs are disabled.  The reason for the disable
   /// can be determined by Amp::GetEventStatus
   AMPEVENT_DISABLED      = 0x00000200,

   /// The positive limit switch is currently active
   AMPEVENT_POSLIM        = 0x00000400,

   /// The negative limit switch is currently active
   AMPEVENT_NEGLIM        = 0x00000800,

   /// The positive software limit is currently active
   AMPEVENT_SOFTLIM_POS   = 0x00001000,

   /// The negative software limit is currently active
   AMPEVENT_SOFTLIM_NEG   = 0x00002000,

   /// The amplifier is presently performing a quick stop sequence
   AMPEVENT_QUICKSTOP     = 0x00004000,

   /// The last profile was aborted without finishing
   AMPEVENT_ABORT         = 0x00008000,

   /// The amplifier is software disabled
   AMPEVENT_SOFTDISABLE   = 0x00010000,

   /// A new home position has been captured.
   /// Note that this features requires firmware version >= 4.77
   AMPEVENT_HOME_CAPTURE  = 0x00020000,

   /// PVT buffer is empty.
   AMPEVENT_PVT_EMPTY     = 0x00040000,

   /// This amplifier's event mask has not yet been initialized.
   /// This event is for internal use only.
   AMPEVENT_NOT_INIT      = 0x80000000
};

/***************************************************************************/
/**
Latching Amplifier faults conditions.  Once a fault is detected in the 
amplifier, the amp will be disabled until the fault condition has been 
cleared.

Use Amp::GetFaults to get a list of any active fault conditions, and 
Amp::ClearFaults to clear one or more faults.
*/
/***************************************************************************/
enum AMP_FAULT
{
   FAULT_DATAFLASH          = 0x0001,   ///< Fatal hardware error: the flash data is corrupt.
   FAULT_ADCOFFSET          = 0x0002,   ///< Fatal hardware error: An A/D offset error has occurred.
   FAULT_SHORT_CRCT         = 0x0004,   ///< The amplifier detected a short circuit condition
   FAULT_AMP_TEMP           = 0x0008,   ///< The amplifier is over temperature
   FAULT_MTR_TEMP           = 0x0010,   ///< A motor temperature error was detected
   FAULT_OVER_VOLT          = 0x0020,   ///< The amplifier bus voltage is over the acceptable limit
   FAULT_UNDER_VOLT         = 0x0040,   ///< The amplifier bus voltage is below the acceptable limit
   FAULT_ENCODER_PWR        = 0x0080,   ///< Over current on the encoder power supply
   FAULT_PHASE_ERR          = 0x0100,   ///< Amplifier phasing error 
   FAULT_TRK_ERR            = 0x0200,   ///< Tracking error, the position error is too large.
   FAULT_I2T_ERR            = 0x0400    ///< Current limited by i^2t algorithm.
};

/***************************************************************************/
/**
The amplifier's halt mode defines it's action when a halt command is issued
(Amp::HaltMove).  When the halt command is issued, the move in progress will
be terminated using the method defined in this mode.  Unless the HALT_DISABLE
method is selected, the amplifier will remain enabled and holding position 
at the end of the halt sequence.
*/
/***************************************************************************/
enum HALT_MODE
{
   HALT_DISABLE             = 0,        ///< Disable the amplifier immediately
   HALT_DECEL               = 1,        ///< Slow down using the profile deceleration
   HALT_QUICKSTOP           = 2,        ///< Slow down using the quick stop deceleration
   HALT_ABRUPT              = 3         ///< Slow down with unlimited deceleration
};

/***************************************************************************/
/**
The amplifier's quick stop mode defines it's action when a quick stop command
is issued (Amp::QuickStop).  

The quick stop command differs from the halt command in that the amplifier 
is always disabled at the end of the sequence.  For some modes, the amplifier
automatically disables after halting the move.  For others, the amplifier 
halts the move and holds in the quick stop state.  No new moves may be started
until the amplifier has manually been disabled.
*/
/***************************************************************************/
enum QUICK_STOP_MODE
{
   QSTOP_DISABLE            = 0,        ///< Disable the amplifier immediately
   QSTOP_DECEL              = 1,        ///< Slow down using the profile deceleration then disable
   QSTOP_QUICKSTOP          = 2,        ///< Slow down using the quick stop deceleration then disable
   QSTOP_ABRUPT             = 3,        ///< Slow down with unlimited deceleration then disable
   QSTOP_DECEL_HOLD         = 5,        ///< Slow down and hold
   QSTOP_QUICKSTOP_HOLD     = 6,        ///< Quick stop and hold
   QSTOP_ABRUPT_HOLD        = 7         ///< Abrupt stop and hold
};

/***************************************************************************/
/**
Home methods supported by the Copley amplifier.  This enumeration gives 
more useful names to the various homing methods currently supported by
the Copley amplifier.

The names of the members of this enumeration define the type of homing
procedure.  These names are made up of the following elements:

-# CHM: prefix that identifies the member as a Copley Home Method
-# Sensor: Defines the type of sensor that defines the location of
           the home position.  It will be one of the following:
   - PLIM:  A positive limit switch
   - NLIM:  A negative limit switch
   - PHOME: A positive home switch.  This is a home switch that goes active
            at some point, and remains active for all greater positions.
   - NHOME: A negative home switch.  This is a home switch that goes active
            at some point, and remains active for all lower positions.
   - LHOME: The lower side of a momentary home switch.  This type of home 
            switch has an active region and is inactive on either side of that 
            region.  This choice selects the lower edge of that switch.
   - UHOME: The upper side of a momentary home switch.  This type of home 
            switch has an active region and is inactive on either side of that 
            region.  This choice selects the upper edge of that switch.
-# Index: This defines whether an encoder index pulse will be used to mark
          the exact home location in conjunction with the sensor.  If so, it
          identifies which index position will be used.  It will be one of the
          following:
   - none: If not specified, then no index is used.  The edge of the sensor 
           will define the home position.
   - ONDX: Outter index switch.  This is an index on the inactive side of
           the sensor
   - INDX: Inner index switch.  This is the first index on the active side of
           the sensor.
-# Initial move direction:  For some home methods, this is provided and defines
           the initial move direction.  The initial move direction is only specified
           if it isn't already obvious based on the home type.
   - none: The initial move direction is always obvious and does not need to be specified.
   - NEG:  Move in the negative direction if the home position is not obvious.
           If the negative limit switch is encountered before the home region is
           found, then the move direction will be reversed.
   - POS:  Move in the negative direction if the home position is not obvious.
           If the positive limit switch is encountered before the home region is
           found, then the move direction will be reversed.
*/
/***************************************************************************/
enum COPLEY_HOME_METHOD
{
   /// Move into the negative limit switch, then back out to the
   /// first encoder index pulse beyond it.  The index position 
   /// is home.
   CHM_NLIM_ONDX      = 1,

   /// Move into the positive limit switch, then back out to the
   /// first encoder index pulse beyond it.  The index position 
   /// is home.
   CHM_PLIM_ONDX      = 2,

   /// Move to a positive home switch, then back out of it to the 
   /// first encoder index outside the home region.  The index 
   /// position is home.
   CHM_PHOME_ONDX     = 3,

   /// Move to a positive home switch, and continue into it to
   /// the first encoder index inside the home region.  The index 
   /// position is home.
   CHM_PHOME_INDX     = 4,

   /// Move to a negative home switch, then back out of it to the 
   /// first encoder index outside the home region.  The index 
   /// position is home.
   CHM_NHOME_ONDX     = 5,

   /// Move to a negative home switch, and continue into it to
   /// the first encoder index inside the home region.  The index 
   /// position is home.
   CHM_NHOME_INDX     = 6,

   /// Move to the lower side of a momentary home switch.  Then
   /// find the first encoder index pulse outside the home region.
   /// If the home switch is not active when the home sequence 
   /// starts, then the initial move will be positive.
   CHM_LHOME_ONDX_POS = 7,

   /// Move to the lower side of a momentary home switch.  Then
   /// find the first encoder index pulse inside the home region.
   /// If the home switch is not active when the home sequence 
   /// starts, then the initial move will be positive.
   CHM_LHOME_INDX_POS = 8,

   /// Move to the upper side of a momentary home switch.  Then
   /// find the first encoder index pulse inside the home region.
   /// If the home switch is not active when the home sequence 
   /// starts, then the initial move will be positive.
   CHM_UHOME_INDX_POS = 9,

   /// Move to the upper side of a momentary home switch.  Then
   /// find the first encoder index pulse outside the home region.
   /// If the home switch is not active when the home sequence 
   /// starts, then the initial move will be positive.
   CHM_UHOME_ONDX_POS = 10,

   /// Move to the upper side of a momentary home switch.  Then
   /// find the first encoder index pulse outside the home region.
   /// If the home switch is not active when the home sequence 
   /// starts, then the initial move will be negative.
   CHM_UHOME_ONDX_NEG = 11,

   /// Move to the upper side of a momentary home switch.  Then
   /// find the first encoder index pulse inside the home region.
   /// If the home switch is not active when the home sequence 
   /// starts, then the initial move will be negative.
   CHM_UHOME_INDX_NEG = 12,

   /// Move to the lower side of a momentary home switch.  Then
   /// find the first encoder index pulse inside the home region.
   /// If the home switch is not active when the home sequence 
   /// starts, then the initial move will be negative.
   CHM_LHOME_INDX_NEG = 13,

   /// Move to the lower side of a momentary home switch.  Then
   /// find the first encoder index pulse outside the home region.
   /// If the home switch is not active when the home sequence 
   /// starts, then the initial move will be negative.
   CHM_LHOME_ONDX_NEG = 14,

   /// Move into the negative limit switch.  The edge of the 
   /// limit is home.
   CHM_NLIM           = 17,

   /// Move into the positive limit switch.  The edge of the
   /// limit is home.
   CHM_PLIM           = 18,

   /// Move to a positive home switch.  The edge of the home
   /// region is home.
   CHM_PHOME          = 19,

   /// Move to a negative home switch.  The edge of the home
   /// region is home.
   CHM_NHOME          = 21,

   /// Move to the lower side of a momentary home switch.  
   /// The edge of the home region is home.
   /// If the home switch is not active when the home sequence 
   /// starts, then the initial move will be positive.
   CHM_LHOME_POS      = 23,

   /// Move to the upper side of a momentary home switch.
   /// The edge of the home region is home.
   /// If the home switch is not active when the home sequence 
   /// starts, then the initial move will be positive.
   CHM_UHOME_POS      = 25,

   /// Move to the upper side of a momentary home switch.
   /// The edge of the home region is home.
   /// If the home switch is not active when the home sequence 
   /// starts, then the initial move will be negative.
   CHM_UHOME_NEG      = 27,

   /// Move to the lower side of a momentary home switch.
   /// The edge of the home region is home.
   /// If the home switch is not active when the home sequence 
   /// starts, then the initial move will be negative.
   CHM_LHOME_NEG      = 29,

   /// Move in the negative direction until the first encoder
   /// index pulse is found.  The index position is home.
   CHM_NDX_NEG        = 33,

   /// Move in the positive direction until the first encoder
   /// index pulse is found.  The index position is home.
   CHM_NDX_POS        = 34,

   /// Set the current position to home.
   CHM_NONE           = 35,

   /// Home to a hard stop.  The motor will start running in the
   /// positive direction until the homing current has been reached.
   /// It will hold this current until the homing delay has expired.
   /// The actual position after that delay is home.
   CHM_HARDSTOP_POS   = 255,

   /// Home to a hard stop.  The motor will start running in the
   /// negative direction until the homing current has been reached.
   /// It will hold this current until the homing delay has expired.
   /// The actual position after that delay is home.
   CHM_HARDSTOP_NEG   = 254,

   /// Home to a hard stop.  The motor will start running in the
   /// positive direction until the homing current has been reached.
   /// It will hold this current until the homing delay has expired.
   /// It will them move away from the hard stop until an index mark
   /// is located.  The index position is home.
   CHM_HARDSTOP_ONDX_POS = 253,

   /// Home to a hard stop.  The motor will start running in the
   /// negative direction until the homing current has been reached.
   /// It will hold this current until the homing delay has expired.
   /// It will them move away from the hard stop until an index mark
   /// is located.  The index position is home.
   CHM_HARDSTOP_ONDX_NEG = 252,

   /// Home using an extended home method.  This is not a real home 
   /// method, but instead a flag that is used by the Amp::GoHome function
   /// to indicate that a non-standard homing method is to be used.
   /// When this value is passed it instructs the GoHome function to 
   /// execute the home sequence through the use of a special low-level 
   /// homing parameter implemented in the amplifier firmware.  This 
   /// allows a bit more flexibility on the home sequencer.
   CHM_EXTENDED = 256
};

/***************************************************************************/
/**
Point to point profile types.  This enumeration gives the various profile
types supported by the Copley amplifiers.  These profile types are used
when running in profile position mode (point to point moves).
*/
/***************************************************************************/
enum PROFILE_TYPE
{
   /// Velocity profile.  In this profile mode the velocity, acceleration and deceleration
   /// values are used.  The position value is also used, but it only defines the direction
   /// of motion ( positive is position is >= 0, negative if position is < 0 ).
   PROFILE_VEL = -1,

   /// Trapezoidal profile.  In this profile mode a position, velocity, acceleration
   /// and deceleration may be specified.
   /// This profile mode allows any of it's parameters (position, vel, accel, decel)
   /// to be changed during the course of a move.
   PROFILE_TRAP = 0,

   /// Jerk limited (S-curve) profile.  In this mode, position, velocity, acceleration,
   /// and jerk (rate of change of acceleration) may be specified.
   PROFILE_SCURVE = 3
};

/***************************************************************************/
/**
Amplifier phasing mode.  This enumeration gives the legal values for the 
amplifier phasing mode setting.  The phasing mode controls what type of input
the amplifier uses to determing the phase angle when commutating a brushless
DC motor.
*/
/***************************************************************************/
enum AMP_PHASE_MODE
{
   /// Use a combination of hall sensors and encoder input.  The hall sensors
   /// are used at startup, and will be used to constantly adjust the phase
   /// angle on every hall transition.  This is the default phase mode, and 
   /// should be used when both hall sensors and encoder input are present.
   PHASE_MODE_ENCODER   = 0,

   /// Phase using only the hall sensor inputs.  This mode gives rougher 
   /// operation then the encoder based mode, however it can be used when no
   /// encoder input is available.
   PHASE_MODE_TRAP      = 1,

   /// Use both encoder & hall inputs, but only use the hall inputs on startup
   /// and ignore them after that.  This mode should normally not be used unless
   /// there is a good reason to ignore the hall inputs after startup.
   PHASE_MODE_NOADJUST  = 2,

   /// This phase mode is used to force brushed DC motor output.  It should only
   /// be used when the amplifier is connected to a brushed DC motor.
   PHASE_MODE_BRUSHED   = 4,

   /// Phase using the encoder only.  In this mode, the amplifier will use an
   /// algorithmic phase initialization on startup.  This mode can be used when an
   /// encoder is present, but no halls are available.
   PHASE_MODE_NOHALL    = 5
};

/***************************************************************************/
/**
Amplifier PWM output mode.  This enumeration gives the legal values for setting
up the amplifier's PWM output mode.  The PWM output mode controls some details 
of how the amplifier drives it's PWM outputs.
*/
/***************************************************************************/
enum AMP_PWM_MODE
{
   /// Standard PWM mode.  This mode should be selected for most applications.
   PWM_MODE_STANDARD    = 0x0000,

   /// This bit forces the amplifier into PWM bus clamping mode.  Bus clamping
   /// mode is a different method of driving the PWM outputs.  It can produce
   /// less switching loss at the expense of greater cross over distortion.
   PWM_MODE_FORCECLAMP  = 0x0001,

   /// Automatically switch between bus clamping and normal output mode based
   /// on the PWM duty cycle.  Bus clamping mode is used at high duty cycles,
   /// normal mode is used at low duty cycles.
   PWM_MODE_AUTOCLAMP   = 0x0002,

   /// If this bit is set, the amplifier's output voltage is limited using a
   /// method known as hexagonal limiting.  If this bit is clear, circular limiting
   /// is used.  Hexagonal limiting gives the maximum voltage output at the 
   /// expense of some added torque ripple.  Higher top speeds may be attained
   /// using hexagonal limiting.
   PWM_MODE_HEXLIMIT    = 0x0010
};

/***************************************************************************/
/**
Amplifier trace variables.  This enumeration lists the amplifier variables that
are available for use with the amplifier's internal trace routine.
*/
/***************************************************************************/
enum AMP_TRACE_VAR
{
   TRACEVAR_HIGH_VOLT    = 6,         ///< High voltage bus
   TRACEVAR_TEMP         = 37,        ///< Amplifier temperature
   TRACEVAR_ANALOG_REF   = 5,         ///< Analog reference input
   TRACEVAR_ENC_SIN      = 46,        ///< Analog encoder sine
   TRACEVAR_ENC_COS      = 47,        ///< Analog encoder cosine

   TRACEVAR_PHASE        = 36,        ///< Motor phase angle
   TRACEVAR_HALLS        = 40,        ///< Hall sensor state
   TRACEVAR_INPUTS       = 48,        ///< digital input pins (after deadtime)
   TRACEVAR_RAW_INPUTS   = 33,        ///< digital input pins (before deadtime)
   TRACEVAR_EVENTS       = 38,        ///< Event status register
   TRACEVAR_EVENTLATCH   = 39,        ///< Latched version of event status register

   TRACEVAR_CRNT_A       = 3,         ///< Actual current, current sensor A
   TRACEVAR_CRNT_B       = 4,         ///< Actual current, current sensor B
   TRACEVAR_CRNT_CMD     = 7,         ///< Commanded current (before limiting)
   TRACEVAR_CRNT_LIM     = 8,         ///< Commanded current (after limiting)
   TRACEVAR_CRNT_CMD_D   = 9,         ///< Commanded current, D axis
   TRACEVAR_CRNT_CMD_Q   = 10,        ///< Commanded current, Q axis
   TRACEVAR_CRNT_ACT_D   = 13,        ///< Actual current, calculated for D axis
   TRACEVAR_CRNT_ACT_Q   = 14,        ///< Actual current, calculated for Q axis
   TRACEVAR_CRNT_ERR_D   = 15,        ///< Current loop error, D axis
   TRACEVAR_CRNT_ERR_Q   = 16,        ///< Current loop error, Q axis
   TRACEVAR_VOLT_D       = 19,        ///< Current loop output voltage, D axis
   TRACEVAR_VOLT_Q       = 20,        ///< Current loop output voltage, Q axis

   TRACEVAR_VEL_MTR      = 23,        ///< Motor velocity with some filtering
   TRACEVAR_VEL_RAW      = 50,        ///< Motor velocity, unfiltered
   TRACEVAR_VEL_LOAD     = 43,        ///< Load encoder velocity
   TRACEVAR_VLOOP_CMD    = 24,        ///< Velocity loop commanded velocity (before limiting)
   TRACEVAR_VLOOP_LIM    = 25,        ///< Velocity loop commanded velocity (after limiting)
   TRACEVAR_VLOOP_ERR    = 26,        ///< Velocity loop error

   TRACEVAR_LOAD_POS     = 28,        ///< Load encoder position
   TRACEVAR_MTR_POS      = 31,        ///< Motor encoder position
   TRACEVAR_POS_ERR      = 30,        ///< Position error
   TRACEVAR_CMD_POS      = 29,        ///< Commanded position from trajectory generator
   TRACEVAR_CMD_VEL      = 44,        ///< Commanded velocity from trajectory generator
   TRACEVAR_CMD_ACC      = 45,        ///< Commanded acceleration from trajectory generator
   TRACEVAR_DEST_POS     = 49         ///< Destination position
};

/***************************************************************************/
/**
 Amplifier trace status bits.  The amplifier's trace mechanism reports it's 
 status as a collection of these bits.
 */
/***************************************************************************/
enum AMP_TRACE_STATUS
{
   TRACESTAT_RUNNING      = 0x0001,
   TRACESTAT_TRIGGERED    = 0x0002,
   TRACESTAT_SAMPLED      = 0x0004,
   TRACESTAT_NODELAY      = 0x0008
};

/***************************************************************************/
/**
Amplifier trace trigger settings.
 */
/***************************************************************************/
enum AMP_TRACE_TRIGGER
{
   /// These bits define which of the trace channels to use for 
   /// triggering.  Not all trigger types require a trace channel,
   /// for those this value is ignored.
   TRACETRIG_CHANNEL    = 0x000F,

   /// These bits define the trace trigger type to use.
   TRACETRIG_TYPE       = 0x0F00,

   /// Trace trigger type none.  The trace is triggered 
   /// immediately on start.	
   TRACETRIG_NONE       = 0x0000,

   /// Trigger as soon as the value on the selected variable 
   /// is above the trigger level.
   TRACETRIG_ABOVE      = 0x0100,

   /// Trigger as soon as the value on the selected variable 
   /// is below the trigger level.
   TRACETRIG_BELOW      = 0x0200,

   /// Trigger when the value on the selected variable changes
   /// from below the trigger level to above it.
   TRACETRIG_RISE       = 0x0300,

   /// Trigger when the value on the selected variable changes
   /// from above the trigger level to below it.
   TRACETRIG_FALL       = 0x0400,

   /// Treat the trigger level as a bit mask which selects one or more
   /// bits on the selected trace variable.  The trigger occurs as soon
   /// as any of the selected bits are set.
   TRACETRIG_BITSET     = 0x0500,

   /// Treat the trigger level as a bit mask which selects one or more
   /// bits on the selected trace variable.  The trigger occurs as soon
   /// as any of the selected bits are clear.
   TRACETRIG_BITCLR     = 0x0600,

   /// Trigger any time the selected trace variable value changes.
   TRACETRIG_CHANGE     = 0x0700,

   /// Treat the trigger level as a bit mask which selects one or more
   /// bits on the amplifier's event status register.  The trigger occurs
   /// as any of the selected bits are set.  Note that this trigger type
   /// does not use a trace variable.
   TRACETRIG_EVENTSET   = 0x0800,

   /// Treat the trigger level as a bit mask which selects one or more
   /// bits on the amplifier's event status register.  The trigger occurs
   /// as any of the selected bits are clear.  Note that this trigger type
   /// does not use a trace variable.
   TRACETRIG_EVENTCLR   = 0x0900,

   /// Trigger at the start of the next function generator cycle.  This trigger
   /// type is only useful when running in function generator mode.  It does
   /// not use a trace variable or the trigger level.
   TRACETRIG_FGEN_CYCLE = 0x0A00,

   /// If this bit is set, then the trigger is allowed to occur even if the 
   /// trace setup delay has not yet occurred.  Normally, if a negative trace
   /// delay is set then that much time must expire after the trace has been
   /// started before a trigger will be recognized.  If this bit is set, the
   /// trigger will be recognized even if the setup delay hasn't been met.
   TRACETRIG_NODELAY    = 0x4000,

   /// Only take a single sample for each trigger.  Normally, the occurance of the
   /// trigger causes the trace to begin sampling data and stop when the trace
   /// buffer is full.  If this bit is set, each trigger occurance will cause a 
   /// single sample of trace data.
   TRACETRIG_SAMPLE     = 0x8000
};

/***************************************************************************/
/**
Position capture configuration.

The amplifier is able to capture the encoder position on one of two events;
either the encoder index signal, or a general purpose input pin which has been
configured as a home switch.

This enumeration gives the values that my be used to configure this capture 
mechanism using the Amp::SetPosCaptureCfg method.
 */
/***************************************************************************/
enum POS_CAPTURE_CFG
{
   /// If this bit is set, the rising edge of the encoder index signal 
   /// will be used to capture the index position.
   CAPTURE_INDEX_RISING      = 0x0001,

   /// If this bit is set, the falling edge of the encoder index signal
   /// will be used to capture the index position.
   CAPTURE_INDEX_FALLING     = 0x0002,

   /// If this bit is set, then index capture values will not be overwritten
   /// if a new index edge is received before the previously captured value
   /// has been read.
   CAPTURE_INDEX_LATCH       = 0x0004,

   /// If this bit is set, then captured home sensor positions will not be
   /// overwritten if a new home input edge is received before the previous
   /// captured value was read.
   CAPTURE_HOME_LATCH        = 0x0040,

   /// If this bit is set, then the high speed input based position capture
   /// is enabled.  Note that this features requires firmware versions >=
   /// 5.12 to work.
   CAPTURE_HIGH_SPEED_INPUT  = 0x0100,

   /// If this bit is set, then captured high speed input positions will not
   /// be overwritten if a new input is received before the previous position
   /// was read.
   CAPTURE_HIGH_SPEED_INPUT_LATCH = 0x0400
};

/***************************************************************************/
/**
  Position capture status register value.

  The current status of the position capture mechanism may be read from the 
  amplifier using the method Amp::GetPosCaptureCfg.

  This status value is bitmapped as described by this enumeration.  Any bits
  not described here should be ignored.  Bits not described here are reserved
  and may be either 1 or 0.
 */
/***************************************************************************/
enum POS_CAPTURE_STAT
{
   /// If this bit is set it indicates that a new encoder index position has
   /// been captured.  This position may be read using the method
   /// Amp::GetIndexCapture.  Reading the captured position will cause this
   /// bit to be cleared.
   CAPTURE_INDEX_FULL         = 0x0001,

   /// If this bit is set it indicates that a new encoder index was received
   /// before the previous captured index position was read from the amplifier.
   /// The setting of the CAPTURE_INDEX_LATCH bit in the capture control register
   /// determines whether or not the new captured position was stored.  If the
   /// CAPTURE_INDEX_LATCH configuration bit is set, then the new captured position
   /// will be lost.  If this bit is clear then the newly captured position will
   /// overwrite the previous position.
   /// 
   /// Reading the captured position will cause this bit to be cleared.
   CAPTURE_INDEX_OVER         = 0x0008,

   /// If this bit is set it indicates that a new home sensor position has
   /// been captured.  This position may be read using the method
   /// Amp::GetHomeCapture.  Reading the captured position will cause this
   /// bit to be cleared.
   CAPTURE_HOME_FULL          = 0x0010,

   /// If this bit is set it indicates that a new home sensor transition was received
   /// before the previous captured home position was read from the amplifier.
   /// The setting of the CAPTURE_HOME_LATCH bit in the capture control register
   /// determines whether or not the new captured position was stored.  If the
   /// CAPTURE_HOME_LATCH configuration bit is set, then the new captured position
   /// will be lost.  If this bit is clear then the newly captured position will
   /// overwrite the previous position.
   /// 
   /// Reading the captured position will cause this bit to be cleared.
   CAPTURE_HOME_OVER          = 0x0080
};

CML_NAMESPACE_END()

#endif

