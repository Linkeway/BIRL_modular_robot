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
This file contains the AMP object methods used to handle unit conversions.
*/
/***************************************************************************/

#include "CML.h"

CML_NAMESPACE_USE();

// macro used for rounding floats
#define Round(x)  ((x>=0) ? (x+0.5) : (x-0.5))

/***************************************************************************/
/**
Configure the user programmable units.

Unit conversions may be enabled or disabled at compile time through a setting
in CML_Settings.h.  If this feature is disabled, then all position, velocity,
acceleration & jerk values are passed as 32-bit integers in the amplifier's 
native units:

- Position: Encoder counts
- Velocity: 0.1 encoder counts / second (i.e. 100 would be 10 counts/sec)
- Acceleration: 10 counts / second ^ 2 (i.e. 100 would be 1000 counts/sec^2)
- Jerk: 100 counts / second^3 (i.e. 100 would be 10000 counts/sec^3).

If unit conversions are enabled in CML_Settings.h, then these values are 
passed as double precision floating point values, and this function may be 
used to set a scaling factor for these units.  The scaling factor is
passed to this function as a number of encoder counts / user distance unit.

Velocity units are always equal to distance units / second.  Likewise, 
acceleration and jerk units are distance units / second^2 and distance
units / second^3.

For example, if the motor in question has a 1 micron encoder, then user units
of meters, meters/sec, meters/sec^2, etc can be selected by passing a value of
1,000,000 to this function (i.e. the number of microns/meter).

When user units are enabled at compile time, the amplifier defaults to units of 
encoder counts, encoder counts / second, etc.

@param cts The number of encoder counts / user distance unit.
@return A pointer to an error object, or NULL on success.  Note that if user
        units are disabled in CML_Settings.h, then this function will return
		  an error.
*/
/***************************************************************************/
const Error *Amp::SetCountsPerUnit( uunit cts )
{
#ifdef CML_ENABLE_USER_UNITS
	u2lPos = cts;         l2uPos = 1.0/u2lPos;
	u2lVel = cts*10.0;    l2uVel = 1.0/u2lVel;
	u2lAcc = cts*0.1;     l2uAcc = 1.0/u2lAcc;
	u2lJrk = cts*0.01;    l2uJrk = 1.0/u2lJrk;

	u2mPos = u2lPos;  m2uPos = l2uPos;
	u2mVel = u2lVel;  m2uVel = l2uVel;
	u2mAcc = u2lAcc;  m2uAcc = l2uAcc;
	return 0;
#else
	return &AmpError::NoUserUnits;
#endif
}

/***************************************************************************/
/**
Configure the user programmable units for a dual encoder system.
This method provides the same feature as the single encoder version however
it takes two scaling parameters; a load encoder scaler and a motor encoder
scaler.  These two values are used to scale amplifier parameters based on
which encoder they refer to.

@param load The load encoder scaling factor.  This gives the number of load
  encoder counts / user position unit.
@param mtr The motor encoder scaling factor.  This gives the number of motor
  encoder counts / user position unit.
  
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *Amp::SetCountsPerUnit( uunit load, uunit mtr )
{
#ifdef CML_ENABLE_USER_UNITS
	u2lPos = load;         l2uPos = 1.0/u2lPos;
	u2lVel = load* 10.0;   l2uVel = 1.0/u2lVel;
	u2lAcc = load*  0.1;   l2uAcc = 1.0/u2lAcc;
	u2lJrk = load*  0.01;  l2uJrk = 1.0/u2lJrk;

	u2mPos = mtr;          m2uPos = 1.0/u2mPos;
	u2mVel = mtr* 10.0;    m2uVel = 1.0/u2mVel;
	u2mAcc = mtr*  0.1;    m2uAcc = 1.0/u2mAcc;
	return 0;
#else
	return &AmpError::NoUserUnits;
#endif
}

/***************************************************************************/
/**
Get the number of encoder counts / user distance unit.  This function is only
available when user units are selected in CML_Settings.h.

This value defaults to 1.0 (i.e. user distance units are in encoder counts).
It can be adjusted if some other distance unit is desired.

This value controls velocity, acceleration, and jerk units also.  These units
are always based on a time interval of seconds.

@param cts The count value will be returned here
@return A pointer to an error object, or NULL on success
*/
/***************************************************************************/
const Error *Amp::GetCountsPerUnit( uunit &cts )
{
#ifdef CML_ENABLE_USER_UNITS
	cts = u2lPos;
	return 0;
#else
	return &AmpError::NoUserUnits;
#endif
}

/***************************************************************************/
/**
Get the number of encoder counts / user distance unit for both encoders
in a dual encoder system.  This function is only available when user units 
are selected in CML_Settings.h.

These values default to 1.0 (i.e. user distance units are in encoder counts).
It can be adjusted if some other distance unit is desired.

These values control velocity, acceleration, and jerk units also.  These units
are always based on a time interval of seconds.

@param load The load encoder scaling factor will be returned here
@param mtr The motor encoder scaling factor will be returned here

@return A pointer to an error object, or NULL on success
*/
/***************************************************************************/
const Error *Amp::GetCountsPerUnit( uunit &load, uunit &mtr )
{
#ifdef CML_ENABLE_USER_UNITS
	load = u2lPos;
	mtr  = u2mPos;
	return 0;
#else
	return &AmpError::NoUserUnits;
#endif
}

/***************************************************************************/
/**
Convert a position from user position units to internal amplifier units.

Internal to the amplifier, all positions are stored in units of encoder counts.
If user units are not enabled in CML_Settings.h, then user units are also in
encoder counts and this function has no effect.

If user units are enabled at compile time, then this function converts from 
user units (defined using Amp::SetCountsPerUnit) to these internal amplifier units.

For dual encoder systems the unit conversion used by this function is based on
the load encoder resolution.  To convert motor encoder positions, use Amp::PosUser2Mtr.
On single encoder systems either of these functions can be used.

@param pos The position in user units
@return The position in encoder counts
*/
/***************************************************************************/
int32 Amp::PosUser2Load( uunit pos )
{
#ifdef CML_ENABLE_USER_UNITS
	pos *= u2lPos;
	return (int32)Round(pos);
#else
	return pos;
#endif
}

/***************************************************************************/
/**
Convert a velocity from user units to internal amplifier units.

Internal to the amplifier, all velocities are stored in units of 
0.1 encoder counts / second.  If user units are not enabled in 
CML_Settings.h, then user units are the same as amplifier units,
and this function has no effect.

If user units are enabled, then this function converts from user units
(defined using Amp::SetCountsPerUnit) to these internal amplifier units.

For dual encoder systems the unit conversion used by this function is based on
the load encoder resolution.  To convert motor encoder velocities, use Amp::VelUser2Mtr.
On single encoder systems either of these functions can be used.

@param vel The velocity in user units
@return The velocity in 0.1 encoder counts / second
*/
/***************************************************************************/
int32 Amp::VelUser2Load( uunit vel )
{
#ifdef CML_ENABLE_USER_UNITS
	vel *= u2lVel;
	return (int32)Round(vel);
#else
	return vel;
#endif
}

/***************************************************************************/
/**
Convert an acceleration from user units to internal amplifier units.

Internal to the amplifier, all accelerations are stored in units of 
10 encoder counts / second / second.  If user units are not enabled in 
CML_Settings.h, then user units are the same as amplifier units, and 
this function has no effect.

If user units are enabled, then this function converts from user units
(defined using Amp::SetCountsPerUnit) to these internal amplifier units.

For dual encoder systems the unit conversion used by this function is based on
the load encoder resolution.  To convert motor encoder accelerations, use 
Amp::AccUser2Mtr.  On single encoder systems either of these functions can be used.

@param acc The acceleration in user units
@return The acceleration in 10 encoder counts / second ^ 2 units
*/
/***************************************************************************/
int32 Amp::AccUser2Load( uunit acc )
{
#ifdef CML_ENABLE_USER_UNITS
	acc *= u2lAcc;
	return (int32)Round(acc);
#else
	return acc;
#endif
}

/***************************************************************************/
/**

Convert a jerk value from user units to internal amplifier units.

Internal to the amplifier, all jerk values are stored in units of 
100 encoder counts / second ^ 3.  If user units are not enabled in 
CML_Settings.h, then user units are the same as amplifier units, and 
this function has no effect.

If user units are enabled, then this function converts from user units
(defined using Amp::SetCountsPerUnit) to these internal amplifier units.

For dual encoder systems the unit conversion used by this function is based on
the load encoder resolution.

@param jrk The jerk in user units
@return The jerk in 100 encoder counts / second ^ 3 units
*/
/***************************************************************************/
int32 Amp::JrkUser2Load( uunit jrk )
{
#ifdef CML_ENABLE_USER_UNITS
	jrk *= u2lJrk;
	return (int32)Round(jrk);
#else
	return jrk;
#endif
}

/***************************************************************************/
/**
Convert a position from internal amplifier units to user units

Internal to the amplifier, all positions are stored in units of encoder counts.
If user units are not enabled in CML_Settings.h, then user units are also in
encoder counts and this function has no effect.

If user units are enabled, then this function converts from amplifier units
to user units (defined using Amp::SetCountsPerUnit).

For dual encoder systems the unit conversion used by this function is based on
the load encoder resolution.  To convert motor encoder positions, use Amp::PosMtr2User.
On single encoder systems either of these functions can be used.

@param pos The position in encoder counts
@return The position in user units
*/
/***************************************************************************/
uunit Amp::PosLoad2User( int32 pos )
{
#ifdef CML_ENABLE_USER_UNITS
	return pos * l2uPos;
#else
	return pos;
#endif
}

/***************************************************************************/
/**
Convert a velocity from internal amplifier units to user units

Internal to the amplifier, all velocities are stored in units of 
0.1 encoder counts / second.  If user units are not enabled in 
CML_Settings.h, then user units are the same as amplifier units,
and this function has no effect.

If user units are enabled, then this function converts from amplifier units
to user units (defined using Amp::SetCountsPerUnit).

For dual encoder systems the unit conversion used by this function is based on
the load encoder resolution.  To convert motor encoder velocities, use Amp::VelMtr2User.
On single encoder systems either of these functions can be used.

@param vel The velocity in 0.1 encoder counts / second
@return The velocity in user units
*/
/***************************************************************************/
uunit Amp::VelLoad2User( int32 vel )
{
#ifdef CML_ENABLE_USER_UNITS
	return vel * l2uVel;
#else
	return vel;
#endif
}

/***************************************************************************/
/**
Convert an acceleration from internal amplifier units to user units

Internal to the amplifier, all accelerations are stored in units of 
10 encoder counts / second^2.  If user units are not enabled in 
CML_Settings.h, then user units are the same as amplifier units,
and this function has no effect.

If user units are enabled, then this function converts from amplifier units
to user units (defined using Amp::SetCountsPerUnit).

For dual encoder systems the unit conversion used by this function is based on
the load encoder resolution.  To convert motor encoder accelerations, use Amp::AccMtr2User.
On single encoder systems either of these functions can be used.

@param acc The acceleration in units of 10 encoder counts / second^2 
@return The acceleration in user units
*/
/***************************************************************************/
uunit Amp::AccLoad2User( int32 acc )
{
#ifdef CML_ENABLE_USER_UNITS
	return acc * l2uAcc;
#else
	return acc;
#endif
}

/***************************************************************************/
/**
Convert a jerk value from internal amplifier units to user units

Internal to the amplifier, all jerk values are stored in units of 
100 encoder counts / second^3.  If user units are not enabled in 
CML_Settings.h, then user units are the same as amplifier units,
and this function has no effect.

If user units are enabled, then this function converts from amplifier units
to user units (defined using Amp::SetCountsPerUnit).

For dual encoder systems the unit conversion used by this function is based on
the load encoder resolution.  

@param jrk The jerk value in units of 100 encoder counts / second^3
@return The jerk in user units
*/
/***************************************************************************/
uunit Amp::JrkLoad2User( int32 jrk )
{
#ifdef CML_ENABLE_USER_UNITS
	return jrk * l2uJrk;
#else
	return jrk;
#endif
}

/***************************************************************************/
/**
Convert a position from user position units to internal amplifier units.

This function converts using motor encoder units on a dual encoder system.  
Load encoder positions can be converted using Amp::PosUser2Load.

@param pos The position in user units
@return The position in encoder counts
*/
/***************************************************************************/
int32 Amp::PosUser2Mtr( uunit pos )
{
#ifdef CML_ENABLE_USER_UNITS
	pos *= u2mPos;
	return (int32)Round(pos);
#else
	return pos;
#endif
}

/***************************************************************************/
/**
Convert a velocity from user units to internal amplifier units.

This function converts using motor encoder units on a dual encoder system.
Load encoder velocities can be converted using Amp::VelUser2Load.

@param vel The velocity in user units
@return The velocity in 0.1 encoder counts / second
*/
/***************************************************************************/
int32 Amp::VelUser2Mtr( uunit vel )
{
#ifdef CML_ENABLE_USER_UNITS
	vel *= u2mVel;
	return (int32)Round(vel);
#else
	return vel;
#endif
}

/***************************************************************************/
/**
Convert an acceleration from user units to internal amplifier units.

This function converts using motor encoder units on a dual encoder system.
Load encoder accelerations can be converted using Amp::AccUser2Load.

@param acc The acceleration in user units
@return The acceleration in 10 encoder counts / second ^ 2 units
*/
/***************************************************************************/
int32 Amp::AccUser2Mtr( uunit acc )
{
#ifdef CML_ENABLE_USER_UNITS
	acc *= u2mAcc;
	return (int32)Round(acc);
#else
	return acc;
#endif
}

/***************************************************************************/
/**
Convert a position from internal amplifier units to user units

This function converts using motor encoder units on a dual encoder system.
Load encoder positions can be converted using Amp::PosLoad2User.

@param pos The position in encoder counts
@return The position in user units
*/
/***************************************************************************/
uunit Amp::PosMtr2User( int32 pos )
{
#ifdef CML_ENABLE_USER_UNITS
	return pos * m2uPos;
#else
	return pos;
#endif
}

/***************************************************************************/
/**
Convert a velocity from internal amplifier units to user units

This function converts using motor encoder units on a dual encoder system.
Load encoder velcities can be converted using Amp::VelLoad2User.

@param vel The velocity in 0.1 encoder counts / second
@return The velocity in user units
*/
/***************************************************************************/
uunit Amp::VelMtr2User( int32 vel )
{
#ifdef CML_ENABLE_USER_UNITS
	return vel * m2uVel;
#else
	return vel;
#endif
}

/***************************************************************************/
/**
Convert an acceleration from internal amplifier units to user units

This function converts using motor encoder units on a dual encoder system.
Load encoder accelerations can be converted using Amp::AccLoad2User.

@param acc The acceleration in units of 10 encoder counts / second^2 
@return The acceleration in user units
*/
/***************************************************************************/
uunit Amp::AccMtr2User( int32 acc )
{
#ifdef CML_ENABLE_USER_UNITS
	return acc * m2uAcc;
#else
	return acc;
#endif
}


