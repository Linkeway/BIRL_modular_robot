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
This file contains the AMP object methods used to upload / download 
structures containing groups of amplifier parameters.
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
                   Amplifier & Motor information
*/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Read the Amplifier information parameters from the drive.
  These parameters describe the amplifiers capabilities.  They 
  are read only.

  @param info A structure that will be filled with the amplifier info
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetAmpInfo( AmpInfo &info )
{
   int32 l;

   const Error *err = sdo.UpldString( OBJID_AMP_MODEL, 0, l=COPLEY_MAX_STRING, info.model   );
   if( !err ) err = sdo.UpldString( OBJID_AMP_MFG,   0, l=COPLEY_MAX_STRING, info.mfgName );
   if( !err ) err = sdo.UpldString( OBJID_AMP_WEB,   0, l=COPLEY_MAX_STRING, info.mfgWeb  );
   if( !err ) err = sdo.UpldString( OBJID_AMP_INFO,  2, l=COPLEY_MAX_STRING, info.mfgInfo );
   if( !err ) err = sdo.UpldString( 0x100A,          0, l=COPLEY_MAX_STRING, info.swVer   );
   if( !err ) err = sdo.Upld32( OBJID_AMP_MODES, 0, info.modes       );
   if( !err ) err = sdo.Upld32( OBJID_AMP_INFO,  1, info.serial      );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO,  3, info.crntPeak    );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO,  4, info.crntCont    );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO,  5, info.crntTime    );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO,  6, info.voltMax     );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO,  7, info.voltMin     );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO,  8, info.voltHyst    );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO,  9, info.tempMax     );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO, 10, info.tempHyst    );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO, 11, info.pwmPeriod   );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO, 12, info.servoPeriod );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO, 13, info.type        );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO, 14, info.crntScale   );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO, 15, info.voltScale   );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO, 16, info.refScale    );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO, 17, info.pwm_off     );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO, 18, info.pwm_dbcont  );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO, 19, info.pwm_dbzero  );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO, 20, info.regenPeak   );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO, 21, info.regenCont   );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO, 22, info.regenTime   );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO, 23, info.aencScale   );
   if( !err ) err = sdo.Upld16( OBJID_AMP_INFO, 24, info.swVerNum    );

   // OK, assuming the upload went well I'll just convert some
   // parameters into nice units and return
   if( err ) return err;


   return 0;
}

/***************************************************************************/
/**
  Motor info structure default constructor.  This simply initializes all members
  to legal default values.
  */
/***************************************************************************/
MtrInfo::MtrInfo( void )
{
   mfgName[0] = 0;
   model[0] = 0;

   hasBrake   = false;
   tempSensor = false;
   mtrReverse = false;
   encReverse = false;

   resistance = 100;
   inductance = 100;
   trqPeak = 10;
   trqCont = 10;
   trqConst = 100;
   backEMF = 1;
   inertia = 10;

   type  = 0;
   poles = 2;
   velMax = 1;
   hallType   = 1;
   hallWiring = 0;
   hallOffset = 0;
   stopTime = 0;
   brakeDelay = 0;
   brakeVel = 0;
   encType = 0;
   ctsPerRev = 4000;
   encUnits = 0;
   encRes = 100;
   eleDist = 100000;
   mtrUnits = 0;
   stepsPerRev = 4000;
   encShift = 0;
   ndxDist = 0;

   loadEncType = 0;
   loadEncRes = 0;
   loadEncReverse = false;

   gearRatio = 0x00010001;
   resolverCycles = 1;
   hallVelShift = 1;
}

/***************************************************************************/
/**
  Read the motor information structure from the amplifier.

  @param info A structure that will be filled with the motor info
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetMtrInfo( MtrInfo &info )
{
   int32 l;

   // Upload those parameters that can be mapped directly into the
   // MtrInfo structure with no translation.
   const Error *err = sdo.UpldString( OBJID_MOTOR_MODEL, 0, l=COPLEY_MAX_STRING, info.model );
   if( !err ) err = sdo.UpldString( OBJID_MOTOR_MFG,   0, l=COPLEY_MAX_STRING, info.mfgName );
   if( !err ) err = sdo.Upld16( OBJID_MOTOR_INFO,  1, info.type );
   if( !err ) err = sdo.Upld16( OBJID_MOTOR_INFO,  2, info.poles );
   if( !err ) err = sdo.Upld16( OBJID_MOTOR_INFO,  4, info.hallType );
   if( !err ) err = sdo.Upld16( OBJID_MOTOR_INFO,  5, info.hallWiring );
   if( !err ) err = sdo.Upld16( OBJID_MOTOR_INFO,  6, info.hallOffset );
   if( !err ) err = sdo.Upld16( OBJID_MOTOR_INFO, 17, info.stopTime   );
   if( !err ) err = sdo.Upld16( OBJID_MOTOR_INFO, 18, info.brakeDelay );
   if( !err ) err = sdo.Upld16( OBJID_MOTOR_INFO, 20, info.encType );
   if( !err ) err = sdo.Upld16( OBJID_MOTOR_INFO, 21, info.encUnits );
   if( !err ) err = sdo.Upld32( OBJID_MOTOR_INFO, 23, info.ctsPerRev );
   if( !err ) err = sdo.Upld16( OBJID_MOTOR_INFO, 24, info.encRes );
   if( !err ) err = sdo.Upld32( OBJID_MOTOR_INFO, 25, info.eleDist );
   if( !err ) err = sdo.Upld32( OBJID_MOTOR_INFO, 26, info.ndxDist );
   if( !err ) err = sdo.Upld16( OBJID_MOTOR_INFO, 27, info.mtrUnits );
   if( !err ) err = sdo.Upld16( OBJID_MOTOR_INFO, 28, info.encShift );
   if( !err ) err = sdo.Upld32( OBJID_MOTOR_INFO, 29, info.stepsPerRev );

   // Upload the rest of the parameters to temporary locations
   // and convert them into the units used in the MtrInfo structure.
   int16 wiring, tSense, brake, encDir;
   int32 maxVel, bVel;

   if( !err ) err = sdo.Upld16( OBJID_MOTOR_INFO,  3, wiring   );
   if( !err ) err = sdo.Upld16( OBJID_MOTOR_INFO,  7, info.resistance );
   if( !err ) err = sdo.Upld16( OBJID_MOTOR_INFO,  8, info.inductance );

   if( !err ) err = sdo.Upld32( OBJID_MOTOR_INFO,  9, info.inertia );
   if( !err ) err = sdo.Upld32( OBJID_MOTOR_INFO, 10, info.backEMF );
   if( !err ) err = sdo.Upld32( OBJID_MOTOR_INFO, 11, maxVel   );
   if( !err ) err = sdo.Upld32( OBJID_MOTOR_INFO, 12, info.trqConst );
   if( !err ) err = sdo.Upld32( OBJID_MOTOR_INFO, 13, info.trqPeak  );
   if( !err ) err = sdo.Upld32( OBJID_MOTOR_INFO, 14, info.trqCont  );

   if( !err ) err = sdo.Upld16( OBJID_MOTOR_INFO, 15, tSense   );
   if( !err ) err = sdo.Upld16( OBJID_MOTOR_INFO, 16, brake    );

   if( !err ) err = sdo.Upld32( OBJID_MOTOR_INFO, 19, bVel     );
   if( !err ) err = sdo.Upld16( OBJID_MOTOR_INFO, 22, encDir   );

   if( err ) return err;

   // Now, convert the uploaded values to more convenient units.
   info.mtrReverse = wiring ? true : false;
   info.encReverse = encDir ? true : false;
   info.hasBrake   = brake ? false : true;
   info.tempSensor = tSense ? true : false;

   info.velMax     = VelMtr2User( maxVel );
   info.brakeVel   = VelMtr2User( bVel );

   // Check for some parameters that we added in later versions of firmware.
   uint8 ct;
   err = sdo.Upld8( OBJID_MOTOR_INFO,  0, ct );
   if( !err && (ct >=30) ) err = sdo.Upld16( OBJID_MOTOR_INFO, 30, info.loadEncType );
   if( !err && (ct >=31) ) err = sdo.Upld16( OBJID_MOTOR_INFO, 31, encDir );
   if( !err && (ct >=32) ) err = sdo.Upld32( OBJID_MOTOR_INFO, 32, info.loadEncRes  );
   info.loadEncReverse = encDir ? true : false;

   // Firmware versions before 4.42 had a bug which prevented reading the gear ratio
   // or resolver cycles/rev parameters.
   if( SwVersionNum < VERSION(4,42) )
      return err;

   if( !err && (ct >=33) ) err = sdo.Upld32( OBJID_MOTOR_INFO, 33, info.gearRatio );
   if( !err && (ct >=34) ) err = sdo.Upld16( OBJID_MOTOR_INFO, 34, info.resolverCycles );

   if ((int16)info.resolverCycles <= 0 )
      info.resolverCycles =1;

   if( !err && (SwVersionNum >= VERSION(4,56)) )
      err = sdo.Upld16( OBJID_HALLVEL_SHIFT, 0, info.hallVelShift );

   return err;
}

/***************************************************************************/
/**
  Update the amplifier's motor information.

  @param info A structure that contains the motor info to be downloaded.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetMtrInfo( MtrInfo &info )
{
   // Translate parameters into the proper units for upload.
   int16 wiring, tSense, brake, encDir;
   int32 maxVel, bVel;

   maxVel  = VelUser2Mtr( info.velMax );
   bVel    = VelUser2Mtr( info.brakeVel );

   wiring = (info.mtrReverse == true);
   encDir = (info.encReverse == true);
   brake  = (info.hasBrake   == false);
   tSense = (info.tempSensor == true);

   // Upload these parameters to the motor
   const Error *err = sdo.DnldString( OBJID_MOTOR_MODEL, 0, info.model   );
   if( !err ) err = sdo.DnldString( OBJID_MOTOR_MFG,   0, info.mfgName );
   if( !err ) err = sdo.Dnld16( OBJID_MOTOR_INFO,  1, info.type );
   if( !err ) err = sdo.Dnld16( OBJID_MOTOR_INFO,  2, info.poles );
   if( !err ) err = sdo.Dnld16( OBJID_MOTOR_INFO,  3, wiring   );
   if( !err ) err = sdo.Dnld16( OBJID_MOTOR_INFO,  4, info.hallType );
   if( !err ) err = sdo.Dnld16( OBJID_MOTOR_INFO,  5, info.hallWiring );
   if( !err ) err = sdo.Dnld16( OBJID_MOTOR_INFO,  6, info.hallOffset );
   if( !err ) err = sdo.Dnld16( OBJID_MOTOR_INFO,  7, info.resistance );
   if( !err ) err = sdo.Dnld16( OBJID_MOTOR_INFO,  8, info.inductance );
   if( !err ) err = sdo.Dnld32( OBJID_MOTOR_INFO,  9, info.inertia  );
   if( !err ) err = sdo.Dnld32( OBJID_MOTOR_INFO, 10, info.backEMF );
   if( !err ) err = sdo.Dnld32( OBJID_MOTOR_INFO, 11, maxVel   );
   if( !err ) err = sdo.Dnld32( OBJID_MOTOR_INFO, 12, info.trqConst );
   if( !err ) err = sdo.Dnld32( OBJID_MOTOR_INFO, 13, info.trqPeak  );
   if( !err ) err = sdo.Dnld32( OBJID_MOTOR_INFO, 14, info.trqCont  );
   if( !err ) err = sdo.Dnld16( OBJID_MOTOR_INFO, 15, tSense   );
   if( !err ) err = sdo.Dnld16( OBJID_MOTOR_INFO, 16, brake    );
   if( !err ) err = sdo.Dnld16( OBJID_MOTOR_INFO, 17, info.stopTime   );
   if( !err ) err = sdo.Dnld16( OBJID_MOTOR_INFO, 18, info.brakeDelay );
   if( !err ) err = sdo.Dnld32( OBJID_MOTOR_INFO, 19, bVel     );
   if( !err ) err = sdo.Dnld16( OBJID_MOTOR_INFO, 20, info.encType );
   if( !err ) err = sdo.Dnld16( OBJID_MOTOR_INFO, 21, info.encUnits );
   if( !err ) err = sdo.Dnld16( OBJID_MOTOR_INFO, 22, encDir   );
   if( !err ) err = sdo.Dnld32( OBJID_MOTOR_INFO, 23, info.ctsPerRev );
   if( !err ) err = sdo.Dnld16( OBJID_MOTOR_INFO, 24, info.encRes );
   if( !err ) err = sdo.Dnld32( OBJID_MOTOR_INFO, 25, info.eleDist );
   if( !err ) err = sdo.Dnld32( OBJID_MOTOR_INFO, 26, info.ndxDist );
   if( !err ) err = sdo.Dnld16( OBJID_MOTOR_INFO, 27, info.mtrUnits );
   if( !err ) err = sdo.Dnld16( OBJID_MOTOR_INFO, 28, info.encShift );
   if( !err ) err = sdo.Dnld32( OBJID_MOTOR_INFO, 29, info.stepsPerRev );
   if( err ) return err;

   // Update the load encoder settings if available in the firmware
   uint8 ct;
   err = sdo.Upld8( OBJID_MOTOR_INFO,  0, ct );

   encDir = (info.loadEncReverse == true);
   if( !err && (ct >= 30) ) err = sdo.Dnld16( OBJID_MOTOR_INFO, 30, info.loadEncType );
   if( !err && (ct >= 31) ) err = sdo.Dnld16( OBJID_MOTOR_INFO, 31, encDir );
   if( !err && (ct >= 32) ) err = sdo.Dnld32( OBJID_MOTOR_INFO, 32, info.loadEncRes  );
   if( SwVersionNum < VERSION(4,42) )
      return err;

   if( !err && (ct >= 33) ) err = sdo.Dnld32( OBJID_MOTOR_INFO, 33, info.gearRatio );
   if( !err && (ct >= 34) ) err = sdo.Dnld16( OBJID_MOTOR_INFO, 34, info.resolverCycles );

   if( !err && (SwVersionNum >= VERSION(4,56)) )
      err = sdo.Dnld16( OBJID_HALLVEL_SHIFT, 0, info.hallVelShift );

   return err;
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
   Control loop gains (position, velocity & current)
   */
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Get the configuration values of the amplifiers position loop.
  @param cfg A structure that will be filled with the config info.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetPosLoopConfig( PosLoopConfig &cfg )
{
   const Error *err = sdo.Upld16( OBJID_POSLOOP, 1, cfg.kp );
   if( !err ) err = sdo.Upld16( OBJID_POSLOOP, 2, cfg.kvff );
   if( !err ) err = sdo.Upld16( OBJID_POSLOOP, 3, cfg.kaff );

   // This parameter was added in firmware version 3.30
   if( !err )
   {
      if( SwVersionNum >= VERSION(3,30) )
	 err = sdo.Upld16( OBJID_POSLOOP, 4, cfg.scale );
      else
	 cfg.scale = 100;
   }

   return err;
}

/***************************************************************************/
/**
  Update the amplifier's position loop configuration.
  @param cfg A structure that will be filled with the config info.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetPosLoopConfig( PosLoopConfig &cfg )
{
   const Error *err = sdo.Dnld16( OBJID_POSLOOP, 1, cfg.kp );
   if( !err ) err = sdo.Dnld16( OBJID_POSLOOP, 2, cfg.kvff );
   if( !err ) err = sdo.Dnld16( OBJID_POSLOOP, 3, cfg.kaff );

   // This parameter was added in firmware version 3.30
   if( !err )
   {
      if( SwVersionNum >= VERSION(3,30) )
	 err = sdo.Dnld16( OBJID_POSLOOP, 4, cfg.scale );
   }

   return err;
}

/***************************************************************************/
/**
  Get the configuration values of the amplifiers velocity loop.
  @param cfg A structure that will be filled with the config info.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetVelLoopConfig( VelLoopConfig &cfg )
{
   int32 maxVel, maxAcc, maxDec, eStop;

   const Error *err = sdo.Upld16( OBJID_VELLOOP, 1, cfg.kp );
   if( !err ) err = sdo.Upld16( OBJID_VELLOOP, 2, cfg.ki );
   if( !err ) err = sdo.Upld16( OBJID_VELLOOP, 3, cfg.kaff );
   if( !err ) err = sdo.Upld16( OBJID_VELLOOP, 4, cfg.shift );
   if( !err ) err = sdo.Upld32( OBJID_VELLIM_MAXACC, 0, maxAcc );
   if( !err ) err = sdo.Upld32( OBJID_VELLIM_MAXDEC, 0, maxDec );
   if( !err ) err = sdo.Upld32( OBJID_VELLIM_ESTOP,  0, eStop  );
   if( !err ) err = sdo.Upld32( OBJID_VELLIM_MAXVEL, 0, maxVel );

   // Convert the velocity and accelerations from internal units to user
   // units.  Note that these acceleration values are not in standard
   // amplifier units of 10 cts/sec^2.  Rather, they are in 1000 cts/sec^2.
   // We'll adjust for this here.
   cfg.maxVel = VelMtr2User( maxVel );
   cfg.maxAcc = AccMtr2User( maxAcc * 100 );
   cfg.maxDec = AccMtr2User( maxDec * 100 );
   cfg.estopDec = AccMtr2User( eStop * 100 );

   return err;
}

/***************************************************************************/
/**
  Update the amplifier's velocity loop configuration.
  @param cfg A structure that will be filled with the config info.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetVelLoopConfig( VelLoopConfig &cfg )
{
   // Convert from user unit acceleration and velocity
   // values to internal amplifier units.
   int32 maxAcc   = AccUser2Mtr(cfg.maxAcc);
   int32 maxDec   = AccUser2Mtr(cfg.maxDec);
   int32 estopDec = AccUser2Mtr(cfg.estopDec);
   int32 maxVel   = VelUser2Mtr(cfg.maxVel);

   // Note that these accelerations are not in the standard
   // 10 ct/sec^2 units, but rather in 1000 ct/sec^2 units.
   // I'll adjust for that here.
   maxAcc   = (maxAcc+50) / 100;
   maxDec   = (maxDec+50) / 100;
   estopDec = (estopDec+50) / 100;

   const Error *err = sdo.Dnld16( OBJID_VELLOOP, 1, cfg.kp );
   if( !err ) err = sdo.Dnld16( OBJID_VELLOOP, 2, cfg.ki );
   if( !err ) err = sdo.Dnld16( OBJID_VELLOOP, 3, cfg.kaff );
   if( !err ) err = sdo.Dnld16( OBJID_VELLOOP, 4, cfg.shift );
   if( !err ) err = sdo.Dnld32( OBJID_VELLIM_MAXACC, 0, maxAcc   );
   if( !err ) err = sdo.Dnld32( OBJID_VELLIM_MAXDEC, 0, maxDec   );
   if( !err ) err = sdo.Dnld32( OBJID_VELLIM_ESTOP,  0, estopDec );
   if( !err ) err = sdo.Dnld32( OBJID_VELLIM_MAXVEL, 0, maxVel   );
   return err;
}

/***************************************************************************/
/**
  Get the configuration values of the amplifiers current loop.
  @param cfg A structure that will be filled with the config info.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetCrntLoopConfig( CrntLoopConfig &cfg )
{
   const Error *err = sdo.Upld16( OBJID_CRNTLOOP, 1, cfg.kp );
   if( !err ) err = sdo.Upld16( OBJID_CRNTLOOP, 2, cfg.ki );
   if( !err ) err = sdo.Upld16( OBJID_CRNTLOOP, 3, cfg.offset );
   if( !err ) err = sdo.Upld16( OBJID_CRNTLIM_PEAK, 0, cfg.peakLim );
   if( !err ) err = sdo.Upld16( OBJID_CRNTLIM_CONT, 0, cfg.contLim );
   if( !err ) err = sdo.Upld16( OBJID_CRNTLIM_TIME, 0, cfg.peakTime );

   // the following configuration values were added in firmware version 3.10
   if( SwVersionNum >= VERSION(3,10) )
   {
      // Upload the remaining parameters directly into the output structure
      if( !err ) err = sdo.Upld16( OBJID_USTEP_HOLDCRNT, 0, cfg.stepHoldCurrent  );
      if( !err ) err = sdo.Upld16( OBJID_USTEP_HOLDTIME, 0, cfg.stepRun2HoldTime );
      if( !err ) err = sdo.Upld16( OBJID_USTEP_VOLTIME, 0, cfg.stepVolControlDelayTime );
   }

   if( !err && (SwVersionNum >= VERSION(4,60) ) )
      err = sdo.Upld32( OBJID_CRNT_SLOPE, 0, cfg.slope );
   return err;
}

/***************************************************************************/
/**
  Update the amplifier's current loop configuration.
  @param cfg A structure that will be filled with the config info.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetCrntLoopConfig( CrntLoopConfig &cfg )
{
   const Error *err = sdo.Dnld16( OBJID_CRNTLOOP, 1, cfg.kp );
   if( !err ) err = sdo.Dnld16( OBJID_CRNTLOOP, 2, cfg.ki );
   if( !err ) err = sdo.Dnld16( OBJID_CRNTLOOP, 3, cfg.offset );
   if( !err ) err = sdo.Dnld16( OBJID_CRNTLIM_PEAK, 0, cfg.peakLim );
   if( !err ) err = sdo.Dnld16( OBJID_CRNTLIM_CONT, 0, cfg.contLim );
   if( !err ) err = sdo.Dnld16( OBJID_CRNTLIM_TIME, 0, cfg.peakTime );

   // the following configuration values were added in firmware version 3.10
   if( SwVersionNum >= VERSION(3,10) )
   {
      if( !err ) err = sdo.Dnld16( OBJID_USTEP_HOLDCRNT, 0, cfg.stepHoldCurrent  );
      if( !err ) err = sdo.Dnld16( OBJID_USTEP_HOLDTIME, 0, cfg.stepRun2HoldTime );
      if( !err ) err = sdo.Dnld16( OBJID_USTEP_VOLTIME, 0, cfg.stepVolControlDelayTime );
   }

   if( !err && (SwVersionNum >= VERSION(4,60) ) )
      err = sdo.Dnld32( OBJID_CRNT_SLOPE, 0, cfg.slope );

   return err;
}


/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
   Tracking windows (position & velocity)
   */
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Default constructor for tracking window structure.  This simply sets all
  tracking window parameter default values of zero.
  */
/***************************************************************************/
TrackingWindows::TrackingWindows( void )
{
   trackErr     = 0;
   trackWarn    = 0;
   settlingWin  = 0;
   settlingTime = 0;
   velWarnWin   = 0;
   velWarnTime  = 0;
}

/***************************************************************************/
/**
  Update the amplifier's tracking window configuration.  This function allows
  all tracking window parameters to be configured with one function call.
  @param cfg A structure that holds the configuration settings.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetTrackingWindows( TrackingWindows &cfg )
{
   const Error *err = SetPositionErrorWindow( cfg.trackErr );
   if( !err ) err = SetPositionWarnWindow( cfg.trackWarn );
   if( !err ) err = SetSettlingWindow( cfg.settlingWin );
   if( !err ) err = SetSettlingTime( cfg.settlingTime );
   if( !err ) err = SetVelocityWarnWindow( cfg.velWarnWin );
   if( !err ) err = SetVelocityWarnTime( cfg.velWarnTime );
   return err;
}

/***************************************************************************/
/**
  Get the configuration values of the amplifiers position & velocity 
  tracking windows.  This function allows all tracking window paramters
  to be read from the amplifier as a group.
  @param cfg A structure that will be filled with the config info.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetTrackingWindows( TrackingWindows &cfg )
{
   const Error *err = GetPositionErrorWindow( cfg.trackErr );
   if( !err ) err = GetPositionWarnWindow( cfg.trackWarn );
   if( !err ) err = GetSettlingWindow( cfg.settlingWin );
   if( !err ) err = GetSettlingTime( cfg.settlingTime );
   if( !err ) err = GetVelocityWarnWindow( cfg.velWarnWin );
   if( !err ) err = GetVelocityWarnTime( cfg.velWarnTime );
   return err;
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
   Software limit switch configuration.
   */
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Set software limit switch settings.  The amplifier's software limit settings
  consist of a positive and negative absolute position.  Any time the motors
  actual position is greater then the positive limit, or less then the negative
  limit, a limit event occurs.  Software limit events are treated by the amplifer
  in the same way that physical limit switches are, no current will be output 
  in the direction of the limit switch, and any running trajectory will be 
  aborted.

  Software limit switches are not used until the amplifier has been homed.  Also,
  if the positive software limit is set to a value greater then or equal to the
  negative software limit, then the limits are disabled.

  @param cfg The limit switch settings to use
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::SetSoftLimits( SoftPosLimit &cfg )
{
   int32 p = PosUser2Load( cfg.pos );
   int32 n = PosUser2Load( cfg.neg );
   int32 a = AccUser2Load( cfg.accel );

   const Error *err = sdo.Dnld32( OBJID_SOFTLIM, 1, n );
   if( !err ) err = sdo.Dnld32( OBJID_SOFTLIM, 2, p );

   // Software limit acceleration was added in version 4.60
   if( !err && (SwVersionNum >= VERSION(4,60) ) )
      err = sdo.Dnld32( OBJID_SOFTLIM_ACCEL, 0, a );

   return err;
}

/***************************************************************************/
/**
  Upload the current software limit switch settings from the amplifier
  @param cfg The limit switch settings will be returned in this structure.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::GetSoftLimits( SoftPosLimit &cfg )
{
   int32 p, n;
   int32 a = 0;

   const Error *err = sdo.Upld32( OBJID_SOFTLIM, 1, n );

   if( !err )
      err = sdo.Upld32( OBJID_SOFTLIM, 2, p );

   if( !err && (SwVersionNum >= VERSION(4,60) ) )
      err = sdo.Upld32( OBJID_SOFTLIM_ACCEL, 0, a );

   cfg.pos = PosLoad2User( p );
   cfg.neg = PosLoad2User( n );
   cfg.accel = AccLoad2User( a );

   return err;
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
   Programmable I/O pins
   */
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Default constructor for AmpIoCfg structure.  This simply sets all member
  parameters to default values of zero.
  */
/***************************************************************************/
AmpIoCfg::AmpIoCfg( void )
{
   int i;

   inputCt = 0;
   outputCt = 0;
   inPullUpCfg = 0;

   for( i=0; i<COPLEY_MAX_INPUTS; i++ )
   {
      inCfg[i] = (INPUT_PIN_CONFIG)0;
      inDebounce[i] = 0;
   }

   for( i=0; i<COPLEY_MAX_OUTPUTS; i++ )
   {
      outCfg[i] = (OUTPUT_PIN_CONFIG)0;
      outMask[i] = 0;
	  outMask1[i] = 0;
   }
}

/***************************************************************************/
/**
  Configure the amplifier's programmable I/O pins using the values passed
  in the config structure.

  The inputCt and outputCt values of the config structure should indicate the
  total number of input & output pins to configure.  If the amplifier has more
  pins then these values indicate, the configuration of the remaining amplifier
  pins will not be changed.

  @param cfg A structure that holds the configuration settings.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetIoConfig( AmpIoCfg &cfg )
{
   uint8 inCt, outCt;
   int i;

   const Error *err = sdo.Upld8( OBJID_INPUT_CFG, 0, inCt );
   if( !err ) err = sdo.Upld8( OBJID_OUTPUT_CFG, 0, outCt );
   if( !err ) err = SetIoPullup( cfg.inPullUpCfg );

   if( inCt > cfg.inputCt ) inCt = cfg.inputCt;
   if( outCt > cfg.outputCt ) outCt = cfg.outputCt;

   if( inCt  > COPLEY_MAX_INPUTS  ) inCt  = COPLEY_MAX_INPUTS;
   if( outCt > COPLEY_MAX_OUTPUTS ) outCt = COPLEY_MAX_OUTPUTS;

   for( i=0; i<inCt && !err; i++ )
   {
      err = SetInputConfig( i, cfg.inCfg[i] );
      if( !err ) 
	 err = SetInputDebounce( i, cfg.inDebounce[i] );
   }

   for( i=0; i<outCt && !err; i++ )
      err = SetOutputConfig( i, cfg.outCfg[i], cfg.outMask[i], cfg.outMask1[i] );

   return err;
}


/***************************************************************************/
/**
  Read the amplifier's programmable I/O pin configuration and return it in 
  the passed config structure.

  @param cfg A structure that holds the configuration settings.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetIoConfig( AmpIoCfg &cfg )
{
   int i;

   const Error *err = sdo.Upld8( OBJID_INPUT_CFG, 0, cfg.inputCt );
   if( !err ) err = sdo.Upld8( OBJID_OUTPUT_CFG, 0, cfg.outputCt );
   if( !err ) err = GetIoPullup( cfg.inPullUpCfg );

   if( cfg.inputCt  > COPLEY_MAX_INPUTS  ) cfg.inputCt  = COPLEY_MAX_INPUTS;
   if( cfg.outputCt > COPLEY_MAX_OUTPUTS ) cfg.outputCt = COPLEY_MAX_OUTPUTS;

   for( i=0; i<cfg.inputCt && !err; i++ )
   {
      err = GetInputConfig( i, cfg.inCfg[i] );
      if( !err ) 
	 err = GetInputDebounce( i, cfg.inDebounce[i] );
   }

   for( i=0; i<cfg.outputCt && !err; i++ )
      err = GetOutputConfig( i, cfg.outCfg[i], cfg.outMask[i], cfg.outMask1[i] );

   return err;
}

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/*
   Regen resister configuration
   */
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

/***************************************************************************/
/**
  Download a new configuration structure for the power regeneration resister.

  Note that not all amplifiers support a regen resister.  Please see the 
  amplifier datasheet to determine if this feature is available for the 
  amplifier being used.

  @param cfg A structure containing the configuration parameters to set.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetRegenConfig( RegenConfig &cfg )
{
   const Error *err = sdo.DnldString( OBJID_REGEN_MODEL, 0, cfg.model );
   if( !err ) err = sdo.Dnld16( OBJID_REGEN_TIME, 0, cfg.peakTime   );
   if( !err ) err = sdo.Dnld16( OBJID_REGEN_RES,  0, cfg.resistance );
   if( !err ) err = sdo.Dnld16( OBJID_REGEN_CONT, 0, cfg.contPower );
   if( !err ) err = sdo.Dnld16( OBJID_REGEN_PEAK, 0, cfg.peakPower );
   if( !err ) err = sdo.Dnld16( OBJID_REGEN_VON,  0, cfg.vOn  );
   if( !err ) err = sdo.Dnld16( OBJID_REGEN_VOFF, 0, cfg.vOff );
   return err;
}

/***************************************************************************/
/**
  Upload the current configuration parameters for the power regeneration 
  resister connected to the amplifier.

  Note that not all amplifiers support a regen resister.  Please see the 
  amplifier datasheet to determine if this feature is available for the 
  amplifier being used.

  @param cfg A structure where the configuration parameters will be returned.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetRegenConfig( RegenConfig &cfg )
{
   int32 l;
   const Error *err = sdo.UpldString( OBJID_REGEN_MODEL, 0, l=COPLEY_MAX_STRING, cfg.model );
   if( !err ) err = sdo.Upld16( OBJID_REGEN_TIME, 0, cfg.peakTime   );
   if( !err ) err = sdo.Upld16( OBJID_REGEN_RES,  0, cfg.resistance );
   if( !err ) err = sdo.Upld16( OBJID_REGEN_CONT, 0, cfg.contPower  );
   if( !err ) err = sdo.Upld16( OBJID_REGEN_PEAK, 0, cfg.peakPower  );
   if( !err ) err = sdo.Upld16( OBJID_REGEN_VON,  0, cfg.vOn        );
   if( !err ) err = sdo.Upld16( OBJID_REGEN_VOFF, 0, cfg.vOff       );

   return 0;
}

/***************************************************************************/
/**
  Configure the amplifier's homing related parameters.  The passed structure
  contains all parameters related to performing a home routine.

  @param cfg A structure holding the configuration parameters.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetHomeConfig( HomeConfig &cfg )
{
   const Error *err = SetHomeMethod( cfg.method, cfg.extended );
   if( !err ) err = SetHomeOffset( cfg.offset );
   if( !err ) err = SetHomeVelFast( cfg.velFast );
   if( !err ) err = SetHomeVelSlow( cfg.velSlow );
   if( !err ) err = SetHomeAccel( cfg.accel );
   if( !err ) err = SetHomeCurrent( cfg.current );
   if( !err ) err = SetHomeDelay( cfg.delay );

   return err;
}

/***************************************************************************/
/**
  Load a structure with all parameters related to homing the amplifier.
  @param cfg A structure where the configuration parameters will be returned.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetHomeConfig( HomeConfig &cfg )
{
   const Error *err = GetHomeMethod( cfg.method, &cfg.extended );
   if( !err ) err = GetHomeOffset( cfg.offset );
   if( !err ) err = GetHomeVelFast( cfg.velFast );
   if( !err ) err = GetHomeVelSlow( cfg.velSlow );
   if( !err ) err = GetHomeAccel( cfg.accel );
   if( !err ) err = GetHomeCurrent( cfg.current );
   if( !err ) err = GetHomeDelay( cfg.delay );
   return err;
}

/***************************************************************************/
/**
  Configure the amplifier's parameters related to point-to-point moves.

  @param cfg A structure holding the configuration parameters.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetProfileConfig( ProfileConfig &cfg )
{
   const Error *err = SetProfileType( cfg.type );
   if( !err ) err = SetTargetPos( cfg.pos );
   if( !err ) err = SetProfileVel( cfg.vel ); 
   if( !err ) err = SetProfileAcc( cfg.acc ); 
   if( !err ) err = SetProfileDec( cfg.dec );
   if( !err ) err = SetProfileJerk( cfg.jrk );
   if( !err ) err = SetQuickStopDec( cfg.abort );
   return err;
}

/***************************************************************************/
/**
  Load a structure with all parameters related to point-to-point moves.
  @param cfg A structure where the configuration parameters will be returned.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetProfileConfig( ProfileConfig &cfg )
{
   const Error *err = GetProfileType( cfg.type );
   if( !err ) err = GetTargetPos( cfg.pos );
   if( !err ) err = GetProfileVel( cfg.vel ); 
   if( !err ) err = GetProfileAcc( cfg.acc ); 
   if( !err ) err = GetProfileDec( cfg.dec );
   if( !err ) err = GetProfileJerk( cfg.jrk );
   if( !err ) err = GetQuickStopDec( cfg.abort );
   return err;
}

/***************************************************************************/
/**
  Configure the amplifier's internal function generator.
  @param cfg A structure holding the configuration to download
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetFuncGenConfig( FuncGenConfig &cfg )
{
   const Error *err = sdo.Dnld16( OBJID_FGEN_CFG, 0, cfg.cfg );
   if( !err ) err = sdo.Dnld16( OBJID_FGEN_FREQ, 0, cfg.freq );
   if( !err ) err = sdo.Dnld32( OBJID_FGEN_AMP,  0, cfg.amp  );
   if( !err ) err = sdo.Dnld16( OBJID_FGEN_DUTY, 0, cfg.duty );
   return err;
}

/***************************************************************************/
/**
  Upload the current configuration of the amplifier's internal function generator.
  @param cfg A structure where the configuration will be returned.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetFuncGenConfig( FuncGenConfig &cfg )
{
   const Error *err = sdo.Upld16( OBJID_FGEN_CFG, 0, cfg.cfg );
   if( !err ) err = sdo.Upld16( OBJID_FGEN_FREQ, 0, cfg.freq );
   if( !err ) err = sdo.Upld32( OBJID_FGEN_AMP,  0, cfg.amp  );
   if( !err ) err = sdo.Upld16( OBJID_FGEN_DUTY, 0, cfg.duty );
   return err;
}

/***************************************************************************/
/**
  Configure the amplifier's analog reference input.  Note that some amplifier
  models do not support the analog reference.
  @param cfg A structure holding the configuration to download
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetAnalogRefConfig( AnalogRefConfig &cfg )
{
   const Error *err = sdo.Dnld32( OBJID_REF_SCALE, 0, cfg.scale );
   if( !err ) err = sdo.Dnld16( OBJID_REF_OFFSET, 0, cfg.offset );
   if( !err ) err = sdo.Dnld16( OBJID_REF_CALOFF, 0, cfg.calibration );
   if( !err ) err = sdo.Dnld16( OBJID_REF_DEADBAND, 0, cfg.deadband );
   return err;
}

/***************************************************************************/
/**
  Upload the amplifier's analog reference input configuration.
  @param cfg A structure where the configuration parameters will be returned.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetAnalogRefConfig( AnalogRefConfig &cfg )
{
   const Error *err = sdo.Upld32( OBJID_REF_SCALE, 0, cfg.scale );
   if( !err ) err = sdo.Upld16( OBJID_REF_OFFSET, 0, cfg.offset );
   if( !err ) err = sdo.Upld16( OBJID_REF_CALOFF, 0, cfg.calibration );
   if( !err ) err = sdo.Upld16( OBJID_REF_DEADBAND, 0, cfg.deadband );
   return err;
}

/***************************************************************************/
/**
  Configure the amplifier's PWM input pins.  Note that these settings are only
  used when the amplifier is controlled by it's PWM (or pulse/direction) input
  pins, i.e. not in CANopen mode.
  @param cfg A structure holding the configuration to download
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetPwmInConfig( PwmInConfig &cfg )
{
   const Error *err = sdo.Dnld16( OBJID_PWMIN_CFG, 0, cfg.cfg );
   if( !err ) err = sdo.Dnld32( OBJID_PWMIN_SCALE, 0, cfg.scale );

   // PWM input frequency was added to CANopen interface in version 4.56
   if( !err && (SwVersionNum >= VERSION(4,56) ) )
      err = sdo.Dnld16( OBJID_PWMIN_FREQ, 0, cfg.freq );
   return err;
}

/***************************************************************************/
/**
  Upload the amplifier's PWM input pin configuration.
  @param cfg A structure where the configuration parameters will be returned.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetPwmInConfig( PwmInConfig &cfg )
{
   const Error *err = sdo.Upld16( OBJID_PWMIN_CFG, 0, cfg.cfg );
   if( !err ) err = sdo.Upld32( OBJID_PWMIN_SCALE, 0, cfg.scale );

   if( !err && (SwVersionNum >= VERSION(4,56) ) )
      err = sdo.Upld16( OBJID_PWMIN_FREQ, 0, cfg.freq );
   return err;
}

/***************************************************************************/
/**
  Set the CANopen node ID and bit rate configuration.  Note that the amplifier
  only uses this parameter at startup or after a reset.  
  @param cfg Structure holding the configuration to set.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetCanNetworkConfig( CanNetworkConfig &cfg )
{
   uint16 value = cfg.ToAmpFormat();
   return sdo.Dnld16( OBJID_CANID_CFG, 0, value );
}

/***************************************************************************/
/**
  Get the current CANopen network configuration programmed into the amplifier.
  @param cfg A structure where the configuration parameters will be returned.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetCanNetworkConfig( CanNetworkConfig &cfg )
{
   uint16 value;
   const Error *err = sdo.Upld16( OBJID_CANID_CFG, 0, value );
   if( err ) return err;

   cfg.FromAmpFormat( value );
   return 0;
}

/***************************************************************************/
/**
  Load the structure from a 16-bit word.
  @param a A 16-bit value encoding the network configuration.  See the amplifier
  documentation for details on the format.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
void CanNetworkConfig::FromAmpFormat( uint16 a )
{
   offset    = a & 0x7F;
   numInPins = (a>>8) & 7;
   useSwitch = ((a & 0x0800) == 0x0800);
   bitRate   = (CAN_BIT_RATE)(a & 0xF000);
}

/***************************************************************************/
/**
  Encode the contents of the structure into a 16-bit word in the format used
  by the amplifier.  See the amplifier documentation for details on this format.
  @return A 16-bit word representing the contents of this structure.
  */
/***************************************************************************/
uint16 CanNetworkConfig::ToAmpFormat( void )
{
   uint16 config = offset & 0x7F;

   config |= (uint16)(numInPins & 7) << 8;

   if( useSwitch ) config |= 0x0800;

   config |= (uint16)bitRate & 0xF000;
   return config;
}

/***************************************************************************/
/**
  Set new coefficients for the velocity loop output filter.
  @param f A structure holding the filter coefficients
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetVloopOutputFilter( Filter &f )
{
   byte data[18];

   f.toBytes( data );

   return sdo.Download( OBJID_VEL_OUTFILT, 0, 18, data );
}

/***************************************************************************/
/**
  Get the coefficients used in the velocity loop output filter.
  @param f A structure where the filter coefficients will be returned
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetVloopOutputFilter( Filter &f )
{
   byte data[18];
   int32 ct = 18;
   const Error *err = sdo.Upload( OBJID_VEL_OUTFILT, 0, ct, data );

   if( !err )
      f.fromBytes(data);

   return err;
}

/***************************************************************************/
/**
  Set new coefficients for the velocity loop command filter.
  @param f A structure holding the filter coefficients
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetVloopCommandFilter( Filter &f )
{
   byte data[18];
   f.toBytes( data );
   return sdo.Download( OBJID_VEL_CMDFILT, 0, 18, data );
}

/***************************************************************************/
/**
  Get the coefficients used in the velocity loop command filter.
  @param f A structure where the filter coefficients will be returned
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetVloopCommandFilter( Filter &f )
{
   byte data[18];
   int32 ct = 18;
   const Error *err = sdo.Upload( OBJID_VEL_CMDFILT, 0, ct, data );

   if( !err )
      f.fromBytes(data);

   return err;
}

/***************************************************************************/
/**
  Read the complete amplifier configuration from the amplifier and return it
  in the passed structure.  This structure holds every amplifier parameter that
  can be stored to the amplifier's internal flash memory.  The contents of the
  structure represent the complete amplifier configuration.
  @param cfg The structure which will hold the uploaded configuration.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::GetAmpConfig( AmpConfig &cfg )
{
   int32 l;

   const Error *err = sdo.UpldString( OBJID_AMP_NAME, 0, l=COPLEY_MAX_STRING, cfg.name );
   if( !err ) err = sdo.Upload( OBJID_CME2_CONFIG, 0, l=COPLEY_MAX_STRING, cfg.CME_Config );

   if( !err ) err = sdo.Upld32( OBJID_MISC_OPTIONS, 0, cfg.options );
   if( !err ) err = sdo.Upld16( OBJID_CAP_CTRL, 0, cfg.capCtrl );
   if( !err ) err = sdo.Upld16( OBJID_ENCOUT_CONFIG, 0, cfg.encoderOutCfg );
   if( !err ) err = sdo.Upld32( OBJID_CANMASK_LIMIT, 0, l );
   cfg.limitBitMask = (EVENT_STATUS)l;

   if( !err ) err = GetVelocityProgrammed( cfg.progVel );
   if( !err ) err = GetCurrentProgrammed( cfg.progCrnt );
   if( !err ) err = GetFaultMask( cfg.faultMask );
   if( !err ) err = GetAmpMode( cfg.controlMode );
   if( !err ) err = GetPwmMode( cfg.pwmMode );
   if( !err ) err = GetPhaseMode( cfg.phaseMode );
   if( !err ) err = GetMicrostepRate( cfg.stepRate );

   if( !err ) err = GetCanNetworkConfig( cfg.can );
   if( !err ) err = GetPosLoopConfig( cfg.pLoop );
   if( !err ) err = GetVelLoopConfig( cfg.vLoop );
   if( !err ) err = GetCrntLoopConfig( cfg.cLoop );
   if( !err ) err = GetMtrInfo( cfg.motor );
   if( !err ) err = GetTrackingWindows( cfg.window );
   if( !err ) err = GetSoftLimits( cfg.limit );
   if( !err ) err = GetIoConfig( cfg.io );
   if( !err ) err = GetHomeConfig( cfg.home );
   if( !err ) err = GetProfileConfig( cfg.profile );
   if( !err ) err = GetAnalogRefConfig( cfg.ref );
   if( !err ) err = GetPwmInConfig( cfg.pwmIn );
   if( !err ) err = GetFuncGenConfig( cfg.fgen );
   if( !err ) err = GetRegenConfig( cfg.regen );
   if( !err ) err = GetVloopOutputFilter( cfg.vloopOutFltr );

   if( !err && (SwVersionNum >= VERSION(4,56) ) )
      err = GetVloopCommandFilter( cfg.vloopCmdFltr );

   return err;
}

/***************************************************************************/
/**
  Update an amplifier's configuration from the passed structure.
  The AmpConfig structure holds all amplifier parameters that may be stored
  in the amplifier's non-volatile flash memory.  This function may be used
  to update all of these parameters in a single call.

  Note that this function updates the copies of these variables in working
  RAM, not directly in the amplifier flash memory.  To copy these parameters
  to non-volatile memory, call Amp::SaveAmpConfig after updating them.

  @param cfg The structure which holds the new configuration.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SetAmpConfig( AmpConfig &cfg )
{
   const Error *err = sdo.DnldString( OBJID_AMP_NAME, 0, cfg.name );
   if( !err ) err = sdo.Download( OBJID_CME2_CONFIG, 0, COPLEY_MAX_STRING-1, cfg.CME_Config );

   if( !err ) err = sdo.Dnld32( OBJID_MISC_OPTIONS, 0, cfg.options );
   if( !err ) err = sdo.Dnld16( OBJID_CAP_CTRL, 0, cfg.capCtrl );
   if( !err ) err = sdo.Dnld32( OBJID_CANMASK_LIMIT, 0, (uint32)cfg.limitBitMask );
   if( !err ) err = sdo.Dnld16( OBJID_ENCOUT_CONFIG, 0, cfg.encoderOutCfg );

   if( !err ) err = SetVelocityProgrammed( cfg.progVel );
   if( !err ) err = SetCurrentProgrammed( cfg.progCrnt );
   if( !err ) err = SetFaultMask( cfg.faultMask );
   if( !err ) err = SetAmpMode( cfg.controlMode );
   if( !err ) err = SetPwmMode( cfg.pwmMode );
   if( !err ) err = SetPhaseMode( cfg.phaseMode );
   if( !err ) err = SetMicrostepRate( cfg.stepRate );

   if( !err ) err = SetCanNetworkConfig( cfg.can );
   if( !err ) err = SetPosLoopConfig( cfg.pLoop );
   if( !err ) err = SetVelLoopConfig( cfg.vLoop );
   if( !err ) err = SetCrntLoopConfig( cfg.cLoop );
   if( !err ) err = SetMtrInfo( cfg.motor );
   if( !err ) err = SetTrackingWindows( cfg.window );
   if( !err ) err = SetSoftLimits( cfg.limit );
   if( !err ) err = SetIoConfig( cfg.io );
   if( !err ) err = SetHomeConfig( cfg.home );
   if( !err ) err = SetProfileConfig( cfg.profile );
   if( !err ) err = SetAnalogRefConfig( cfg.ref );
   if( !err ) err = SetPwmInConfig( cfg.pwmIn );
   if( !err ) err = SetFuncGenConfig( cfg.fgen );
   if( !err ) err = SetRegenConfig( cfg.regen );
   if( !err ) err = SetVloopOutputFilter( cfg.vloopOutFltr );

   if( !err && (SwVersionNum >= VERSION(4,56) ) )
      err = SetVloopCommandFilter( cfg.vloopCmdFltr );

   return err;
}

/***************************************************************************/
/**
  Save all amplifier parameters to internal flash memory.  Flash memory is a 
  type of non-volatile RAM which allows amplifier parameters to be saved 
  between power cycles.  When this function is called, any amplifier parameters
  that may be stored to flash will be copied from their working (RAM) locations
  to the stored (flash) locations.

  For a list of those amplifier parameters which may be saved to flash memory,
  see the AmpConfig structure.  Every member of that structure represents an
  amplifier parameter that may be saved to flash.

  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SaveAmpConfig( void )
{
   return sdo.Download( 0x1010, 1, 4, "save" );
}

/***************************************************************************/
/**
  Upload the passed amplifier configuration to the amplifier's workign memory,
  and then copy that working memory to flash.

  @param cfg The structure which holds the new configuration.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Amp::SaveAmpConfig( AmpConfig &cfg )
{
   const Error *err = SetAmpConfig( cfg );
   if( !err ) err = SaveAmpConfig();
   return err;
}

