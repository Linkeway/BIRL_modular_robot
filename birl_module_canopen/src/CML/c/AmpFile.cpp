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
  This file contains code used to read a CME-2 .ccx amplifier file.
  */
/***************************************************************************/

#include "CML.h"

#ifdef CML_FILE_ACCESS_OK
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#endif


CML_NAMESPACE_USE();

CML_NEW_ERROR( AmpFileError, format,        "Amplifier file format error" );
CML_NEW_ERROR( AmpFileError, tooOld,        "Amplifier file format is too old, use CME version 3.1 or later" );
CML_NEW_ERROR( AmpFileError, noFileAccess,  "File access was not enabled at compile time.  See CML_Settings.h" );
CML_NEW_ERROR( AmpFileError, fileOpen,      "Error opening amplifier file" );
CML_NEW_ERROR( AmpFileError, range,         "A parameter in the amplifier file is out of range" );
CML_NEW_ERROR( AmpFileError, axis,          "Amplifier file is for multi axis, not supported" );

// Local functions
#ifdef CML_FILE_ACCESS_OK
static const Error *ReadLine( FILE *fp, char *buff, int max );
static int SplitLine( char *buff, char **seg, int max, char delim=',' );
static const Error *StrToInt32( char *str, int32 &i, int base=0 );
static const Error *StrToUInt32( char *str, uint32 &i, int base=0 );
static const Error *StrToInt16( char *str, int16 &i, int base=0 );
static const Error *StrToUInt16( char *str, uint16 &i, int base=0 );
static const Error *StrToOutCfg( char *str, OUTPUT_PIN_CONFIG &cfg, uint32 &mask1, uint32 &mask2 );
static const Error *StrToFilter( char *str, Filter &flt );
static const Error *StrToHostCfg( char *str, char *hostCfg );
static COPLEY_HOME_METHOD HomeMethodConvert( uint16 x );
#endif

// Handy macros
#define StrToLoadPos( str, pos )  { int32 i32; err=StrToInt32( str, i32 ); pos = PosLoad2User(i32); }
#define StrToLoadVel( str, vel )  { int32 i32; err=StrToInt32( str, i32 ); vel = VelLoad2User(i32); }
#define StrToLoadAcc( str, acc )  { int32 i32; err=StrToInt32( str, i32 ); acc = AccLoad2User(i32); }
#define StrToLoadJrk( str, jrk )  { int32 i32; err=StrToInt32( str, i32 ); jrk = JrkLoad2User(i32); }
#define StrToMtrPos( str, pos )  { int32 i32; err=StrToInt32( str, i32 ); pos = PosMtr2User(i32); }
#define StrToMtrVel( str, vel )  { int32 i32; err=StrToInt32( str, i32 ); vel = VelMtr2User(i32); }
#define StrToMtrAcc( str, acc )  { int32 i32; err=StrToInt32( str, i32 ); acc = AccMtr2User(i32); }

/***************************************************************************/
/**
  Load the specified amplifier data file.  This function presently supports
  loading *.ccx files created by the CME-2 program, version 3.1 and later.
  @param name The name (and optionally path) of the file to load
  @param line If not NULL, the last line number read from the file is returned
  here.  This is useful for finding file format errors.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::LoadFromFile( const char *name, int &line )
{
#define MAX_LINE_LEN  200
#define MAX_LINE_SEGS 4

   line = 0;
#ifndef CML_FILE_ACCESS_OK
   return &AmpFileError::noFileAccess;
#else
   AmpConfig cfg;

   // Load the configuration structure with the current amplifier
   // configuration data.  This ensures that any parameters not 
   // specified in the file will remain unchanged.
   const Error *err = GetAmpConfig( cfg );
   if( err ) return err;

   // Open the file and read each parameter into my configuration
   // structure.
   FILE *fp;

   fp = fopen( name, "rt" );
   if( !fp )
      return &AmpFileError::fileOpen;

   char buff[MAX_LINE_LEN];
   char *seg[MAX_LINE_SEGS];
   int ct;
   int numOfSegs = 3;
   int segIndex = 2;

   int16 fileRev, mtrFamily;
   int16 i16;
   uint16 u16;
   int32 i32;

   // Read file version number
   line++;
   ReadLine( fp, buff, MAX_LINE_LEN );
   SplitLine( buff, seg, MAX_LINE_SEGS );
   err = StrToInt16( seg[0], fileRev, 10 );
   if( err ) return err;

   if( fileRev < 9 ) 
      return &AmpFileError::tooOld;

   line++;
   ReadLine( fp, buff, MAX_LINE_LEN );
   SplitLine( buff, seg, MAX_LINE_SEGS );

   if(fileRev < 12){
      // Read the motor family info
      err = StrToInt16( seg[0], mtrFamily, 10 );
      if( err ) return err;
   }

   if(fileRev == 12){
      // Read the motor family info
      SplitLine( seg[2], seg, 4, ':' );
      err = StrToInt16( seg[0], mtrFamily, 16 );
      if( err ) return err;
   }

   if (fileRev >= 13)
   {
      numOfSegs = 4;
      segIndex = 3;

      // read the number of axis
      err = StrToInt16( seg[0], i16, 10 );
      if( err ) return err;

      // only capable of 1 axis
      if( i16 > 1 ) 
	 return &AmpFileError::axis;

      // Just initialize motor family to zero.
      // This will be updated below when the host 
      // config string is read from the file
      mtrFamily = 0;
   }

   if( mtrFamily < 0 || mtrFamily > 2 ) 
      return &AmpFileError::format;

   cfg.CME_Config[4] = (char)(mtrFamily>>8);
   cfg.CME_Config[5] = (char)(mtrFamily);

   // Read all parameters
   while( !feof(fp) && !err )
   {
      int16 param;

      line++;
      ReadLine( fp, buff, MAX_LINE_LEN );

      ct = SplitLine( buff, seg, MAX_LINE_SEGS );
      if( ct == 0 )
	 continue;

      if( ct != numOfSegs )
	 err = &AmpFileError::format;
      else
	 err = StrToInt16( seg[0], param, 16 );

      if( err ) break;

      switch( param )
      {
	 case 0x000: err = StrToInt16( seg[segIndex], cfg.cLoop.kp ); break;
	 case 0x001: err = StrToInt16( seg[segIndex], cfg.cLoop.ki ); break;
	 case 0x002: err = StrToInt16( seg[segIndex], cfg.progCrnt ); break;

	 case 0x019: err = StrToInt32( seg[segIndex], cfg.ref.scale  ); break;
	 case 0x01a: err = StrToInt16( seg[segIndex], cfg.ref.offset ); break;

	 case 0x021: err = StrToInt16( seg[segIndex], cfg.cLoop.peakLim ); break;
	 case 0x022: err = StrToInt16( seg[segIndex], cfg.cLoop.contLim ); break;
	 case 0x023: err = StrToInt16( seg[segIndex], cfg.cLoop.peakTime ); break;

	 case 0x024:
	    err = StrToInt16( seg[segIndex], i16 );
	    cfg.controlMode = (AMP_MODE)(i16<<8);

	    // The ccx file only holds the amplifier control method.
	    // Build a proper amp mode out of this if it's one of the
	    // CANopen control methods.
	    if( (cfg.controlMode==AMPMODE_CAN_SERVO) || 
		  (cfg.controlMode==AMPMODE_CAN_USTEP) )
	       cfg.controlMode = (AMP_MODE)(cfg.controlMode | AMPMODE_CAN_PROFILE);
	    break;

	 case 0x026: err = StrToInt16( seg[segIndex], cfg.ref.deadband ); break;
	 case 0x027: err = StrToInt16( seg[segIndex], cfg.vLoop.kp ); break;
	 case 0x028: err = StrToInt16( seg[segIndex], cfg.vLoop.ki ); break;
	 case 0x02e: err = StrToInt16( seg[segIndex], cfg.vLoop.kaff ); break;
	 case 0x02f: StrToMtrVel( seg[segIndex], cfg.progVel ); break;
	 case 0x030: err = StrToInt16( seg[segIndex], cfg.pLoop.kp    ); break;
	 case 0x031: err = StrToInt16( seg[segIndex], cfg.vLoop.shift ); break;
	 case 0x033: err = StrToInt16( seg[segIndex], cfg.pLoop.kvff  ); break;
	 case 0x034: err = StrToInt16( seg[segIndex], cfg.pLoop.kaff  ); break;

	 // These three parameters use non-standard units in the amplifier
	 // thus the multiplication after converting them to user units.
	 case 0x036:
	    StrToMtrAcc( seg[segIndex], cfg.vLoop.maxAcc ); 
	    cfg.vLoop.maxAcc *= 100;
	    break;

	 case 0x037: 
	    StrToMtrAcc( seg[segIndex], cfg.vLoop.maxDec ); 
	    cfg.vLoop.maxDec *= 100;
	    break;

	 case 0x039: 
	    StrToMtrAcc( seg[segIndex], cfg.vLoop.estopDec ); 
	    cfg.vLoop.estopDec *= 100;
	    break;

	 case 0x03a: StrToMtrVel( seg[segIndex], cfg.vLoop.maxVel ); break;
	 case 0x03e: StrToMtrVel( seg[segIndex], cfg.window.velWarnWin ); break;
	 case 0x03f: err = StrToUInt16( seg[segIndex], cfg.window.velWarnTime ); break;

	 case 0x040: err = StrToUInt16( seg[segIndex], cfg.motor.type ); break;
	 case 0x041: strncpy( cfg.motor.mfgName, seg[segIndex], COPLEY_MAX_STRING ); break;
	 case 0x042: strncpy( cfg.motor.model,   seg[segIndex], COPLEY_MAX_STRING ); break;
	 case 0x043:
	    if( !strcmp( seg[segIndex], "Metric"  ) ) 
	       cfg.motor.mtrUnits = 0;
	    else if( !strcmp( seg[segIndex], "English"  ) ) 
	       cfg.motor.mtrUnits = 1;
	    else
	       err = StrToInt16( seg[segIndex], cfg.motor.mtrUnits  );
	    break;

	 case 0x044: 
	    err = StrToUInt32( seg[segIndex], cfg.motor.inertia ); 
	    break;

	 case 0x045: 
	    err = StrToInt16( seg[segIndex], cfg.motor.poles ); 
	    break;

	 case 0x046:
	    if( !strcmp( seg[segIndex], "No" ) ) 
	       cfg.motor.hasBrake = false;

	    else if( !strcmp( seg[segIndex], "Yes" ) ) 
	       cfg.motor.hasBrake = true;

	    else
	    {
	       err = StrToInt16( seg[segIndex], i16 );
	       cfg.motor.hasBrake = (i16==0);
	    }
	    break;

	 case 0x048: err = StrToUInt32( seg[segIndex], cfg.motor.trqConst ); break;
	 case 0x049: err = StrToUInt16( seg[segIndex], cfg.motor.resistance ); break;
	 case 0x04a: err = StrToUInt16( seg[segIndex], cfg.motor.inductance ); break; 
	 case 0x04b: err = StrToUInt32( seg[segIndex], cfg.motor.trqPeak ); break;
	 case 0x04c: err = StrToUInt32( seg[segIndex], cfg.motor.trqCont ); break;

	 case 0x04d:
	    StrToMtrVel( seg[segIndex], cfg.motor.velMax );
	    break;

	 case 0x04e: err = StrToInt16( seg[segIndex], i16 ); cfg.motor.mtrReverse = (i16!=0); break; 
	 case 0x04f: err = StrToInt16( seg[segIndex], cfg.motor.hallOffset ); break;

	 case 0x050: 
	    if( !strcmp( seg[segIndex], "None"  ) ) cfg.motor.hallType = 0;
	    else if( !strcmp( seg[segIndex], "Digital"  ) ) cfg.motor.hallType = 1;
	    else if( !strcmp( seg[segIndex], "Analog"   ) ) cfg.motor.hallType = 2;
	    else
	       err = StrToInt16( seg[segIndex], cfg.motor.hallType   );
	    break;

	 case 0x052: 
	    err = StrToInt16( seg[segIndex], cfg.motor.hallWiring ); 
	    break;

	 case 0x053: 
	    err = StrToInt16( seg[segIndex], cfg.motor.stopTime ); 
	    break;

	 case 0x054: 
	    err = StrToInt16( seg[segIndex], cfg.motor.brakeDelay ); 
	    break;

	 case 0x055: 
	    StrToMtrVel( seg[segIndex], cfg.motor.brakeVel );
	    break;

	 case 0x056: 
	    err = StrToUInt32( seg[segIndex], cfg.motor.backEMF ); 
	    break;

	 case 0x057:
	    err = StrToInt32( seg[segIndex], cfg.motor.stepsPerRev );
	    break;

	 case 0x058:
	    err = StrToInt32( seg[segIndex], cfg.motor.gearRatio );
	    break;

	 case 0x059:
	    err = StrToInt16( seg[segIndex], cfg.motor.hallVelShift );
	    break;

	 case 0x5A:
	    err = StrToUInt16( seg[segIndex], cfg.encoderOutCfg );
	    break;

	 case 0x05B:
	    err = StrToInt32( seg[segIndex], cfg.motor.loadEncRes );
	    break;

	 case 0x05C:
	    err = StrToInt16( seg[segIndex], i16 ); 
	    cfg.motor.loadEncReverse = (i16!=0);
	    break;

	 case 0x05D:
	    err = StrToInt16( seg[segIndex], cfg.motor.loadEncType );
	    break;

	 case 0x05f: 
	    err = StrToFilter( seg[segIndex], cfg.vloopOutFltr );
	    break;

	 case 0x060:
	    if( !strcmp( seg[segIndex], "Incremental"  ) ) cfg.motor.encType = 0;
	    else if( !strcmp( seg[segIndex], "None"    ) ) cfg.motor.encType = 1;
	    else if( !strcmp( seg[segIndex], "Analog"  ) ) cfg.motor.encType = 2;
	    else if( !strcmp( seg[segIndex], "Absolute") ) cfg.motor.encType = 3;
	    else
	       err = StrToInt16( seg[segIndex], cfg.motor.encType );
	    break;

	 case 0x061: err = StrToInt16( seg[segIndex], cfg.motor.encUnits    ); break;
	 case 0x062: err = StrToInt32( seg[segIndex], cfg.motor.ctsPerRev   ); break;
	 case 0x063: err = StrToInt16( seg[segIndex], cfg.motor.encRes      ); break;
	 case 0x064: err = StrToInt32( seg[segIndex], cfg.motor.eleDist     ); break;

	 case 0x065: 
	    err = StrToInt16( seg[segIndex], i16 ); 
	    cfg.motor.encReverse = (i16!=0); 
	    break; 

	 case 0x066:
	    err = StrToInt32( seg[segIndex], cfg.motor.ndxDist );
	    break;

	 case 0x067:
	    err = StrToInt16( seg[segIndex], cfg.motor.encShift );
	    break;

	 case 0x06A:
	    err = StrToInt32( seg[segIndex], cfg.cLoop.slope );
	    break;

	 case 0x06B:
	    err = StrToFilter( seg[segIndex], cfg.vloopCmdFltr );
	    break;

	 case 0x06e:
	    err = StrToUInt16( seg[segIndex], cfg.motor.resolverCycles );
	    break;

	 case 0x06f:
	    err = StrToInt16( seg[segIndex], i16 ); 
	    cfg.pwmMode = (AMP_PWM_MODE)i16; 
	    break;

	 case 0x070:
	    err = StrToOutCfg( seg[segIndex], cfg.io.outCfg[0], cfg.io.outMask[0], cfg.io.outMask1[0]  );
	    break;

	 case 0x071:
	    err = StrToOutCfg( seg[segIndex], cfg.io.outCfg[1], cfg.io.outMask[1], cfg.io.outMask1[1]  );
	    break;

	 case 0x072:
	    err = StrToOutCfg( seg[segIndex], cfg.io.outCfg[2], cfg.io.outMask[2], cfg.io.outMask1[2]  );
	    break;

	 case 0x073: 
	    err = StrToOutCfg( seg[segIndex], cfg.io.outCfg[3], cfg.io.outMask[3], cfg.io.outMask1[3]  );
	    break;

	 case 0x078:
	    err = StrToInt16( seg[segIndex], i16 );
	    cfg.io.inCfg[ 0] = (INPUT_PIN_CONFIG)i16;
	    break;

	 case 0x079:
	    err = StrToInt16( seg[segIndex], i16 );
	    cfg.io.inCfg[ 1] = (INPUT_PIN_CONFIG)i16;
	    break;

	 case 0x07a: 
	    err = StrToInt16( seg[segIndex], i16 );
	    cfg.io.inCfg[ 2] = (INPUT_PIN_CONFIG)i16;
	    break;

	 case 0x07b:
	    err = StrToInt16( seg[segIndex], i16 );
	    cfg.io.inCfg[ 3] = (INPUT_PIN_CONFIG)i16;
	    break;

	 case 0x07c: 
	    err = StrToInt16( seg[segIndex], i16 );
	    cfg.io.inCfg[ 4] = (INPUT_PIN_CONFIG)i16;
	    break;

	 case 0x07d:
	    err = StrToInt16( seg[segIndex], i16 );
	    cfg.io.inCfg[ 5] = (INPUT_PIN_CONFIG)i16;
	    break;

	 case 0x07e:
	    err = StrToInt16( seg[segIndex], i16 );
	    cfg.io.inCfg[ 6] = (INPUT_PIN_CONFIG)i16;
	    break;

	 case 0x07f: 
	    err = StrToInt16( seg[segIndex], i16 );
	    cfg.io.inCfg[ 7] = (INPUT_PIN_CONFIG)i16;
	    break;

	 // This group of parameters are in the ccx file
	 // for reference only.  They describe read-only
	 // parameters of the amplifier.
	 case 0x80: case 0x81: case 0x82: case 0x83:
	 case 0x84: case 0x85: case 0x86: case 0x87:
	 case 0x88: case 0x89: case 0x8A: case 0x8B:
	 case 0x8C: case 0x8D: case 0x8E: case 0xAD:
	    break;

	 case 0x092: 
	    strncpy( cfg.name, seg[segIndex], COPLEY_MAX_STRING );
	    break;

	 case 0x093: 
	    err = StrToInt16( seg[segIndex], i16 );
	    cfg.CME_Config[0] = (char)(i16>>8);
	    cfg.CME_Config[1] = (char)(i16);
	    break;

	 case 0x095:
	    err = StrToHostCfg( seg[segIndex], cfg.CME_Config );
	    break;

	 case 0x098: err = StrToInt16( seg[segIndex], cfg.fgen.cfg  ); break;
	 case 0x099: err = StrToInt16( seg[segIndex], cfg.fgen.freq ); break;
	 case 0x09a: err = StrToInt32( seg[segIndex], cfg.fgen.amp  ); break;
	 case 0x09b: err = StrToInt16( seg[segIndex], cfg.fgen.duty ); break;
	 case 0x0A5: err = StrToUInt16( seg[segIndex], cfg.io.inPullUpCfg ); break;

	 case 0x0A7: 
	    err = StrToInt32( seg[segIndex], i32 ); 
	    cfg.faultMask = (AMP_FAULT)i32; 
	    break;

	 case 0x0A8: err = StrToInt16 ( seg[segIndex], cfg.pwmIn.cfg    ); break;
	 case 0x0A9: err = StrToInt32 ( seg[segIndex], cfg.pwmIn.scale  ); break;
	 case 0x0Ae: err = StrToInt16 ( seg[segIndex], cfg.cLoop.offset ); break;
	 case 0x0Af: err = StrToUInt32( seg[segIndex], cfg.options      ); break;
	 case 0x0B1: err = StrToInt16 ( seg[segIndex], cfg.stepRate     ); break;

	 case 0x0B2: 
	    err = StrToInt16( seg[segIndex], i16 ); 
	    cfg.phaseMode = (AMP_PHASE_MODE)i16; 
	    break;

	 case 0x0B6:
	    err = StrToInt16( seg[segIndex], cfg.pwmIn.freq ); 
	    break;

	 case 0x0B8:
	    StrToLoadPos( seg[segIndex], cfg.limit.pos );
	    break;

	 case 0x0B9:
	    StrToLoadPos( seg[segIndex], cfg.limit.neg );
	    break;

	 case 0x0BA:
	    StrToLoadPos( seg[segIndex], cfg.window.trackErr );
	    break;

	 case 0x0BB:
	    StrToLoadPos( seg[segIndex], cfg.window.trackWarn );
	    break;

	 case 0x0BC:
	    StrToLoadPos( seg[segIndex], cfg.window.settlingWin );
	    break;

	 case 0x0BD: 
	    err = StrToUInt16( seg[segIndex], cfg.window.settlingTime ); 
	    break;

	 case 0x0BE:
	    StrToLoadAcc( seg[segIndex], cfg.limit.accel );
	    break;

	 case 0x0BF:
	    err = StrToInt16( seg[segIndex], cfg.home.delay );
	    break;

	 case 0x0C1:
	    err = StrToInt16( seg[segIndex], i16 );
	    cfg.can.FromAmpFormat( i16 );
	    break;

	 case 0x0C2:
	    err = StrToUInt16( seg[segIndex], u16 );
	    cfg.home.method = HomeMethodConvert( u16 );
	    break;

	 case 0x0C3:
	    StrToLoadVel( seg[segIndex], cfg.home.velFast );
	    break;

	 case 0x0C4:
	    StrToLoadVel( seg[segIndex], cfg.home.velSlow );
	    break;

	 case 0x0C5:
	    StrToLoadAcc( seg[segIndex], cfg.home.accel );
	    break;

	 case 0x0C6:
	    StrToLoadPos( seg[segIndex], cfg.home.offset );
	    break;

	 case 0x0C7:
	    err = StrToInt16( seg[segIndex], cfg.home.current );
	    break;

	 case 0x0C8: 
	    // When setting the profile type over the CANopen interface
	    // we use an encoding that is consistent with DSP402.  Convert
	    // this here:
	    err = StrToInt16( seg[segIndex], i16 );
	    if( err ) break;

	    switch( i16 & 7 )
	    {
	       case 0: cfg.profile.type = PROFILE_TRAP;   break;
	       case 1: cfg.profile.type = PROFILE_SCURVE; break;
	       case 2: cfg.profile.type = PROFILE_VEL;    break;
	       case 3: cfg.profile.type = PROFILE_TRAP;   break;
	       default:
		       err = &AmpFileError::range;
		       break;
	    }
	    break;

	 case 0x0CA: StrToLoadPos( seg[segIndex], cfg.profile.pos   ); break;
	 case 0x0CB: StrToLoadVel( seg[segIndex], cfg.profile.vel   ); break;
	 case 0x0CC: StrToLoadAcc( seg[segIndex], cfg.profile.acc   ); break;
	 case 0x0CD: StrToLoadAcc( seg[segIndex], cfg.profile.dec   ); break;
	 case 0x0CE: StrToLoadJrk( seg[segIndex], cfg.profile.jrk   ); break;
	 case 0x0CF: StrToLoadAcc( seg[segIndex], cfg.profile.abort ); break;

	 case 0x0D0: 
	    err = StrToInt16( seg[segIndex], i16 ); 
	    cfg.io.inCfg[ 8] = (INPUT_PIN_CONFIG)i16; 
	    break;

	 case 0x0D1: 
	    err = StrToInt16( seg[segIndex], i16 ); 
	    cfg.io.inCfg[ 9] = (INPUT_PIN_CONFIG)i16;
	    break;

	 case 0x0D2:
	    err = StrToInt16( seg[segIndex], i16 );
	    cfg.io.inCfg[10] = (INPUT_PIN_CONFIG)i16;
	    break;

	 case 0x0D3:
	    err = StrToInt16( seg[segIndex], i16 );
	    cfg.io.inCfg[11] = (INPUT_PIN_CONFIG)i16;
	    break;

	 case 0x0D8: 
	    err = StrToUInt16( seg[segIndex], cfg.regen.resistance ); 
	    break;

	 case 0x0D9: 
	    err = StrToUInt16( seg[segIndex], cfg.regen.contPower ); 
	    break;

	 case 0x0DA: 
	    err = StrToUInt16( seg[segIndex], cfg.regen.peakPower ); 
	    break;

	 case 0x0DB: 
	    err = StrToUInt16( seg[segIndex], cfg.regen.peakTime ); 
	    break;

	 case 0x0E1: 
	    strncpy( cfg.regen.model, seg[segIndex], COPLEY_MAX_STRING ); 
	    break;

	 case 0x0E3:
	    err = StrToInt16( seg[segIndex], cfg.pLoop.scale  );
	    break;

	 case 0x0E8: err = StrToUInt16( seg[segIndex], cfg.cLoop.stepHoldCurrent ); break;
	 case 0x0E9: err = StrToUInt16( seg[segIndex], cfg.cLoop.stepRun2HoldTime ); break;
	 case 0x0ED: err = StrToUInt16( seg[segIndex], cfg.cLoop.stepVolControlDelayTime ); break;

	 case 0x0F0: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 0] ); break;
	 case 0x0F1: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 1] ); break;
	 case 0x0F2: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 2] ); break;
	 case 0x0F3: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 3] ); break;
	 case 0x0F4: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 4] ); break;
	 case 0x0F5: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 5] ); break;
	 case 0x0F6: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 6] ); break;
	 case 0x0F7: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 7] ); break;
	 case 0x0F8: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 8] ); break;
	 case 0x0F9: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[ 9] ); break;
	 case 0x0FA: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[10] ); break;
	 case 0x0FB: err = StrToInt16( seg[segIndex], cfg.io.inDebounce[11] ); break;

	 default:
	    cml.Debug( "Unknown paramaeter in CCX file: 0x%02x\n", param );
	    break;
      }
   }

   fclose(fp);

   if( err ) return err;

   // The file was read in successfully.  Now, upload the configuration
   // structure to the amplifier.
   return SetAmpConfig( cfg );
#endif
}

#ifdef CML_FILE_ACCESS_OK
/***************************************************************************/
/**
  Read a string from the passed file.  The string is read until the EOF or end
  of line is encountered.  If the line is too long to fit in the passed buffer,
  the line will be clipped, but the fill will be read until the EOL and any 
  extra data will be lost.
  @param fp The file pointer of an open file
  @param buff The buffer where the data will be returned
  @param max The size of the buffer.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
static const Error *ReadLine( FILE *fp, char *buff, int max )
{
   for( int i=0; ; )
   {
      int c = getc(fp);

      if( c == EOF || c == '\n' )
      {
	 buff[i] = 0;
	 return 0;
      }

      if( i < (max-1) )
	 buff[i++] = c;
   }
}


/***************************************************************************/
/**
  Split the passed string into segments delimited by a character.
  This also strips white space at the start and end of each segment.
  @param buff The string to split.  This should end with a terminating zero.
  @param seg An array of character pointers which will point to the split
  line segments on return
  @param max The maximum number of segments to split the line into.
  @param delim The delimiter (comma by default)
  @return The number of segments the line was split into
  */
/***************************************************************************/
static int SplitLine( char *buff, char **seg, int max, char delim )
{
   int i;
   for( i=0; i<max && *buff; i++ )
   {
      // Skip leading white space
      while( isspace(*buff) )
	 buff++;

      if( i==0 && *buff==0 )
	 break;

      // Grab start of next segment
      seg[i] = buff;

      // Find end
      while( *buff && *buff != delim )
	 buff++;

      // Strip trailing white space
      for( char *ptr = buff-1; ptr >= seg[i] && isspace(*ptr); *ptr-- = 0 );

      // End segment
      if( *buff == delim )
	 *buff++ = 0;
   }
   return i;
}

/***************************************************************************/
/**
  Convert an ASCII string into a 32-bit integer value.
  @param str The string to convert
  @param i The integer value will be returned here
  @param base The base to use during the conversion
  @return An error pointer or NULL on success
  */
/***************************************************************************/
static const Error *StrToInt32( char *str, int32 &i, int base )
{
   char *end;
   i = (int32)strtol( str, &end, base );

   if( *end )
      return &AmpFileError::format;
   return 0;
}

/***************************************************************************/
/**
  Convert an ASCII string into a 32-bit unsigned integer value.
  @param str The string to convert
  @param i The integer value will be returned here
  @param base The base to use during the conversion
  @return An error pointer or NULL on success
  */
/***************************************************************************/
static const Error *StrToUInt32( char *str, uint32 &i, int base )
{
   char *end;
   i = (int32)strtoul( str, &end, base );

   if( *end )
      return &AmpFileError::format;
   return 0;
}

/***************************************************************************/
/**
  Convert an ASCII string into a 16-bit integer value.
  @param str The string to convert
  @param i The integer value will be returned here
  @param base The base to use during the conversion
  @return An error pointer or NULL on success
  */
/***************************************************************************/
static const Error *StrToInt16( char *str, int16 &i, int base )
{
   int32 l;
   const Error *err = StrToInt32( str, l, base );
   if( err ) return err;

   if( (l < -32768) || (l>32767) )
      return &AmpFileError::range;

   i = (int16)l;
   return 0;
}

/***************************************************************************/
/**
  Convert an ASCII string into a 16-bit unsigned integer value.
  @param str The string to convert
  @param i The integer value will be returned here
  @param base The base to use during the conversion
  @return An error pointer or NULL on success
  */
/***************************************************************************/
static const Error *StrToUInt16( char *str, uint16 &i, int base )
{
   uint32 l;
   const Error *err = StrToUInt32( str, l, base );
   if( err ) return err;

   if( (l>65535) )
      return &AmpFileError::range;

   i = (uint16)l;
   return 0;
}

/***************************************************************************/
/**
  Convert an ASCII string (as read from a .ccx file) into an output pin
  configuration & mask value.
  @param str The string to decode
  @param cfg The configuration will be returned here
  @param mask1 The first output pin mask value will be returned here.
  @param mask2 The second output pin mask value will be returned here.
  @return An error pointer or NULL on success
  */
/***************************************************************************/
static const Error *StrToOutCfg( char *str, OUTPUT_PIN_CONFIG &cfg, uint32 &mask1, uint32 &mask2 )
{
   const Error *err = 0;
   char *seg[4];
   int16 c;
   
   // The second mask is optional.  If it's not in the 
   // file then it defaults to zero.
   mask2 = 0;

   switch( SplitLine( str, seg, 4, ':' ) )
   {
      case 3:
	 err = StrToUInt32( seg[2], mask2, 16 );

      case 2:
	 if( !err ) err = StrToInt16( seg[0], c, 16 );
	 if( !err ) err = StrToUInt32( seg[1], mask1, 16 );
	 cfg = (OUTPUT_PIN_CONFIG)c;
	 break;

      default:
	 return &AmpFileError::format;
   }

   return err;
}
/***************************************************************************/
/**
  Convert an ASCII string (as read from a .ccx file) into a set of filter
  coefficients.
  @param str The string to decode
  @param flt The filter structure to be filled 
  @return An error pointer or NULL on success
  */
/***************************************************************************/
static const Error *StrToFilter( char *str, Filter &flt )
{
   char *seg[10];

   if( SplitLine( str, seg, 10, ':' ) != 9 )
      return &AmpFileError::format;


   const Error *err = 0;
   int16 coef[9];
   for( int i=0; i<9; i++ )
   {
      if( !err ) 
	 err = StrToInt16( seg[i], coef[i] );
   }

   flt.fromWords( coef );
   return err;
}

static const Error *StrToHostCfg( char *str, char *hostCfg )
{
   char *seg[20];

   if( SplitLine( str, seg, 20, ':' ) != 20 )
      return &AmpFileError::format;

   const Error *err = 0;
   for( int i=0; i<20; i++ )
   {
      int16 c;
      err = StrToInt16( seg[i], c );
      if( err ) return err;

      hostCfg[2*i  ] = (char)(c>>8);
      hostCfg[2*i+1] = (char)(c);
   }

   return 0;
}

/***************************************************************************/
/**
Convert the homing method from the 16-bit value in the ccx file to a CANopen
standard homing method.
*/
/***************************************************************************/
#define HOME_BITS_MODE    0x000F
#define HOME_BITS_DIR     0x0010
#define HOME_BITS_NDX     0x0020
#define HOME_BITS_NDXIN   0x0040
#define HOME_BITS_LOW     0x0100
static COPLEY_HOME_METHOD HomeMethodConvert( uint16 x )
{
   // The home method stored in the ccx file is formatted as follows
   //  xxxxxxxxxxxxxxxx
   //  ............\\\\---- MODE:     General home mode.  See below for details.
   //  ...........\-------- DIR:      Direction (0=positive, 1=negative)
   //  ..........\--------- INDEX:    Home on an index if set
   //  .........\---------- NDX_IN:   Index select.  if set, use inner index, outter for clear
   //  ........\----------- NDXFALL:  If set, capture falling edge of index, else rising.
   //  .......\------------ LOW:      Selects home switch edge (0=high, 1=low)
   //  ......\------------- ZERO:     If set, move to zero after home.
   //
   // Not all of these bits map into the CANopen home method, but we do the best
   // that we can here.

   switch( x & HOME_BITS_MODE )
   {
      // Home to index
      case 0:
	 if( !(x & HOME_BITS_NDX) )
	    return CHM_NONE;
	 else if( x & HOME_BITS_DIR )
	    return CHM_NDX_NEG;
	 else
	    return CHM_NDX_POS;

      // Home to limit switch
      case 1:
	 if( x & HOME_BITS_NDX )
	    return (x&HOME_BITS_DIR) ? CHM_NLIM_ONDX : CHM_PLIM_ONDX;
	 else
	    return (x&HOME_BITS_DIR) ? CHM_NLIM : CHM_PLIM;

      // Constant home switch
      case 2:
	 if( !(x & HOME_BITS_NDX) )
	    return (x&HOME_BITS_DIR) ? CHM_NHOME : CHM_PHOME;

	 else if( x & HOME_BITS_NDXIN )
	    return (x&HOME_BITS_DIR) ? CHM_NHOME_INDX : CHM_PHOME_INDX;

	 else
	    return (x&HOME_BITS_DIR) ? CHM_NHOME_ONDX : CHM_PHOME_ONDX;

      // Momentary home switch
      case 3:
	 if( !(x & HOME_BITS_NDX) )
	 {
	    if( x & HOME_BITS_LOW )
	       return (x&HOME_BITS_DIR) ? CHM_LHOME_NEG : CHM_LHOME_POS;
	    else
	       return (x&HOME_BITS_DIR) ? CHM_UHOME_NEG : CHM_UHOME_POS;
	 }

	 else if( x & HOME_BITS_NDXIN )
	 {
	    if( x & HOME_BITS_LOW )
	       return (x&HOME_BITS_DIR) ? CHM_LHOME_INDX_NEG : CHM_LHOME_INDX_POS;
	    else
	       return (x&HOME_BITS_DIR) ? CHM_UHOME_INDX_NEG : CHM_UHOME_INDX_POS;
	 }

	 else
	 {
	    if( x & HOME_BITS_LOW )
	       return (x&HOME_BITS_DIR) ? CHM_LHOME_ONDX_NEG : CHM_LHOME_ONDX_POS;
	    else
	       return (x&HOME_BITS_DIR) ? CHM_UHOME_ONDX_NEG : CHM_UHOME_ONDX_POS;
	 }

      // Home to hard stop
      case 4:
	 if( !(x & HOME_BITS_NDX) )
	    return (x&HOME_BITS_DIR) ? CHM_HARDSTOP_NEG : CHM_HARDSTOP_POS;
	 else
	    return (x&HOME_BITS_DIR) ? CHM_HARDSTOP_ONDX_NEG : CHM_HARDSTOP_ONDX_POS;

      // Default to no movement.
      default:
	 return CHM_NONE;
   }
}

#endif
