//======================================================

#include "CML_Settings.h"
#ifdef CML_ALLOW_FLOATING_POINT

#include <math.h>
#include "CML.h"

CML_NAMESPACE_USE();

CML_NEW_ERROR( TcurveError, BadParam,  "An illegal input parameter was passed" );
CML_NEW_ERROR( TcurveError, NoCalc,    "Trjaectory has not been calculated" );
CML_NEW_ERROR( TcurveError, InUse,     "Trajectory is currently in use" );
CML_NEW_ERROR( TcurveError, NotInUse,  "Trajectory has not been started" );

//----------------------------------------------------------------------
TrjTcurve::TrjTcurve( void )
{
	start = 0;
	segNum = 0;
	inUse = false;
	init = false;
	lastSegVel=0;
	remain=0;
	seg=0;
}

//--------------------------------------------------------------------
void TrjTcurve::SetStartPos( uunit s )
{
	start = s;
	return;
}

//-------------------------------------------------------------------
uunit TrjTcurve::GetStartPos( void )
{
	return start;
}

//-------------------------------------------------------------------
uunit TrjTcurve::GetLastSegmentPos(void)
{
	return end;
}

//------------------------------------------------------------------
void TrjTcurve::SetSegmentNumber(uunit num)
{
	segNum = num;
	return;
}

//------------------------------------------------------------------

double TrjTcurve::GetLastSegmentVel(void)
{
	return lastSegVel;
}

//------------------------------------------------------------------
bool TrjTcurve::IsEndSegment(void)
{
	if (segNum == 1)
	{
		return  true;
	}
	else
	{
		return false;
	}
}

//-----------------------------------------------------------------
const Error *TrjTcurve::Calculate( uunit start, uunit end, uunit vel, uunit acc)
{
   uunit dist = end-start;
   const Error *err = Calculate( dist, vel, acc);
   if( err ) return err;
   SetStartPos( start );
   return 0;
}

//----------------------------------------------------------------
const Error *TrjTcurve::Calculate( uunit dist, uunit maxVel, uunit maxAcc )
{
	if( maxVel<=0 || maxAcc<=0)
		return &TcurveError::BadParam;
	if( inUse ) 
		return &TcurveError::InUse;

	#ifdef CML_ENABLE_USER_UNITS
	P = dist;
	V = maxVel;
	A = maxAcc;
	#else
	P = dist;
	V = maxVel * 0.1;
	A = maxAcc * 10.0;
	#endif

	bool negMove = (P<0);
	if( negMove )
	{
		P *= -1.0;
	}
		

	if( P==0 )
	{
		ta = tv = 0;
		init = true;
		if(IsEndSegment())
		{
			td = 0;
		}
		return 0;
	}

	double lastVel = GetLastSegmentVel();
	double tempMaxVel;
	double tempA;
	uunit movePos;
	uunit remainPos;

	if (V>=lastVel)
	{
		if(IsEndSegment())
		{
			ta = (V-lastVel)/A;
			movePos = lastVel*ta+A*ta*ta/2;
			if(movePos>=P)
			{
				tempMaxVel = sqrt((2*P*A+lastVel*lastVel)/2);
				ta = (tempMaxVel - lastVel)/A;
				td = tempMaxVel/A;
				tv = 0;
			}
			else
			{
				td =V/A;
				movePos = lastVel*ta+A*ta*ta/2+V*td-A*td*td/2;
				if(movePos>=P)
				{
					tempMaxVel = sqrt((2*P*A+lastVel*lastVel)/2);
					ta = (tempMaxVel - lastVel)/A;
					td = tempMaxVel/A;
					tv = 0;
				}
				else
				{
					remainPos = P-movePos;
					tv = remainPos/V;
				}
			}

            lastSegVel=0;
		}
		else
		{
			ta = (V-lastVel)/A;
			movePos = lastVel*ta+A*ta*ta/2;
			if(movePos>=P)
			{
				ta = ((sqrt(lastVel*lastVel+2*A*P)-lastVel))/A;
				td=0;
				tv=0;
				lastSegVel=sqrt(lastVel*lastVel+2*A*P);
			}
			else
			{
				td = 0;
				remainPos = P-(lastVel*ta+A*ta*ta/2);
				tv = remainPos/V;
                lastSegVel=V;
			}
            
		}

	}
	else
	{
		if(IsEndSegment())
		{
			td = lastVel/A;
			movePos = lastVel*td-A*td*td/2;
			if(movePos>=P)
			{
				//tempMaxVel = sqrt(lastVel*lastVel-2*P*A);
				tempA=(lastVel*lastVel)/(2*P);
				td = 0;
				ta = lastVel/tempA;
				tv = 0;
			}
			else
			{
				/*
				td =V/A;
				movePos = lastVel*ta-A*ta*ta/2+V*td-A*td*td/2;
				if(movePos>=P)
				{
					tempMaxVel = sqrt(lastVel*lastVel-2*P*A);
					ta = (lastVel-tempMaxVel)/A;
					td = tempMaxVel/A;
					tv = 0;
				}
				else
				{
					remainPos = P-movePos;
					tv = remainPos/V;
				}
				*/
				remainPos = P-movePos;
				tv = remainPos/V;
				ta = (lastVel-V)/A;
				td=V/A;
			}
			lastSegVel=0;
		}
		else
		{   
			
			ta = (lastVel-V)/A;
			movePos = lastVel*ta-A*ta*ta/2;
			if(movePos>=P)
			{
				//ta = (lastVel-(sqrt(lastVel*lastVel-2*A*P)))/A;
				tempA=(lastVel*lastVel-V*V)/(2*P);
				ta=(lastVel-V)/tempA;
				td=0;
				tv=0;
			}
			else
			{
				td = 0;
				remainPos = P-(lastVel*ta-A*ta*ta/2);
				tv = remainPos/V;
			}
			lastSegVel=V;
		}
	}
	
	

	if( negMove )
	{
		P  *= -1.0;
	}

	init = true;
	segNum--;
	
	return 0;
}

//-----------------------------------------------------------------
const Error *TrjTcurve::StartNew( void )
{
   if( !init ) return &TcurveError::NoCalc;
   if( inUse ) return &TcurveError::InUse;

   inUse = true;

   p = start;
   //p=20000;
   v = a = 0;
   segNum = 20;
   lastSegVel = 0;
   remain = ta;
   //remain=10000;
   seg = 0;
   a=A;
   return 0;
}

//-------------------------------------------------------------------
void TrjTcurve::Finish( void )
{
   inUse = false;
}

//------------------------------------------------------------------
const Error *TrjTcurve::NextSegment( uunit &pout, uunit &vout, uint8 &tout )
{
	if( !inUse ) return &TcurveError::NotInUse;

	#ifdef CML_ENABLE_USER_UNITS
	pout = p;
	vout = v;
	#else
	if( p >= 0 ) 
		pout = (uunit)(p + 0.5);
	else         
		pout = (uunit)(p - 0.5);

	if( v >= 0 ) 
		vout = (uunit)(10.0 * v + 0.5);
	else         
		vout = (uunit)(10.0 * v - 0.5);
	#endif
/*
	if(IsEndSegment())
	{
		if( seg =3)
		{
			tout = 233;
			//tout=255;
			return 0;
		}
	}
	else
	{
		if (seg = 2)
		{
			tout = 250;
			//tout=254;
			return 0;
		}
	}
*/
	bool eos = false;
	double t=0.255;

	if( remain > 0.510 )
	{
		t = 0.255;
		tout = 255;
	}
	else if( remain > 0.254 )
	{
		tout = (uint8)floor(500*remain);
		t = 0.001 * tout;
	}
	else
	{
		eos = true;
		tout = (uint8)floor(remain*1000 + 0.01);
		t = remain;
	}

	p += v*t + a*t*t/2;
	v += a*t ;
	remain -= t;
	if( remain < 0 ) 
	{
		remain = 0;
	}

	if( !eos )
	{
		return 0;
	}

	double ms;
	double f = modf( 1000*t, &ms );
	double tf = 0;

	// Increment my output time if I didn't end on an even millisecond.
	if( f > 0.001 )
	{
		tf = 0.001 * (1.0 - f);
		tout++;
	}

	// Now, keep advancing to the next segment until I either come
	// to the end of the move, or find a segment with more then 1ms
	// available.
	while( AdvanceSegment(tf) && (remain < 0.001) );

	return 0;
}

//------------------------------------------------------------------
int TrjTcurve::AdvanceSegment( double &tf )
{
	// Find the next segment with a non-zero time
	switch( seg++ )
	{
	case 0:
		{
			remain = tv;  
            a=0;
			if( remain > 0.000001 ) 
				break; 
			seg++;
		}
	case 1:
		{
			if(IsEndSegment())
			{
				remain = td; 
//				a=-A;
				break;
			}
			else
			{
				return 0;
			}
			
		}
	default: return 0;
	}

	// Advance the p,v,a values by the amount passed, or by
	// the total time in this segment if it's less.
	double t = (tf > remain) ? remain : tf;

	p += v*t + a*t*t/2;
	v += a*t;

	// Reduce the total time left in this segment by the value used.
	remain -= t;
	tf -= t;
	return 1;
}

#endif

