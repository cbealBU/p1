// handwheel.c - Device driver for SICK/Stegmann absolute encoder used to
// measure handwheel position.
//
// $Revision: 182 $ $Date: 2005-08-23 16:05:29 -0700 (Tue, 23 Aug 2005) $
// Original author: Christopher Gadda
// (C) 2005 Dyanmic Design Laboratory -- Stanford University
//
// This file implements a "level-2" s-function for use with xPC & Simulink.
// The device driver uses the Ruby-MM digital I/O lines to read the current
// value of the encoder.  The device driver uses a single VCM-DAS-2 digital
// I/O line to control the output latch on the encoder.  This is necessary to
// prevent occasional erroneous readings, due to changing data on the encoder
// outputs coincident with an inp instruction reading the I/O lines.

// These defines control where the I/O cards are mapped in the I/O address
// space.  Probably these should go in a project level header file.
#define BASE_IO_RUBYMM	(0x300)		// Base I/O Address of the Ruby-MM card
#define BASE_IO_VCMDAS	(0x000)		// Base I/O Address of the VCM-DAS-2 card

#define ENCODER_RESOLUTION (15)		// Encoder resolution in bits.

// Comment this out if you don't want to check the parity bit.
#define CHECK_PARITY

// This define controls how sensitive the corruption detection logic is.
// The driver will make two readings of the encoder back, one immediately
// after the other.  If they match to within the limit set by
// CORRUPTION_THRESHOLD (in ticks), then the reading is considered to be valid.
#define CORRUPTION_THRESHOLD	(2)	// Maximum change between two readings

#define S_FUNCTION_NAME  handwheel
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>
#ifdef    MATLAB_MEX_FILE
#include  "mex.h"
#endif

#define NUMOFPARAMS 1
#define NUMOFOUTPUTS 1

/* This little section keeps MATLAB from crashing if you happen to update a Simulink model with this
block in it. */
#ifdef	MATLAB_MEX_FILE
#define inp(x) (1)  /* Safer to return 1 than 0 */
#define inpw(x) (1)  /* Safer to return 1 than 0 */
#define	outp(x,y) (0)  /* Essentially does nothing */
#define	outpw(x,y) (0)  /* Essentially does nothing */
#endif

#define PI					(3.14159265358979)

// These offsets are specific to the Ruby-MM I/O board.
#define OFFSET_LSB			(0x0D)

// Some derived constants
#define HALF_REVOLUTION		(1<<(ENCODER_RESOLUTION-1))
#define BIT_MASK			((1<<ENCODER_RESOLUTION)-1)
#define TICKS_TO_RADIANS	((2.0*PI)/((1<<ENCODER_RESOLUTION)*1.0))

static char msg[80];

static void mdlInitializeSizes(SimStruct *S)
{    
    ssSetNumSFcnParams(S, NUMOFPARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        ssSetErrorStatus(S,"Incorrect number of parameters.");
        return;
    }
    
    ssSetNumDiscStates(S, 0);
    ssSetNumContStates(S, 0);
	
    if (!ssSetNumInputPorts(S, 0))
    	return;
	
    if (!ssSetNumOutputPorts(S, NUMOFOUTPUTS))
    	return;
	ssSetOutputPortWidth(S, 0, 1);
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 1);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
	
    ssSetOptions(S, 0);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
	ssSetSampleTime(S, 0, mxGetPr(ssGetSFcnParam(S,0))[0]);
    ssSetOffsetTime(S, 0, 0.0);
}

#ifdef CHECK_PARITY
static int CheckParity(unsigned short dataword)
{
	int i;
	char bit=0;
	
	// Here we loop through all the encoder bits and the parity bit, which 
	// we assume to be one bit position above the most significant bit.
	for(i=0;i<ENCODER_RESOLUTION+1;i++)
	{
		bit^=dataword&1;	// XOR each bit with the running sum.
		dataword>>=1;		// shift the data word over so we get the next bit.
	}
	return !bit;		// Even parity, so bit should == 1 at this point.
}
#endif // CHECK_PARITY

unsigned short FlipBits(unsigned short in)
{
    int i=0;
    unsigned int out;
	
    out=0;
    for(i=0;i<16;i++)
    {
        out=(out<<1)+(in&1);
        in>>=1;
    }
    return out;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
	const real_T previousHandwheelAngle=ssGetRWorkValue(S,0);
	real_T *handwheelAngle=ssGetOutputPortRealSignal(S,0);
	unsigned short word;
    int ticks,delta;
	
	// Read in the data.
	word=FlipBits(inpw(BASE_IO_RUBYMM+OFFSET_LSB));
	ticks=(int)(word&BIT_MASK);
#ifdef CHECK_PARITY
	if(CheckParity(word))
	{
		// If we get here, the parity bit didn't match, so the data must be bad.
        *handwheelAngle=previousHandwheelAngle;
		return;
	}
#endif // CHECK_PARITY
	
	*handwheelAngle=(ticks>HALF_REVOLUTION) ?
		((ticks-2*HALF_REVOLUTION)*TICKS_TO_RADIANS) : (ticks*TICKS_TO_RADIANS);
	
	// Check the integrity of the data by taking a second reading.
	word=FlipBits(inpw(BASE_IO_RUBYMM+OFFSET_LSB));
	
	delta=abs(ticks-(int)(word&BIT_MASK));  // How much did things change?
									   // We may need to unwrap delta.
	if(delta>HALF_REVOLUTION)
		delta-=(2*HALF_REVOLUTION);
	// Now we'll check to see how much of change there has been, if any.
	if(delta>CORRUPTION_THRESHOLD)
   		*handwheelAngle=previousHandwheelAngle;
	
	// Last of all, store the new value of handwheelAngle just in case we need
	// it on the next time step (i.e. if we detect data corruption).
	ssSetRWorkValue(S,0,*handwheelAngle);
}

static void mdlTerminate(SimStruct *S)
{
}

// Here we undef all the standard s-function callbacks that we're not using.
#undef MDL_INITIALIZE_CONDITIONS 
#undef MDL_UPDATE
#undef MDL_DERIVATIVES
#undef MDL_START

/*=============================*
* Required S-function trailer *
*=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif

