/*
 * File: encoderinit.c  - Simulink encoder initialization s-function 
 *
 * Author: Christopher Gadda
 * (C)2005 Dynamic Design Laboratory -- Stanford University
 *
 * $Revision: 107 $ $Date: 2005-07-10 12:22:29 -0700 (Sun, 10 Jul 2005) $
 *
 * This file implements an s-function for use with Simulink.  
 *
 * This file is based (very loosely) on $MATLAB/simulink/src/sfuntmpl_basic.c
 * 
 * Code revised by C. Beal on 10.26.2021 for more thorough commenting and
 * clarity of the code.
 *
 */

#define S_FUNCTION_NAME  encoderinit
#define S_FUNCTION_LEVEL 2

#include <math.h>
#include "simstruc.h"

#define round(x) (floor((x)+.5))

#define NUMOFSTATES	6
// x[0] = FSM state: {firstav|roughcal|secondav|finalcal}
// x[1] = running average
// x[2] = start time of the second calibration
// x[3] = starting position of the second calibration
// x[4] = unwrap count at end of first calibration
// x[5] = second running average
#define NUMOFPARAMS 3   // (see below)
#define NUMOFINPUTS 3   // (see below)
#define NUMOFOUTPUTS 1  // Calibrated encoder output

// Parameter defines & descriptions (from block mask)
#define TS				0 // Sample time
#define FIRST_CAL_TIME	1 // First calibration averaging time
#define SECOND_CAL_TIME	2 // Second calibration averaging time

// Input defines & descriptions
#define UNWRAPPED   0 // Unwrapped encoder signal
#define RAW         1 // Raw encoder signal
#define POT         2 // Potentiometer signal (scaled by gearbox ratio)

// FSM (finite state machine) states
#define FIRSTAV 0
#define ROUGHCAL 1
#define SECONDAV 2
#define FINALCAL 3

#define PI 3.14159265358979

static int INPUTWIDTHS[]={1,1,1};
static int OUTPUTWIDTHS[]={1};

#define JUMPTHRESH .1

/*====================*
* S-function methods *
*====================*/
static void mdlInitializeSizes(SimStruct *S)
{
	int i;
    
    ssSetNumSFcnParams(S, NUMOFPARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        ssSetErrorStatus(S,"Incorrect number of parameters.");
        return;
    }
    
    ssSetNumDiscStates(S, NUMOFSTATES);
    ssSetNumContStates(S, 0);
	
    if (!ssSetNumInputPorts(S, NUMOFINPUTS))
    	return;
    
    for(i=0;i<NUMOFINPUTS;i++)
    {
        ssSetInputPortWidth(S, i, INPUTWIDTHS[i]);
        ssSetInputPortDirectFeedThrough(S, i, 1);
    }
    
    if (!ssSetNumOutputPorts(S, NUMOFOUTPUTS))
    	return;
    for(i=0;i<NUMOFOUTPUTS;i++)
    {
        ssSetOutputPortWidth(S, i, OUTPUTWIDTHS[i]);
    }
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
	
    ssSetOptions(S, 0);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
	const real_T Ts=*mxGetPr(ssGetSFcnParam(S,TS));
	
    ssSetSampleTime(S, 0, Ts);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS
#ifdef MDL_INITIALIZE_CONDITIONS
/* Function: mdlInitializeConditions =================================== */
// This function runs once at the start of the model run.
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *x=ssGetRealDiscStates(S);
	int i;
	
    x[0]=FIRSTAV;  // Set the FSM state to first averaging (state 0).
	for(i=1;i<NUMOFSTATES;i++)
		x[i]=0;  // Initialize all states (averages and counts) to zero
}
#endif /* MDL_INITIALIZE_CONDITIONS */


/* Function: mdlOutputs ================================================ */
// This function gets run once each time step, after mdlUpdate runs.
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // The pointer 'u' is set to the input port address
    const real_T *u=*ssGetInputPortRealSignalPtrs(S,0);
    // The pointer 'out' is set to the output port address
    real_T *out=ssGetOutputPortRealSignal(S,0);
    // The pointer 'x' is set to the start address of the state vector
    const real_T *x=ssGetRealDiscStates(S);
	
	// Offset the unwrapped encoder value by whatever calibration we have
	// available. The latest and greatest calibration is always in x[1].
	*out=u[0]+x[1];
}

#define MDL_UPDATE 
#ifdef MDL_UPDATE
/* Function: mdlUpdate ================================================= */
static void mdlUpdate(SimStruct *S, int_T tid)
{
    // Set pointers to each of the s-function parameters
	const real_T Ts=*mxGetPr(ssGetSFcnParam(S,TS));
	const real_T firstCalTime=*mxGetPr(ssGetSFcnParam(S,FIRST_CAL_TIME));
	const real_T secondCalTime=*mxGetPr(ssGetSFcnParam(S,SECOND_CAL_TIME));
    // Set a pointer to each of the input signals
	const real_T unwrapped=**ssGetInputPortRealSignalPtrs(S,UNWRAPPED);
	const real_T raw=**ssGetInputPortRealSignalPtrs(S,RAW);
	const real_T pot=**ssGetInputPortRealSignalPtrs(S,POT);
    // Get the value of the simulation time
	const time_T time=ssGetT(S);
    // Set a pointer to the start of the state vector
	real_T *x=ssGetRealDiscStates(S);
    // Define a variable for the handwheel steering angle
	real_T delta;
	
    // Use a switch case to determine the current state of the FSM
	switch((int)x[0])
	{
        // CODE FOR the FIRSTAV state
		case FIRSTAV:
			// When the calibration period elapses switch to the ROUGHCAL state.
			// Otherwise, continue averaging the difference between
			// the potentiometer reading and the encoder reading.
			if(time>=firstCalTime)   
			{
                // Records the current unwrap count for comparison in the ROUGHCAL state
				x[4]=raw-unwrapped;
                // Causes the FSM to advance to the ROUGHCAL state on the next cycle
				x[0]=ROUGHCAL;
			}
			else  
            {
                // Update the running average at each time step
				x[1]+=(pot-unwrapped)*Ts/firstCalTime;
                // CEB: this isn't accurate until the end of the first averaging time window. 
                // Seems like it would be possible to keep the sum separate and divide by
                // the number of steps, or adjust for it each time
                // i.e. x[1] = x[1]*(time-Ts)/Ts + (pot-unwrapped)*(Ts/time);
                // The first term multiplies by the previous number of time steps (un-averages)
                // and the second term divides by the current number of time steps (re-averages)
            }
			break;
		case ROUGHCAL:
			// A rough calibration has been completed.  Now we wait until we
			// detect an unwrap jump in the encoder signal.  We can spot such
			// a jump by comparing the raw and the unwrapped encoder inputs.
			// When we detect a jump, we initialize everything for the second
			// averaging cycle.
			if(fabs(raw-unwrapped-x[4])>JUMPTHRESH)
			{
				x[2]=time;  // Record the start time.
				x[3]=unwrapped;  // Record the start position.
                // Causes the FSM to advance to the ROUGHCAL state on the next cycle
				x[0]=SECONDAV;
			}
			break;
		case SECONDAV:
			// We've just found an unwrap jump in the encoder signal.  This
			// means we're near the index mark, so we need to compute a second
			// averaged potentiometer reading.  Since we're clearly in motion
			// at this point, we need to subtract off any motion recorded by
			// the encoder.  When the calibration period elapses, we quantize
			// the averaged value to the nearest complete revolution.  We also
			// switch to the FINALCAL state.
			if((time-x[2])>secondCalTime)   
			{
                // CEB: I think the first line here is the correct one. Not sure
                // why the second one was added and the first commented out.
				x[1]=2*PI*(round((x[5]+x[3])/(2*PI))-round(x[3]/(2*PI)));
				//x[1]=2*PI*round(x[5]/(2*PI));
				x[0]=FINALCAL;      
			}
			else
				x[5]+=(pot-unwrapped)*Ts/secondCalTime;
			break;
		case FINALCAL:
			// In this state we don't need to do anything; we're done!
			break;
		default:;
	}
}
#endif /* MDL_UPDATE */

#undef MDL_DERIVATIVES  /* This is a discrete-time filter. */

/* Function: mdlTerminate ===================================================== */
static void mdlTerminate(SimStruct *S)
{
}

/*=============================*
* Required S-function trailer *
*=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
