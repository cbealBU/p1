/*
 * File: ppsmillennium2.c
 * Author: Jihan Ryu
 * Version: 2.2
 * Date: 09/05/2006
 * Abstract:
 *	Calculates the time delay between the PPS pulse and the first integer UTC
 *  time that ocurrs after the PPS pulse. The time delay is outputted in terms
 *	of number of samples.
 *	The inputs are
 *		*uPtrs[0] = PPS pulse from Beeline
 *		*uPtrs[1] = UTC time from Beeline
 * 	The output is
 *		y[0] = time delay in terms of samples
 *
 * See matlabroot/simulink/src/sfuntmpl_doc.c for more details
 */

/*EDIT, 9/05/2006: Rami Hindiyeh implemented the following changes in the code:
 * *EXTERNAL PARAMETER SPECIFICATION: A significant portion of the #define 
 *statements used by Jihan have been replaced by liberal use of the
 *ssGetSFcnParam() function to retrieve these parameters from a newly
 *created parameter file entitled ssest_params.  The parameters are pulled
 *from ssest_params.m by being specified in the "S-Function Parameters"
 *dialog box in SIMULINK.  Also, init_model.m has been included in the 
 initialization of offline_ssest to permit offline availability of the 
 sampling time variable Ts  The parameters for which #define statements have
 *been replaced are as follows:
 
//max_delay    ssGetSFcnParam(S,0) (gpsins.maxoem4delay) /* maximum time delay (see comments below) */
//Ts           ssGetSFcnParam(S,1) (Ts, initialized in init_model.m) /*Sampling Time*/

/*DISCARDING OF BEELINE GPS UPDATES WITH LONG DELAYS: OEM4 GPS updates
 *with delays exceeding the time (in sample times) specified in max_delay
 *are replaced with a flag of "-2" that instructs velfilter_acc.c 
 to neglect those GPS updates entirely.  Examination of 
 *the OEM 4 GPS data from a sampling of old data sets revealed that only a
 *tiny fraction of OEM4 GPS updates exceeded 20-25 sample times (0.04-0.05s
 *at 500 Hz); however, substantially higher delays of 600 sample times 
 *(1.2s) very rarely appeared, and were almost surely the cause
 *of CPU overloads (and associated steering failures) observed by lab 
 *members during use of the 1st generation real time sideslip estimation
 *filter.  With this in mind, max_delay was set at 0.06s to allow for 
 *inclusion of all nominal GPS updates while ignoring the excessively 
 *delayed updates and their disastrous consequences.  Owing to the rarity
 *of long OEM4 delays, split integration was NOT implemented in 
 *velfilter_acc.c*/

/* EDIT, RYH 5/12/07: Set sample time to Ts, not inherited for compatibility
 *with simulation */

#define S_FUNCTION_NAME  ppsmillennium2
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>

#define NUMOFINPUTS 	2
#define NUMOFOUTPUTS	1

// #define Ts				ssGetSFcnParam(S,1)	/* sampling period */
#define DEFAULT_DELAY	16		/* default time delay, not used */
// #define max_delay		ssGetSFcnParam(S,0)		/* maximum time delay */
#define PPS_THRESHHOLD	0.5		/* threshhold for PPS signal (high) */

#define TRU	1
#define FALS	0

/*==================*
 * Global Variables *
 *==================*/
static int_T BEGINHIGHoem, PPSHIGHoem;
static real_T PPStimeNewOem, PPStimeOem, oldUTCppsOem, delayPPSoem;

/*====================*
 * S-function methods *
 *====================*/
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S,2);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 1)) {
    	return;
    }
    ssSetInputPortWidth(S, 0, NUMOFINPUTS);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) {
    	return;
    }
    ssSetOutputPortWidth(S, 0, NUMOFOUTPUTS);

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
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
static void mdlInitializeConditions(SimStruct *S)
{
    BEGINHIGHoem = FALS;
	PPSHIGHoem = FALS;
	PPStimeNewOem = PPStimeOem = -1;
	delayPPSoem = DEFAULT_DELAY;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    real_T time = ssGetT(S);
    real_T PPS = *uPtrs[0];
    real_T UTC = *uPtrs[1];
    
    const real_T Ts = mxGetPr(ssGetSFcnParam(S,1))[0];
    const real_T max_delay = mxGetPr(ssGetSFcnParam(S,0))[0];

	/* round UTC */
	UTC = floor((UTC * 10.0) + 0.5) / 10.0;
    /* looking for PPS pulse changes and record the time */
    if (PPS >= PPS_THRESHHOLD) {
    	if (time == 0) {
    	    BEGINHIGHoem = TRU;
    	}
    	else if (!PPSHIGHoem && !BEGINHIGHoem) {
    		PPSHIGHoem = TRU;
    		PPStimeNewOem = time;
    	}
    }
    else {
    	PPSHIGHoem = FALS;
   	    BEGINHIGHoem = FALS;
    }

    if (time == 0) {
    	oldUTCppsOem = UTC;
    }
    /* when UTC changes */
    if (((UTC - oldUTCppsOem) > 0.01) || ((UTC - oldUTCppsOem) < -0.01)) {
	    /* looking for integer jumps of UTC */
	    if (UTC - (int_T)(oldUTCppsOem) >= 1) {
	    	PPStimeOem = PPStimeNewOem;
	    }
	    delayPPSoem = (time - (PPStimeOem + UTC - (int_T)(UTC))) / Ts;
	    oldUTCppsOem = UTC;
    }
    /* before the first PPS pulse or exceed the limit */
    if (PPStimeOem < 0) {
    	delayPPSoem = -1;
    }
    
    if (delayPPSoem > max_delay) {
        delayPPSoem = -2;
    }
    
    y[0] = delayPPSoem;
}

static void mdlTerminate(SimStruct *S)
{
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
