/*
 * File: ppsbeeline.c
 * Author: Jihan Ryu
 * Version: 2.2
 * Date: 9/05/2006
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
 
//max_delay    ssGetSFcnParam(S,0) (gpsins.maxbeelinedelay) /* maximum time delay (see comments below) */
//Ts           ssGetSFcnParam(S,1) (Ts, initialized in init_model.m) /*Sampling Time*/

/*DISCARDING OF BEELINE GPS UPDATES WITH LONG DELAYS: Beeline GPS updates
 *with delays exceeding the time (in sample times) specified in max_delay
 *are replaced with a flag of "-2" that instructions hdgfilter_s.c and 
 *rollfilter.c to neglect those GPS updates entirely.  Examination of 
 *the Beeline GPS data from a sampling of old data sets revealed that no
 *delays appeared above 271 sample times (0.5420s at 500 Hz); with this
 *in mind, max_delay was set at 0.600 s such that nearly all (if not all)
 *Beeline GPS updates would be included in the filtering process and any 
 *lengthy delays would be handled using the split integration routines in
 *hdgfilter_s.c and rollfilter.c.  If processing constraints require that
 *max_delay be lowered for feasible operation, the user should be warned 
 *that increased performance will be at the expense of dropped GPS updates*/

/* EDIT, RYH 5/12/07: Set sample time to Ts, not inherited for compatibility
 *with simulation */

#define S_FUNCTION_NAME  ppsbeeline
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>

#define NUMOFINPUTS 	2
#define NUMOFOUTPUTS	2

#define DEFAULT_DELAY	50		/* default time delay, not used */
#define PPS_THRESHHOLD	0.5		/* threshhold for PPS signal (high) */

#define TRU	1
#define FALS	0

/*==================*
 * Global Variables *
 *==================*/
static int_T BEGINHIGHbee, PPSHIGHbee;
static real_T PPStimeNewBee, PPStimeBee, oldUTCppsBee, delayPPSbee;

/*====================*
 * S-function methods *
 *====================*/
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 2);
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

    if (!ssSetNumOutputPorts(S, NUMOFOUTPUTS)) {
    	return;
    }
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortWidth(S, 1, 1);

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
	BEGINHIGHbee = FALS;
    PPSHIGHbee = FALS;
	PPStimeNewBee = PPStimeBee = -1;
	delayPPSbee = DEFAULT_DELAY;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    real_T *y0 = ssGetOutputPortRealSignal(S, 0);
    real_T *y1 = ssGetOutputPortRealSignal(S, 1);
    real_T time = (real_T)ssGetT(S);
    real_T PPS = *uPtrs[0];
    real_T UTC = *uPtrs[1];
    
    const real_T Ts = mxGetPr(ssGetSFcnParam(S,1))[0];
    const real_T max_delay = mxGetPr(ssGetSFcnParam(S,0))[0];

	/* round UTC */
	UTC = floor((UTC * 100.0) + 0.5) / 100.0;
    /* looking for PPS pulse changes and record the time */
    if (PPS >= PPS_THRESHHOLD) { // check for a logical high
    	if (time == 0) { // if high on the first sample
    	    BEGINHIGHbee = TRU; // set flag to handle starting high
    	}
    	else if (!PPSHIGHbee && !BEGINHIGHbee) { // if high and last state was not high and the first sample wasn't high
    		PPSHIGHbee = TRU; // store that this sample was high
    		PPStimeNewBee = time; // record the time of the rising edge
    	}
    }
    else {
    	PPSHIGHbee = FALS; // store that this sample was low
    	BEGINHIGHbee = FALS; // flag that we're not starting low (only needed once)
    }

    if (time == 0) { // one-time initialization
    	oldUTCppsBee = UTC;// grab the startup UTC time of PPS signal as the saved value
    }
    /* when UTC changes */
    if (((UTC - oldUTCppsBee) > 0.01) || ((UTC - oldUTCppsBee) < -0.01)) {
	    /* looking for integer jumps of UTC */
	    if (UTC - (int_T)(oldUTCppsBee) >= 1) {
	    	PPStimeBee = PPStimeNewBee;
	    }
	    delayPPSbee = (time - PPStimeBee - (UTC - (int_T)(UTC))) / Ts; // (int_T) cast makes this test for jumps of 1 AND even seconds
	    oldUTCppsBee = UTC; // store the time of this UTC change
        y1[0] = 1; // UTC change, set the flag high
    }
    else {
        y1[0] = 0; // no new UTC, so set the flag low
    }
    
    /* if the calculation produces a faulty answer */
    if (delayPPSbee < 0) {
        delayPPSbee = -3.33; // non-integer to disambiguate from bad data
    }

    /* before the first PPS pulse */
    if (PPStimeBee < 0) {
    	delayPPSbee = -1.11; // non-integer to disambiguate from bad data
    }
    
    /*  if the delay exceeds the limit */
    if (delayPPSbee > max_delay)  {
        delayPPSbee = -2.22; // non-integer to disambiguate from bad data
    }
    
    y0[0] = delayPPSbee;
}

static void mdlTerminate(SimStruct *S)
{
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
