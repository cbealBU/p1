/*
 * File: utcchangebeeline.c
 * Author: Jihan Ryu
 * Version: 1.2
 * Date: 07/10/2003
 * Abstract:
 *	Outputs a pulse when the UTC Time changes.
 *	The input is
 *		*uPtrs[0] = UTC time from Beeline
 * 	The output is
 *		y[0] = Pulse (1 if UTC time changes else 0)
 *
 * See matlabroot/simulink/src/sfuntmpl_doc.c for more details
 */
/* EDIT, RYH 5/12/07: Set sample time to Ts, not inherited for compatibility
 *with simulation */

#define S_FUNCTION_NAME  utcchangebeeline
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>
#include <stdio.h>

#define NUMOFINPUTS 	1
#define NUMOFOUTPUTS	1

/*==================*
 * Global Variables *
 *==================*/
static real_T oldUTCbee;

/*====================*
 * S-function methods *
 *====================*/
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 1);
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
    real_T Ts = mxGetPr(ssGetSFcnParam(S, 0))[0];
    ssSetSampleTime(S, 0, Ts);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
static void mdlInitializeConditions(SimStruct *S)
{
	oldUTCbee = 0;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    real_T time = ssGetT(S);
    real_T UTC = *uPtrs[0];
    
    if (time == 0) {
    	oldUTCbee = UTC;
    }

   	/* round UTC */
   	/*
	UTC = floor((UTC * 10.0) + 0.5) / 10.0;
	oldUTCbee = floor((oldUTCbee * 10.0) + 0.5) / 10.0;
    */

	if (((UTC - oldUTCbee) > 0.01) || (UTC - oldUTCbee < -0.01)) {
    /* if (oldUTCbee != UTC) { */
    	y[0] = 1;
    	oldUTCbee = UTC;
    }
    else {
    	y[0] = 0;
    }
}

static void mdlTerminate(SimStruct *S)
{
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
