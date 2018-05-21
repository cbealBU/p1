/*
 * File: unwrapper.c  - Simulink unwrap s-function 
 *
 * Author: Christopher Gadda
 * (C)2005 Dynamic Design Laboratory -- Stanford University
 *
 * $Revision: 107 $ $Date: 2005-07-10 12:22:29 -0700 (Sun, 10 Jul 2005) $
 *
 * This file implements an s-function for use with Simulink.  
 *
 * This file is based (very loosely on) $MATLAB/simulink/src/sfuntmpl_basic.c
 */

#define S_FUNCTION_NAME  unwrapper
#define S_FUNCTION_LEVEL 2

#include <math.h>
#include "simstruc.h"

#define NUMOFPARAMS 2  // Sample time and "ticks per revolution"
#define NUMOFSTATES	2  // x[0] = Previous input, x[1] = wrap counter
#define NUMOFINPUTS 1
#define NUMOFOUTPUTS 1

static int INPUTWIDTHS[]={1};
static int OUTPUTWIDTHS[]={1};


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
	const real_T Ts=*mxGetPr(ssGetSFcnParam(S,0));
	
    ssSetSampleTime(S, 0, Ts);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS
#ifdef MDL_INITIALIZE_CONDITIONS
/* Function: mdlInitializeConditions ========================================= */
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *x=ssGetRealDiscStates(S);
	
    x[0]=x[1]=0;  // Initialize the wrap counter to zero.
}
#endif /* MDL_INITIALIZE_CONDITIONS */


/* Function: mdlOutputs ======================================================= */
static void mdlOutputs(SimStruct *S, int_T tid)
{
	const real_T ticks=*mxGetPr(ssGetSFcnParam(S,1));
    const real_T in=**ssGetInputPortRealSignalPtrs(S,0);
    real_T *out=ssGetOutputPortRealSignal(S,0);
    const real_T *x=ssGetRealDiscStates(S);
    real_T delta;
	
	*out=in+x[1]; // Add the current value of the wrap counter to the input.

	delta=in-x[0]; // Compute the change since the last sample.

	if(delta>ticks/2)  // If we've jumped up a lot,
		*out-=ticks;   // offset the output down.
	if(delta<-ticks/2) // If we've jumped down a lot,
		*out+=ticks;   // offset the output up.
}


#define MDL_UPDATE 
#ifdef MDL_UPDATE
/* Function: mdlUpdate ====================================================== */
static void mdlUpdate(SimStruct *S, int_T tid)
{
	const real_T ticks=*mxGetPr(ssGetSFcnParam(S,1));
    const real_T in=**ssGetInputPortRealSignalPtrs(S,0);
    real_T *x=ssGetRealDiscStates(S);
	real_T delta;
	
	delta=in-x[0]; // Compute the change since the last sample.
	x[0]=in; // Stash the current input for next time.
	
	if(delta>ticks/2)  // If we've jumped up a lot,
		x[1]-=ticks;   // decrement the wrap counter.
	if(delta<-ticks/2) // If we've jumped down a lot,
		x[1]+=ticks;   // increment the wrap counter.
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
