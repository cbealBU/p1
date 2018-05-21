/*
 * File: enable.c
 *
 * Author: Christopher Gadda
 * (C)2004 Dynamic Design Laboratory -- Stanford University
 *
 * This file implements an s-function for use with Simulink, which combines
 *
 * This file is based (very loosely) on $MATLAB/simulink/src/sfuntmpl_basic.c
 *
 * Information from Shad, by Mick, Nov 5, 09
 * Function of this enable is to make sure that we don't burn up the motors by sitting in place doing nothing.  
 * It's not uncommon that the code is running and the car is stopped on concrete/asphalt (e.g. all of the time while on the parking garage).
 * In this configuration, the motors often don't have enough torque to overcome the scrubbing of the stationary wheel.  
 * So, what happens is that the handwheel gets a bit misaligned from the road wheels, the controller quickly saturates the motor command 
 * at 20A (*), and the motors sit there, doing nothing and making a lot of heat.  The *continuous* rating of the motors is well below 20A, 
 * so this is bad.
 * I think there was also a secondary reason.  When the steering controller starts up, it's not uncommon for 
 * the wheels to move very violently at first.  It's important to ensure that no one is in the 
 * way when the controller is turned on or else injury could occur.  So, the logic was written to be 
 * one more step to ensure the driver really wanted to get moving.   
 */


#define S_FUNCTION_NAME  enable
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>

#define NUMOFPARAMS 3
#define NUMOFSTATES	2
#define NUMOFINPUTS 4
#define NUMOFOUTPUTS 1

static int INPUTWIDTHS[]={1,1,1,1};
static int OUTPUTWIDTHS[]={1};

#define PI 3.14159265358979
#define ZERO_THRESH (2*PI/180)
#define RATE_THRESH (5*PI/180)
#define SPEED_THRESH .1


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
        ssSetInputPortDirectFeedThrough(S, i, 0);
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
	const real_T Ts=*mxGetPr(ssGetSFcnParam(S,2));

    ssSetSampleTime(S, 0, Ts);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS
#ifdef MDL_INITIALIZE_CONDITIONS
/* Function: mdlInitializeConditions ========================================= */
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *x=ssGetRealDiscStates(S);

    x[0]=0.0; /* Have we ever seen a signal on the zero input?  No, not yet. */
	x[1]=0.0; /* How long since we were last tickled?  O sec. */
}
#endif /* MDL_INITIALIZE_CONDITIONS */


/* Function: mdlOutputs ======================================================= */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T *Enable=ssGetOutputPortRealSignal(S,0);
    const real_T timeout=*mxGetPr(ssGetSFcnParam(S,0));
    const real_T delay=*mxGetPr(ssGetSFcnParam(S,1));
    const real_T *x=ssGetRealDiscStates(S);
    const real_T time=ssGetT(S);
	int i;
    
    if((x[1]<timeout)&&(x[0]>.01)&&(time>delay))    // activate steering system
        // active when last active time < time out, OK to steer (steering close to zero or autosteer switch is on, 
        // and we pass the start up time
		*Enable=1.0;
    else   // don't activate steering system
		*Enable=0.0;
}

#define MDL_UPDATE 
#ifdef MDL_UPDATE
/* Function: mdlUpdate ======================================================
 * Abstract:
 *    This function is called once for every major integration time step.
 *    Discrete states are typically updated here, but this function is useful
 *    for performing any tasks that should only take place once per
 *    integration step.
 */

static void mdlUpdate(SimStruct *S, int_T tid)
{
    const real_T angle=**ssGetInputPortRealSignalPtrs(S,0);
    const real_T rate=**ssGetInputPortRealSignalPtrs(S,1);
    const real_T speed=**ssGetInputPortRealSignalPtrs(S,2);
    const int_T autoSteerSwitch=**ssGetInputPortRealSignalPtrs(S, 3);  // get state of the autosteer switch
    real_T *x=ssGetRealDiscStates(S);
	const real_T timeout=*mxGetPr(ssGetSFcnParam(S,0));
	const real_T Ts=*mxGetPr(ssGetSFcnParam(S,2));
	
	
	if( (fabs(angle)<ZERO_THRESH) || autoSteerSwitch )  /* If we see an angle near zero, 
     * or if the autosteer switch is on, it's now OK to activate the steering*/ 
		x[0]=1.0;	/* make a note of that. */
	
	if((fabs(rate)>RATE_THRESH)||(speed>SPEED_THRESH))  /* If we get tickled, */
		x[1]=0.0;	 /* reset the tickle counter. */
	else
		x[1]+=Ts;  /* Otherwise increment it. */
	
	if(x[1]>timeout)
		x[1]=timeout+10.0;  /* This prevents unnessary overflow issues... */
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
