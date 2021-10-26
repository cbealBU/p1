/*
 * File: vsbc8led.c
 *
 * Author: Christopher Gadda
 * (C)2004 Dynamic Design Laboratory -- Stanford University
 *
 * This file implements an s-function for use with Simulink, which controls
 * the programmable LED on the VSBC8 motherboard.  This can be used for a
 * variety of purposes; I plan to use it as a heartbeat indicator.
 *
 * This file is based (very loosely on) $MATLAB/simulink/src/sfuntmpl_basic.c
 */

#define S_FUNCTION_NAME  vsbc8led
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define NUMOFPARAMS 0
#define NUMOFSTATES	0
#define NUMOFINPUTS 1
#define NUMOFOUTPUTS 0

#define LED_ADDRESS 0xE0  /* May need to change this to 0x1E0 depending on the BIOS setup. */

/* This little section keeps MATLAB from crashing if you happen to update a Simulink model with this
   block in it. */
#ifdef	MATLAB_MEX_FILE
#define inp(x) (1)  /* Safer to return 1 than 0 */
#define	outp(x,y) (0)  /* Essentially does nothing */
#endif

static char msg[80];

/*====================*
 * S-function methods *
 *====================*/
static void mdlInitializeSizes(SimStruct *S)
{    
    ssSetNumSFcnParams(S, NUMOFPARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        ssSetErrorStatus(S,"Incorrect number of parameters.");
        return;
    }
    
    ssSetNumDiscStates(S, NUMOFSTATES);
    ssSetNumContStates(S, 0);

    if (!ssSetNumInputPorts(S, 1))
    	return;
    
	ssSetInputPortWidth(S, 0, 1);
	ssSetInputPortDirectFeedThrough(S, 0, 0);
    
    if (!ssSetNumOutputPorts(S, 0))
    	return;
    
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

#undef MDL_INITIALIZE_CONDITIONS

/* Function: mdlOutputs ======================================================= */
static void mdlOutputs(SimStruct *S, int_T tid)
{
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
    const real_T onoff=**ssGetInputPortRealSignalPtrs(S,0);

	unsigned char tmp;
	
	if(onoff>.5)
	{
		// Turn the LED on!
		tmp=inp(LED_ADDRESS);
		tmp|=0x80;
		outp(LED_ADDRESS,tmp);
	}
	else
	{
		// Turn the LED off.
		tmp=inp(LED_ADDRESS);
		tmp&=0x7F;
		outp(LED_ADDRESS,tmp);
	}
}	        
#endif /* MDL_UPDATE */

#undef MDL_DERIVATIVES  /* This is a discrete-time filter. */

/* Function: mdlTerminate ===================================================== */
static void mdlTerminate(SimStruct *S)
{
	unsigned char tmp;

	// Turn the LED off.
	tmp=inp(LED_ADDRESS);
	tmp&=0x7F;
	outp(LED_ADDRESS,tmp);
}

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
