/*
 * File: estimate_aligningTorque.c
 *
 * Author: Helen
 * (C)2009 Dynamic Design Laboratory -- Stanford University
 *
 * $Revision: 1$ $Date: July 22, 2009 $
 *
 * This file implements an s-function version of a basic envelope controller created in SimulateObserver_full_envelope_control.m for use with Simulink.  
 * 
 *
 * This controller outputs: 
 * 1. steer angle addition (rad)
 * 2. controller status (off = -1, no_saturation = 0, front_saturation = 1, rear_saturation = 2)
 * 
 */

#define S_FUNCTION_NAME  estimate_aligningTorque      /* must match name of c-file */
#define S_FUNCTION_LEVEL 2                      /* usually this value works, do not modify */

#include <math.h>
#include "simstruc.h"                           /* need this for SimStruct later */

/* parameter list:
 * tp, Cf, Fz_f, tm 
 */

/* global constants */
#define NUMOFPARAMS 5                      
#define NUMOFSTATES	0
#define NUMOFINPUTS 4                     
#define NUMOFOUTPUTS 5

static int INPUTWIDTHS[]={1,1,1,1};       
static int OUTPUTWIDTHS[]={1,1,1,1,1};

/*9 global variables */
static real_T If, mu, c0, c1, c2, c3, c4, e;

/* function prototypes */
static real_T GetPneumaticTrail(real_T If, real_T tp0, real_T Cf, real_T alpha_f, real_T aslide);
static real_T GetLateralForce(real_T If, real_T Cf, real_T alpha_f, real_T aslide);
static real_T GetAligningTorque(real_T tp, real_T Fy, real_T tm);
/* ============================================================================================== *
 * Set up the structure of the simulink block. This MUST compile to build s-function blk
 * ============================================================================================== */
static void mdlInitializeSizes(SimStruct *S)        
{
	int i;
    
    ssSetNumSFcnParams(S, NUMOFPARAMS);             /* set # of parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        ssSetErrorStatus(S,"Incorrect number of parameters.");
        return;
    }
    
    ssSetNumDiscStates(S, NUMOFSTATES);             /* set # of discrete states */
    ssSetNumContStates(S, 0);                       /* set # of continuous states */

    if (!ssSetNumInputPorts(S, NUMOFINPUTS))        /* set # of inputs */
    	return;
    
    for(i=0;i<NUMOFINPUTS;i++)                      /* set number of feedthrough inputs */ 
    {
        ssSetInputPortWidth(S, i, INPUTWIDTHS[i]);
        ssSetInputPortDirectFeedThrough(S, i, 1);   /* IS SET TO 1 FOR DEBUGGING!! set to 0 if ith input is NOT fed thru */
    }                                               /* set to 1 if it is fed through */
    
    if (!ssSetNumOutputPorts(S, NUMOFOUTPUTS))      /* set # of outputs */
    	return;
    for(i=0;i<NUMOFOUTPUTS;i++)
    {
        ssSetOutputPortWidth(S, i, OUTPUTWIDTHS[i]);
    }
    
    ssSetNumSampleTimes(S, 1);
    /* if you want to access to prev time step's parameters, 
     * but usually you just append your state, so leave these empty */
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, 0);
}

/* ============================================================================================== *
 * Set sample times
 * ============================================================================================== */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);   /* This usually works, do not modify */
    ssSetOffsetTime(S, 0, 0.0);
}

#undef MDL_INITIALIZE_CONDITIONS

/* ============================================================================================== *
 * Set outputs as a function of the states
 * ============================================================================================== */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    /* rename outputs */
    real_T *aligningTorque=ssGetOutputPortRealSignal(S,0);   
    real_T *tp=ssGetOutputPortRealSignal(S,1);     
    real_T *Fy=ssGetOutputPortRealSignal(S,2);     
    real_T *alphaSlide=ssGetOutputPortRealSignal(S,3);    
    real_T *alpha_f=ssGetOutputPortRealSignal(S,4);
    
    /*ssGetInputPortRealSignalPtrs returns a pointer referencing where the inputs are stored */
    real_T delta=**ssGetInputPortRealSignalPtrs(S,0); /*composite steering angle*/
    real_T Vx=**ssGetInputPortRealSignalPtrs(S,1);
    real_T r=**ssGetInputPortRealSignalPtrs(S,2);
    real_T Vy=**ssGetInputPortRealSignalPtrs(S,3);
    
    /* rename 5 parameters */
    /* ssGetSFcnParam returns parameters as a complex number, mxGetPr gets the real part */
    const real_T tp0=*mxGetPr(ssGetSFcnParam(S,0));
    const real_T Cf=*mxGetPr(ssGetSFcnParam(S,1));
    /*Fz_f is normal force for one tire*/
    const real_T Fz_f=*mxGetPr(ssGetSFcnParam(S,2));
    const real_T tm=*mxGetPr(ssGetSFcnParam(S,3));
    const real_T a=*mxGetPr(ssGetSFcnParam(S,4));
    
    /*assume mu is constant for now*/
    mu=.7;
    
    /*create offset so that Vx is never zero*/
    e=.4;
    
    /*inverse of peak lateral force for front tires combined*/
    If=1/(mu*Fz_f*2);
    
    if (fabs(Vx)<0.1)
    {   *alpha_f=0;}
    else
    {   *alpha_f=-(atan((Vy+a*r)/(Vx+e))-delta);}
    
    *alphaSlide=atan(3/(Cf*If));
    *tp=GetPneumaticTrail(If, tp0, Cf, *alpha_f, *alphaSlide);
    *Fy=GetLateralForce(If, Cf, *alpha_f, *alphaSlide);
    *aligningTorque=GetAligningTorque(*tp, *Fy, tm);
}


/*================*
 * help functions *
 *================*/

/* Calculate pneumatic trail using linear model*/
static real_T GetPneumaticTrail(real_T If, real_T tp0, real_T Cf, real_T alpha_f, real_T aslide)
{
    c0=tp0;
    c1=-(tp0*Cf)/3;
    
    if (fabs(alpha_f)<aslide)
    { return c0+c1*If*fabs(tan(alpha_f));}
    else
    {    return 0;}
    
}

/* Calculate lateral force using Fiala model*/
static real_T GetLateralForce(real_T If, real_T Cf, real_T alpha_f, real_T aslide)
{
    c2=-Cf;
    c3=pow(Cf,2)/3;
    c4=-pow(Cf,3)/27;
    
    if (fabs(alpha_f)<aslide)
    { return c2*tan(alpha_f)+c3*fabs(tan(alpha_f))*tan(alpha_f)*If+c4*(pow(tan(alpha_f),3))*(pow(If,2));}
    else
    { return -(alpha_f/fabs(alpha_f))/If;
    }
}

/* Calculate aligning torque assuming a constant mechanical trail */
static real_T GetAligningTorque(real_T tp, real_T Fy, real_T tm)
{
    return -(tm+tp)*Fy;
}

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
