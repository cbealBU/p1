/*
 * File: discrete_bike_nonlinear.c
 *
 * Author: Carrie Bobier
 * (C)2009 Dynamic Design Laboratory -- Stanford University
 *
 * $Revision: 1$ $Date: September 16, 2009 $
 *
 * This file implements an s-function version of a discrete bike model with
 * nonlinear tires.  
 * 
 *	The model inputs are
 *		*uPtrs[0] = speed in m/s
 *		*uPtrs[1] = steering angle in RADIANS
 * The model outputs are
 *		y[0] = slip angle at CG in RADIANS
 *		y[1] = yaw rate (rad/s)
 *      y[2] = Beta_dot
 *      y[3] = r_dot
 */

#define S_FUNCTION_NAME  discrete_bike_nonlinear
#define S_FUNCTION_LEVEL 2

#include <math.h>
#include "simstruc.h"                           /* need this for SimStruct later */

/* parameter list:
 * a, b, CalphaF, CalphaR, Fzf, Fzr, mu, mu_s, T, m, Iz
 */

#define NUMOFINPUTS  2 	 
#define NUMOFOUTPUTS 4	 
#define NUMOFSTATES	 2
#define NUMOFPARAMS  11

#define g         9.81
#define minVx       2 //4.5

static int INPUTWIDTHS[]={1,1};       
static int OUTPUTWIDTHS[]={1,1,1,1};

// Helper Functions ========================================================================== *

static real_T getFialaTireForce(real_T Calpha, real_T alpha, real_T Fz, real_T mu, real_T mu_s){
    
    real_T tan_alpha;
    real_T Fy;
    
    // Define alpha slip
    const real_T alpha_slip = atan2(3*mu*Fz, Calpha);
    
    // If tire is not sliding
    if (fabs(alpha) < alpha_slip) {
        tan_alpha = tan(alpha);
        Fy = -Calpha*tan_alpha + pow(Calpha,2)*(2 - mu_s/mu)*fabs(tan_alpha)*tan_alpha/(3*mu*Fz) - pow(Calpha,3)*pow(tan_alpha,3)*(1-2*mu_s/(3*mu))/(9*pow(mu,2)*pow(Fz,2));
    }
    // If tire is sliding
    else {
        if (alpha > 0) {
            Fy = -mu_s*Fz;
        }
        else if (alpha < 0) {
            Fy = mu_s*Fz;
        }
    }
    // Return tire force value
    return Fy;    
}

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

/* ============================================================================================== *
 * Initialize block
 * ============================================================================================== */
#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
      real_T  *x = ssGetRealDiscStates(S);
      x[0] = 0;
      x[1] = 0;
  }
#endif /* MDL_INITIALIZE_CONDITIONS */

/* ============================================================================================== *
 * Get model outputs
 * ============================================================================================== */
/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block. Generally outputs are placed in the output vector, ssGetY(S).
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T Bdot, rdot, alphaR, alphaF, Fyf, Fyr;
    
    // y = Ix (feed states through as outputs)
    real_T  *x = ssGetRealDiscStates(S);
    real_T  *y = ssGetOutputPortSignal(S,0);
    real_T   B = x[0];
    real_T   r = x[1];
      
    // Define current inputs - 
    // ssGetInputPortRealSignalPtrs() returns a pointer 
    // referencing where the inputs are stored 
    const real_T Vx = **ssGetInputPortRealSignalPtrs(S,0);
    const real_T d  = **ssGetInputPortRealSignalPtrs(S,1);
    
    // Define parameters
    // ssGetSFcnParam() returns parameters as a complex number, 
    // mxGetPr() gets the real part
    const real_T a        = *mxGetPr(ssGetSFcnParam(S,0));
    const real_T b        = *mxGetPr(ssGetSFcnParam(S,1));
    const real_T CalphaF  = *mxGetPr(ssGetSFcnParam(S,2));
    const real_T CalphaR  = *mxGetPr(ssGetSFcnParam(S,3));
    const real_T Fzf      = *mxGetPr(ssGetSFcnParam(S,4));
    const real_T Fzr      = *mxGetPr(ssGetSFcnParam(S,5));
    const real_T mu       = *mxGetPr(ssGetSFcnParam(S,6));
    const real_T mu_s     = *mxGetPr(ssGetSFcnParam(S,7));
    const real_T T        = *mxGetPr(ssGetSFcnParam(S,8));
    const real_T m        = *mxGetPr(ssGetSFcnParam(S,9));
    const real_T Iz       = *mxGetPr(ssGetSFcnParam(S,10));    
      
    // Find front and rear slip angles from previous states
    alphaF = -d + atan2(B*Vx + a*r, Vx);
    alphaR = atan2(B*Vx - b*r, Vx);
      
    // Find fiala tire forces
    //Fyf = getFialaTireForce(CalphaF, alphaF, Fzf, mu, mu_s);
    //Fyr = getFialaTireForce(CalphaR, alphaR, Fzr, mu, mu_s);
    Fyf = -CalphaF*alphaF;
    Fyr = -CalphaR*alphaR;
      
    // Solve for sideslip angle
    Bdot = (Fyf*cos(d) + Fyr)/m/Vx - r;
    // Solve for yaw rate
    rdot = (a*Fyf*cos(d) - b*Fyr)/Iz;
    
    // Limit r
    if (r > mu*g/Vx){
      r = mu*g/Vx;
    }
    else if (r < -mu*g/Vx){
      r = -mu*g/Vx;
    }
    
    // Set outputs
    y[0] = x[0];
    y[1] = r;
    
    if (Vx > minVx){    
        y[2] = Bdot;
        y[3] = rdot; 
    }
    else{
        y[2] = 0;
        y[3] = 0;
    }
}

/* ============================================================================================== *
 * update discrete states
 * ============================================================================================== */
#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
      real_T alphaR, alphaF, Fyf, Fyr;
      
      // Define states from last time step
      real_T *x = ssGetRealDiscStates(S);
      real_T B = x[0];
      real_T r = x[1];
      
      // Define current inputs - 
      // ssGetInputPortRealSignalPtrs() returns a pointer 
      // referencing where the inputs are stored 
      const real_T Vx = **ssGetInputPortRealSignalPtrs(S,0);
      const real_T d  = **ssGetInputPortRealSignalPtrs(S,1);
     
      // Define parameters
      // ssGetSFcnParam() returns parameters as a complex number, 
      // mxGetPr() gets the real part
      const real_T a        = *mxGetPr(ssGetSFcnParam(S,0));
      const real_T b        = *mxGetPr(ssGetSFcnParam(S,1));
      const real_T CalphaF  = *mxGetPr(ssGetSFcnParam(S,2));
      const real_T CalphaR  = *mxGetPr(ssGetSFcnParam(S,3));
      const real_T Fzf      = *mxGetPr(ssGetSFcnParam(S,4));
      const real_T Fzr      = *mxGetPr(ssGetSFcnParam(S,5));
      const real_T mu       = *mxGetPr(ssGetSFcnParam(S,6));
      const real_T mu_s     = *mxGetPr(ssGetSFcnParam(S,7));
      const real_T T        = *mxGetPr(ssGetSFcnParam(S,8));
      const real_T m        = *mxGetPr(ssGetSFcnParam(S,9));
      const real_T Iz       = *mxGetPr(ssGetSFcnParam(S,10));    
      
      if (Vx > minVx){
          // Find front and rear slip angles from previous states
          alphaF = -d + atan2(B*Vx + a*r, Vx);
          alphaR = atan2(B*Vx - b*r, Vx);

          // Find fiala tire forces
          //Fyf = getFialaTireForce(CalphaF, alphaF, Fzf, mu, mu_s);
          //Fyr = getFialaTireForce(CalphaR, alphaR, Fzr, mu, mu_s);
          Fyf = -CalphaF*alphaF;
          Fyr = -CalphaR*alphaR;

          // Solve for sideslip angle
          x[0] = B + T*(Fyf*cos(d) + Fyr)/m/Vx - T*r;
          // Solve for yaw rate
          x[1] = r + T*(a*Fyf*cos(d) - b*Fyr)/Iz;
      }
      else{
          x[0] = 0;
          x[1] = 0;
      }
  }
#endif /* MDL_UPDATE */

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