/* 
 * File: inbounds_control
 *
 * Author: Carrie Bobier
 * (C)2011 Dynamic Design Laboratory -- Stanford University
 *
 * $Revision: 1$ $Date: May 5, 2011 $
 *
 * This file implements an s-function to calculate the proportional control
 * law inside of the isocline safe area 'triangles'
 * 
 *	The model inputs are
 *		*uPtrs[0] = yaw rate in RADIANS/S
 *		*uPtrs[1] = sideslip in RADIANS
 *      *uPtrs[2] = handwheel steering angle in RADIANS
 *      *uPtrs[3] = speed in METER/S
 *      *uPtrs[4] = SSC enable in BINARY
 *      *uPtrs[5] = accelerator pedal input in VOLTS
 * The model outputs are
 *      y[0] = max angle of the p-control triangle in RADIANS;
 *      y[1] = current states' angle inside the triangle in RADIANS;
 *      y[2] = commanded steering angle in RADIANS;
 *      y[3] = proportional control enable signal in BINARY;
 */

#define S_FUNCTION_NAME  inbounds_control
#define S_FUNCTION_LEVEL 2

#include <math.h>
#include "simstruc.h"                           /* need this for SimStruct later */

/* parameter list:
 * a, b, CalphaF, CalphaR, Fzf, Fzr, mu, mu_s
 */

#define NUMOFINPUTS  6 	 
#define NUMOFOUTPUTS 4	 
#define NUMOFSTATES	 0
#define NUMOFPARAMS  8

#define g         9.81

static int INPUTWIDTHS[]={1,1,1,1,1,1};       
static int OUTPUTWIDTHS[]={1,1,1,1};

// Helper Functions ========================================================================== *


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
    real_T Fyf, Fyr, Fxr, max_Fyr;
    real_T phi_safe, phi_current, d_control, enablePC;
    real_T alpha_slf, alpha_slr, dmax, B_C, r_C, B_D, r_D;
    
    // y = Ix (feed states through as outputs)
    real_T  *y = ssGetOutputPortSignal(S,0);

    // Define current inputs - 
    // ssGetInputPortRealSignalPtrs() returns a pointer 
    // referencing where the inputs are stored 
    const real_T r         = **ssGetInputPortRealSignalPtrs(S,0);
    const real_T B         = **ssGetInputPortRealSignalPtrs(S,1);
    const real_T d         = **ssGetInputPortRealSignalPtrs(S,2);
    const real_T Vx        = **ssGetInputPortRealSignalPtrs(S,3);
    const real_T enableSSC = **ssGetInputPortRealSignalPtrs(S,4);
    const real_T pedal     = **ssGetInputPortRealSignalPtrs(S,5);

    
    // Define parameters
    // ssGetSFcnParam() returns parameters as a complex number, 
    // mxGetPr() gets the real part
    const real_T a              = *mxGetPr(ssGetSFcnParam(S,0));
    const real_T b              = *mxGetPr(ssGetSFcnParam(S,1));
    const real_T CalphaF        = *mxGetPr(ssGetSFcnParam(S,2));
    const real_T CalphaR        = *mxGetPr(ssGetSFcnParam(S,3));
    const real_T Fzf            = *mxGetPr(ssGetSFcnParam(S,4));
    const real_T Fzr            = *mxGetPr(ssGetSFcnParam(S,5));
    const real_T mu             = *mxGetPr(ssGetSFcnParam(S,6));
    const real_T mu_s           = *mxGetPr(ssGetSFcnParam(S,7));
    //const real_T m              = *mxGetPr(ssGetSFcnParam(S,8));
    //const real_T Iz             = *mxGetPr(ssGetSFcnParam(S,9));  
    
    // Find the rear longitudinal tire forces
    
    if (pedal >= 1.7){
        Fxr = 5.5*2*190.5*(pedal-1.7)/(2.8-1.7)/.3; //divide by pedal range and tire radius
    }
    else {
        Fxr = 5.5*2*70.5*(pedal-1.7)/(0.9-1.7)/.3;
    }

/*
    if (pedal >= 1.9){
        Fxr = 5.5*2*210*(pedal-1.9)/(2.8-1.9)/.3; //divide by pedal range and tire radius
    }
    else {
        Fxr = 5.5*2*90*(pedal-1.7)/(0.9-1.9)/.3;
    }
*/
    // Get the maximum available rear lateral tire force
    if (Fxr > Fzr){
        Fxr = Fzr;
    }
    if (Fxr < -Fzr){
        Fxr = -Fzr;
    }
    max_Fyr = pow((Fzr*Fzr - Fxr*Fxr), .5);
    
    // Set outputs to do nothing if controller is off
    phi_safe = 0;
    phi_current = 0;
    d_control = d;
    enablePC = 0;

    // Solve for controller action inside triangle
    if (Vx > 2){
        // We don't want this controller to activate when we are outside 
        // the safe boundaries because SSC will be on then
        if(enableSSC == 0) {
        
            // define quantities needed for yaw rate (triangle) boundaries
            alpha_slf = atan2(3*mu*Fzf, CalphaF);                           // max front slip angle
            alpha_slr = atan2(3*mu*Fzr, CalphaR);                       // max rear slip angle
            dmax = atan((a+b)*mu*g/Vx/Vx - tan(alpha_slr)) + alpha_slf;     // max stable delta
            B_C = b*mu*g/Vx/Vx - tan(alpha_slr);                            // upper left corner of safe area                      
            r_C = mu*g/Vx;                                                  // upper left corner of safe area 
            B_D = (b/(a+b))*(tan(alpha_slf+dmax) - tan(alpha_slr)) + tan(alpha_slr); // upper right corner of safe area
            r_D = (Vx/(a+b))*(tan(alpha_slf+dmax) - tan(alpha_slr));       // upper right corner of safe area
            
            // check if the driver's steering command is too big for the current speed
            if (fabs(d) > fabs(dmax)) {
                
                // Check if we are above the steady state yaw rate bound
                if(r > mu*g/Vx) {
                    // Find the car's angle through the safe area triangle
                    phi_safe = acos( (B_D-B_C) / sqrt((B_D-B_C)*(B_D-B_C)+(r_D-r_C)*(r_D-r_C)) );
                    phi_current = acos( (B-B_C) / sqrt((B-B_C)*(B-B_C)+(r-r_C)*(r-r_C)) );
                    // Give a proportional control determined steering command
                    d_control = d + (phi_current/phi_safe)*(dmax-d);
                    // Turn on the proportional control enable switch
                    enablePC = 1;
                }
                if(r < -mu*g/Vx) {
                    // Find the car's angle through the safe area triangle
                    phi_safe = acos( (B_D-B_C) / sqrt((B_D-B_C)*(B_D-B_C)+(r_D-r_C)*(r_D-r_C)) );
                    phi_current = fabs(acos( (-B_C-B) / sqrt((-B_C-B)*(-B_C-B)+(r+r_C)*(r+r_C)) ));
                    // Give a proportional control determined steering command
                    d_control = d + (phi_current/phi_safe)*(-dmax-d);
                    // Turn on the proportional control enable switch
                    enablePC = 1;
                }
            }
        }
	}
    
    // Set outputs
    y[0] = phi_safe;
    y[1] = phi_current;
    y[2] = d_control;
    y[3] = enablePC;
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