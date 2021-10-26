/*
 * File : longitudinalController_psi.c
 * Modified from TTSlongitudinalController_psi.c
 *
 * Abstract:
 *       C-file S-function to create longitudinal controller for the TTS to controller the torque.
 *  Torque in this case is a loosely word.  If torque is positive, mean we want more torque (accerelate), if we
 *  want less torque, that means we want decelerate, could be from letting off the throttle or brake.  The concept
 * of this controller is based on the feedback of the lanekeeping heading angle error.  The
 * target of this psi should cycle around 0 rad, if this is more, then car should slow down, understeer, but
 * if this is less, floor the throttle
 *
 * Real-Time Workshop note:
 *   This file can be used as is (noninlined) with the Real-Time Workshop
 *   C rapid prototyping targets, or it can be inlined using the Target
 *   Language Compiler technology and used with any target. See
 *     matlabroot/toolbox/simulink/blocks/tlc_c/timestwo.tlc
 *     matlabroot/toolbox/simulink/blocks/tlc_ada/timestwo.tlc
 *   the C and Ada TLC code to inline the S-function.
 *
 * See simulink/src/sfuntmpl_doc.c
 *
 * By Mick
 * Created on December 11,08
 * Modified on September 30,09
 * $Revision: 1.0 $
 */

#define S_FUNCTION_NAME  longitudinalController_psi
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"
#include <stdio.h>

#define NUM_PARAMS                  	(2)
#define NUMOFINPUTS (7)
#define NUMOFOUTPUTS (1)
#define gravity (9.81)
#define pi (3.1416)

/*==================================================*
 * Macros to access the S-function parameter values *
 *==================================================*/

/*================*
 * Build checking *
 *================*/


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumSFcnParams(S, NUM_PARAMS);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }
    
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, NUMOFINPUTS);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, NUMOFOUTPUTS);
    
    ssSetNumSampleTimes(S, 1);
    
    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE |
            SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}


/* Function: mdlOutputs =======================================================
 * Abstract:
 *    Filter out any error in the sensor read out
 */
static void mdlOutputs(SimStruct *S, int_T tid) {
    
    real_T *Y = ssGetOutputPortRealSignal(S, 0);
    
    InputRealPtrsType in = ssGetInputPortRealSignalPtrs(S, 0);
    
    // Extracting vehicle parameters
    const real_T delta_psi_threshold = mxGetPr(ssGetSFcnParam(S, 0))[0];  // threshold for enabling throttle [rad]
    const real_T gain_torque = mxGetPr(ssGetSFcnParam(S, 1))[0];  // gain for the throttle controller [Volt/rad]
    
    double Vx, sideSlip, yawRate, delta, torqueCommand, FFWdelta;
    double delta_psi, segType;
    
    Vx=*in[0];  // vehicle Vx
    sideSlip=*in[1];  // side Slip [rad]
    yawRate=*in[2];  // yaw Rate [rad/s]
    delta=*in[3];  // delta [rad]
    FFWdelta=*in[4];  // Feed Forward delta [rad]
    delta_psi=*in[5];  // Lane Keeping delta [rad]
    segType=*in[6];  // segment Type, if not =0, then vehicle is cornering
    
    // this controller only work when the vehicle is turning, if it's not turning, don't activate
    if ( segType == 0 ) // Vechicle still going straight, no speed feedback
    {
        // calculate command
        torqueCommand=0;
    }
    else    // vehicle is turning
    {
        // throttle control
        
        if (FFWdelta >= 0) // vehicle is turning left, sideslip is negative
        {
            torqueCommand=( delta_psi-delta_psi_threshold )*gain_torque;
        }
        else    // vehicle is turning right, sideslip is positive
        {
            torqueCommand=(-delta_psi+delta_psi_threshold )*gain_torque;
        }
        
    }
    
    // output
    Y[0]=torqueCommand;        //Output torque feedback [N.m]
    
    
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S) {
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
