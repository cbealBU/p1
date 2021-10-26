/*
 * File : FFWcontrolClothoid.c
 * Modified from FFWcontrol.c
 *
 * Abstract:
 *       C-file S-function to create feed forward throttle control for the P1 on the
 * Shoreline parking lot, oval race track
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
 * Created on Oct 9,08
 * $Revision: 1.0 $
 * This is the hack version, only use constant radius, didn't use information that David Gave yet
 */

#define S_FUNCTION_NAME  FFWcontrolClothoid
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"
#include <stdio.h>

#define NUM_PARAMS                  	(5)
#define NUMOFINPUTS (2)
#define NUMOFOUTPUTS (1)
#define gravity (9.81)

/*==================*
 * Global Variables *
 *==================*/
//Global variable (preserved between time steps) to see what is the current state of the FFW speed
static int_T FFWspeedState;

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
static void mdlInitializeSizes(SimStruct *S)
{
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
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}


//Set FFW speed state to 0
#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
static void mdlInitializeConditions(SimStruct *S) {
    FFWspeedState = 0;
}


/* Function: mdlOutputs =======================================================
 * Abstract:
 *    Filter out any error in the sensor read out
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    
    real_T *Y = ssGetOutputPortRealSignal(S,0);
    
    InputRealPtrsType in = ssGetInputPortRealSignalPtrs(S,0);
    
    // Extracting vehicle parameters
    const real_T Caf = mxGetPr(ssGetSFcnParam(S,0))[0];  // front axle cornering stiffness
    const real_T Car = mxGetPr(ssGetSFcnParam(S,1))[0];  // rear axle cornering stiffness
    const real_T mass = mxGetPr(ssGetSFcnParam(S,2))[0];  // vehicle total mass
    const real_T a = mxGetPr(ssGetSFcnParam(S,3))[0];  // distance from front axle to CG
    const real_T b = mxGetPr(ssGetSFcnParam(S,4))[0];  // distance from rear axle to CG
    const real_T L = a+b;  // wheelbase
    
    double e, deltaPsi, curvature, steeringAngle, FFWspeed, FFWacceleration, targetSpeed, dist2nextSegment, lengthNextSegment;
    double startCurvature, endCurvature;
    double understeer_gradient, Wf, Wr, vxCG;
    
    curvature=*in[0];   // [1/m]
    vxCG=*in[1];        // vehicle speed at CG [m/s]
    
    Wf=(mass*gravity*b)/(a+b);  // front weight [N]
    Wr=(mass*gravity*a)/(a+b);  // rear weight [N]
    understeer_gradient=Wf/Caf-Wr/Car;
             
    // steering control
    steeringAngle=(L+(understeer_gradient*pow(vxCG,2)/gravity) )*curvature;
    
    Y[0]=steeringAngle;            //Output steering angle at the road wheel [rad]
    
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
