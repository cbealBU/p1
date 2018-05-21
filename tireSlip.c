/*
 * File : tireSlip.c
 * Modified from longitudinalController_psi.c
 *
 * Abstract:
 *       C-file S-function to calculate front and rear axle lateral slips (alpha), unit in radian.
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

#define S_FUNCTION_NAME  tireSlip
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"
#include <stdio.h>

#define NUM_PARAMS                  	(2)
#define NUMOFINPUTS (6)
#define NUMOFOUTPUTS (4)

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
    const real_T a = mxGetPr(ssGetSFcnParam(S, 0))[0];  // distance from front axle to CG [m]
    const real_T b = mxGetPr(ssGetSFcnParam(S, 1))[0];  // distance from rear axle to CG [m]
    
    double sideSlip, yawRate, vxCG, delta;
    double alphaFront, alphaRear, kappaFront, kappaRear, vxFrontWheel, vxRearWheel, vxFrontAxle;
    
    sideSlip=*in[0];  // side Slip [rad]
    yawRate=*in[1];  // yaw Rate [rad/s]
    vxCG=*in[2];  // vehicle speed [m/s]
    delta=*in[3];  // delta [rad]
    vxFrontWheel=*in[4];  // front wheel speed [m/s]
    vxRearWheel=*in[5];  // rear wheel speed [m/s]
    
//    if (vxRearWheel > 1)   // calculation only work when driven wheel (rear for P1) speed of the vehicle > 1m/s
    if (vxCG > 1)   // calculation only work when speed of the vehicle > 1m/s
    {
        // finding front slip angle [rad]
        alphaFront = atan(sideSlip + a*yawRate/(vxCG+0.00001)) - delta;   
        // finding rear slip angle [rad]
        alphaRear = atan(sideSlip - b*yawRate/(vxCG+0.00001));
        kappaRear=(vxCG-vxRearWheel)/(vxCG+0.00001);
        // include effect of wheel turning
        vxFrontAxle=vxCG*cos(delta)+a*yawRate*sin(delta);
        kappaFront=(vxFrontAxle-vxFrontWheel)/(vxFrontAxle+0.00001);
    }
    else
    {
        alphaFront=0;
        alphaRear=0;
        kappaFront=0;
        kappaRear=0;

    }
    
    
    // output
    Y[0]=alphaFront;        //Front Lateral Slip [rad]
    Y[1]=alphaRear;        //Rear Lateral Slip [rad]
    Y[2]=kappaFront;        //Front Longitudinal Slip 
    Y[3]=kappaRear;        //Rear Longitudinal Slip 
    

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
