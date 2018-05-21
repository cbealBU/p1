/*
 * File : FBsteeringLimitAlpha.c
 * Modified from tireSlip.c
 *
 * Abstract:
 *       C-file S-function to calculate amount of feedback steering to limit the slip angle of the front axle.
 * Whatever the front alpha slip read, if it exceeds the limit, the front steering will be reduced to maintain 
 * the amount of maximum slip angle.
 *      All units are in radian
 *      Function take front slip angle and the limit in and spit out steering wheel correction
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
  * $Revision: 1.0 $
 */

#define S_FUNCTION_NAME  FBsteeringLimitAlpha
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"
#include <stdio.h>

#define NUM_PARAMS                  	(0)
#define NUMOFINPUTS (2)
#define NUMOFOUTPUTS (1)

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
       
    double alphaFront, alphaFrontLimit, FBdeltaAlpha, deltaSteering;
    
    alphaFront=*in[0];  // Front Slip [rad]
    alphaFrontLimit=*in[1];  // Front slip limit [rad]
  
    if (fabs(alphaFront) > alphaFrontLimit)   // Feedback delta only work when alphafront  > alpha front limit
    {
        deltaSteering=fabs(alphaFront)-alphaFrontLimit;
        if (alphaFront < 0)
        {
            FBdeltaAlpha=-deltaSteering; // car tunring left, reduce steering by turning right
        }
        else
        {
            FBdeltaAlpha=deltaSteering; // car tunring right, reduce steering by turning left
        }
   
    }
    else
    {
        FBdeltaAlpha=0;
    }
    
    
    // output
    Y[0]=FBdeltaAlpha;        //Feedback front wheel delta [rad] 
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
