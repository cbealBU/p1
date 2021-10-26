/*
 * File : FFWstraight.c
 * Modified from longitudinalController_psi.c
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

#define S_FUNCTION_NAME  FFWstraight
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"
#include <stdio.h>

#define NUM_PARAMS                  	(2)
#define NUMOFINPUTS (3)
#define NUMOFOUTPUTS (1)
#define NUM_I_WORK   1

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
    
    // Set the number of work variables (static module-variables)
    ssSetNumIWork(S, NUM_I_WORK);
    
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


//Set last brake flag to 0
static void mdlInitializeConditions(SimStruct *S) {
    // Get static variables as pointers, so we can update them
    int_T *lastBrakeFlag = ssGetIWork(S);
    
    *lastBrakeFlag = 0;
}



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    Filter out any error in the sensor read out
 */
static void mdlOutputs(SimStruct *S, int_T tid) {
    
    #define maxFx (1)
    #define minFx (-1)
    
    real_T *Y = ssGetOutputPortRealSignal(S, 0);
    
    InputRealPtrsType in = ssGetInputPortRealSignalPtrs(S, 0);
    
    // Extracting vehicle parameters
    const real_T max_axRegen = mxGetPr(ssGetSFcnParam(S, 0))[0];  // maximum regenerative brake, negative value [m/s^2]
    const real_T maxUxRegen = mxGetPr(ssGetSFcnParam(S, 1))[0];  // maximum entry speed [m/s]
    
    double Vx, dstNextSegment,segType,FxCommand;
    int brakeFlag;
    
    // Get static variables as pointers, so we can update them
    int_T *lastBrakeFlag = ssGetIWork(S);
    
    Vx=*in[0];  // vehicle Vx
    dstNextSegment=*in[1];  // distance to next segment [m]
    segType=*in[2];  // segment Type, if not =0, then vehicle is cornering
    
    brakeFlag = *lastBrakeFlag;
            
    // this controller only work when the vehicle is turning, if it's not turning, don't activate
    if ( segType == 0 ) // Vechicle still going straight, no speed feedback
    {
        if ( brakeFlag == 1 )   //already command brake, continue to brake
        {
            FxCommand=minFx;
        }
        else    // vehicle is accerelating and wait to brake
        {
            // still pedal to the metal
            FxCommand=maxFx;
            
            // check if hitting the brake point
            if ( dstNextSegment < (maxUxRegen*maxUxRegen-Vx*Vx)/(2*max_axRegen) )   // distance to next segment < distance required for braking
            {
                *lastBrakeFlag = 1;
            }
            
        }
    
    }
    else    // vehicle is turning
    {
        // Update static variables 
        *lastBrakeFlag = 0;
    }

    // output
    Y[0]=FxCommand;        //Output FFW Fx [N]
    
    
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
