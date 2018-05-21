/*
 * File : longitudinalControllerSlipCircle.c
 * Modified from longitudinalControllerSlipCircle.c
 *
 * Abstract:
 *       C-file S-function to decide what should be the steering input, if the slip 
 * of any of the tire go beyond the slip circle, then hold the steering and wait until 
 * the longitudinal brake until bring the slip back into the circle before decide to 
 * steer as normal.
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
 * Created on December 19,09
 * $Revision: 1.0 $
 */

#define S_FUNCTION_NAME  holdingSteering
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"
#include <stdio.h>

#define NUM_PARAMS  (0)
#define NUMOFINPUTS (10)
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
    
    double alphaFront, alphaRear, kappaFront, kappaRear, steeringCommand, previousDeltaCommand;
    double alphaFrontLimit, alphaRearLimit, kappaFrontLimit, kappaRearLimit;
    double kappaRearRatio, alphaRearRatio, deltaKappaRear, deltaOutput;
    
    #define pi (3.1416)
                    
    alphaFront=*in[0];  // front alpha slip [rad]
    alphaRear=*in[1];  // rear alpha slip [rad]
    kappaFront=*in[2];  // front kappa [unitless]
    kappaRear=*in[3];  // rear kappa [unitless]
    steeringCommand=*in[4];  // current delta command from the controller [rad]
    previousDeltaCommand=*in[5];  // previous delta command from the controller at road wheel [rad]
    alphaFrontLimit=*in[6];  // maximum allowable front alpha slip [rad]
    alphaRearLimit=*in[7];  // maximum allowable rear alpha slip [rad]
    kappaFrontLimit=*in[8];  // maximum allowable front kappa [rad]
    kappaRearLimit=*in[9];  // maximum allowable front kappa [rad]
    
    // alphaRearRatio=alphaRear/alphaRearLimit;
    // kappaRearRatio=kappaRear/kappaRearLimit;
    
    if ( fabs(alphaFront)>alphaFrontLimit )
    // front wheels are sliding
    {
        deltaOutput=previousDeltaCommand;   // hold the steering
    }
    else    // front tires are not saturated
    {
        // just come out from saturation, prevent from delta have big jump.  i.e. limit slew rate
        if ( fabs(fabs(previousDeltaCommand)-fabs(steeringCommand))> 1*pi/180 )
        {
            //fix slew rate to .25 deg/sample
            if ( steeringCommand-previousDeltaCommand > 0 )
            {
                deltaOutput=previousDeltaCommand+.25*pi/180; 
            }
            else
            {
                deltaOutput=previousDeltaCommand-.25*pi/180;
            }
        }
        // everything is OK, pass the steering command through
        else
        {
            deltaOutput=steeringCommand;   
        }

    }
               
    
    // output
    Y[0]=deltaOutput;        //Output steering command [rad]
    
    
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
