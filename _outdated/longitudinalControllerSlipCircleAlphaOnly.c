/*
 * File : longitudinalControllerSlipCircleAlphaOnly.c
 * Modified from longitudinalControllerSlipCircle.c
 *
 * Abstract:
 *       C-file S-function to create longitudinal controller for the P1 to controller the Fx force.
 * The feedback Fx decides what to do and how much to do depends on the condition of the vehicle, i.e. 
 * is the front wheels are sliding or is the rear wheels are sliding.  The controller will try to make sure
 * that the slip of the tires are within the slip circle.  
 * The different between this longitudinalControllerSlipCircleAlphaOnly and longitudinalControllerSlipCircle is that
 * this controller only care about alpha slip and doesn't take kappa into account
 *
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

#define S_FUNCTION_NAME  longitudinalControllerSlipCircleAlphaOnly
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"
#include <stdio.h>

#define NUM_PARAMS  (4)
#define NUMOFINPUTS (11)
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
    
    // Extracting vehicle parameters
    const real_T gainFrontAlphaSlip = mxGetPr(ssGetSFcnParam(S, 0))[0];  // gain for front alpha slip [Volt/rad]
    const real_T gainRearKappaSlip = mxGetPr(ssGetSFcnParam(S, 1))[0];  // gain for front kappa [Volt]
    const real_T gainRearAlphaSlip = mxGetPr(ssGetSFcnParam(S, 2))[0];  // gain for rear alpha slip [Volt/rad]
    const real_T gainRearNoneSlip = mxGetPr(ssGetSFcnParam(S, 3))[0];  // gain for rear when none of the tires are slipping [Volt/rad]
        
    double vxCG, alphaFront, alphaRear, kappaFront, kappaRear, FBdeltaAlphaLimited, segType;
    double alphaFrontLimit, alphaRearLimit, kappaFrontLimit, kappaRearLimit;
    double kappaRearRatio, alphaRearRatio, deltaKappaRear, FxCommand;
    
    #define frontScalingLimit 1.5*3.1416/180  // the different before the saturation limit activate.
            // if the different between the front tire slip and the front alpha limit is less than this value, than scale it to prevent jump in Fx command when it switch from positive to negative
            // But if exceeding this value, than don't scale it
            // reason why we need to do this is because we control the slip of the rear wheel, but if it's suddenly switch to
            // front wheel sliding, then there will be jump in the command.  thus, this scaling will minimise the jump

    //initialise FxCommand, want to initialise this every time step
    FxCommand=0;   
    
    vxCG=*in[0];  // vehicle Vx
    alphaFront=*in[1];  // front alpha slip [rad]
    alphaRear=*in[2];  // rear alpha slip [rad]
    kappaFront=*in[3];  // front kappa [unitless]
    kappaRear=*in[4];  // rear kappa [unitless]
    FBdeltaAlphaLimited=*in[5];  // feedback steering based on constraining front slip angle [rad]
    segType=*in[6];  // Type of segment
    alphaFrontLimit=*in[7];  // maximum allowable front alpha slip
    alphaRearLimit=*in[8];  // maximum allowable rear alpha slip
    kappaFrontLimit=*in[9];  // maximum allowable front kappa
    kappaRearLimit=*in[10];  // maximum allowable front kappa
        
    if ( fabs(alphaFront)>alphaFrontLimit || fabs(FBdeltaAlphaLimited)>0 || fabs(alphaRear)>alphaRearLimit) //front wheel or all wheels are sliding
    {
        if ( fabs(alphaFront)>alphaFrontLimit || fabs(FBdeltaAlphaLimited)>0 ) // front tires slide or four wheels slide
        {
            // feedback based on how much front tires slide
            FxCommand=-gainFrontAlphaSlip*( (fabs(alphaFront)-alphaFrontLimit)+fabs(FBdeltaAlphaLimited) );
        }
        else   // rear tires slip
        {
            // always brake
            FxCommand=-gainRearAlphaSlip*( fabs(alphaRear)-alphaRearLimit );
        }
    }
    else    // none of the tires are saturated
    {
        if (segType == 0){  // in a stright section, trust the feedforward and do not brake or accerelate more
            FxCommand=0;
        }
        else // not in a straight line
        {
            // either let off brake more or accerelate more
            if ( fabs( alphaFrontLimit-fabs(alphaFront) )> frontScalingLimit ){ // no scaling
                FxCommand=gainRearNoneSlip*( alphaRearLimit-fabs(alphaRear) ); // this could cause the car to drift as it will try to maintain rear alpha at alphaRearLimit degree
            }
            else{   // smooth out the command to avoid any jump when front tires start to saturate
                FxCommand=(fabs( alphaFrontLimit-fabs(alphaFront) )/frontScalingLimit )*( gainRearNoneSlip*( alphaRearLimit-fabs(alphaRear) ));
            }
        }
    }     
    
    // output
    Y[0]=FxCommand;        //Output Force feedback [should be N, but this unit is in Volt for P1]
    
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
