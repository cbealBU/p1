/*
 * File : FFWsteeringForceSS.c
 * Modified from FFWcontrolSS.c
 *
 * Abstract:
 *       C-file S-function to create feed forward steering using state space format.
 * The equation use bike model to calculate Uy (vyCG) and r (yaw rate).
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

#define S_FUNCTION_NAME  FFWsteeringForceSS
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"
#include <stdio.h>

#define NUM_PARAMS  (13)
#define NUMOFINPUTS (11)
#define NUMOFOUTPUTS (5)
#define gravity (9.81)
#define vThreshold (1)  // (m/s)
#define NUM_R_WORK   2

// #define MAP(S) ssGetSFcnParam(S, 6)
// #define numberSegment ((int)(mxGetPr(MAP(S))[5]))
// 
// #define SEG(S) ssGetSFcnParam(S, 5)
// #define length (mxGetPr(SEG(S))+numberSegment*3)     // [m] segment length, in vector format for each segment, start from segment 0
// #define startCurvature (mxGetPr(SEG(S))+numberSegment*5)     // [1/m] start curvature, in vector format for each segment, start from segment 0
// #define endCurvature (mxGetPr(SEG(S))+numberSegment*6)     // [1/m] end curvature, in vector format for each segment, start from segment 0

/*==================*
 * Global Variables *
 *==================*/
//Global variable (preserved between time steps) to see what is the current state of the FFW speed

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
    
    // Set the number of work variables (static module-variables)
    ssSetNumRWork(S, NUM_R_WORK);

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

#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove  function */
#if defined(MDL_INITIALIZE_CONDITIONS)

static void mdlInitializeConditions(SimStruct *S) {
    // Get static variables as pointers, so we can update them
    // set all initial values to zero
    ssSetRWorkValue(S, 0, 0);
    ssSetRWorkValue(S, 1, 0);
    ssSetRWorkValue(S, 2, 0);
}
#endif /* MDL_INITIALIZE_CONDITIONS */

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
    const real_T m = mxGetPr(ssGetSFcnParam(S, 2))[0];  // vehicle total mass
    const real_T a = mxGetPr(ssGetSFcnParam(S, 3))[0];  // distance from front axle to CG
    const real_T b = mxGetPr(ssGetSFcnParam(S, 4))[0];  // distance from rear axle to CG
    const real_T Iz = mxGetPr(ssGetSFcnParam(S, 5))[0];  // Inertia around CG in zz direction (kg*m^2)
    const real_T muPeak = mxGetPr(ssGetSFcnParam(S, 6))[0];  // mu peak
    const real_T muSlide = mxGetPr(ssGetSFcnParam(S, 7))[0];  // mu slide
    const real_T Ts = mxGetPr(ssGetSFcnParam(S, 8))[0];  // sampling time (s)
    const real_T L = a+b;  // wheelbase
    const real_T FzR = m*a*9.81/L;  // rear normal load (N)
    const real_T lam1 = mxGetPr(ssGetSFcnParam(S, 9))[0];//lambda1
    const real_T lam2 = mxGetPr(ssGetSFcnParam(S, 10))[0];//lambda2
    const real_T lam3 = mxGetPr(ssGetSFcnParam(S, 11))[0];//lambda3
    const real_T lam4 = mxGetPr(ssGetSFcnParam(S, 12))[0];//lambda4
    
    double curvature, slope;
    double axCG, sideSlip, yawRate, axDesired;
    double vxCG, sDot, sDotDot, vyCG, kDot, alphaRslide;
    double rSimDot, vyCGsimDot, rError, vyCGerror;  
    double vyCGsim, rSim, sDotDotSim, FFW_FyFsim, FFW_FyRsim, alphaRsim;    // simulation parameters
    double rPureSim;
    double vyCGpureSim;
    int segType, segNumber;
    double startCurvature, endCurvature, length;
    
    vxCG=*in[0];            // vehicle speed at CG [m/s]
    axCG=*in[1];            // vehicle acceleration at CG [m/s^2]
    sideSlip=*in[2];        // vehicle side slip at CG [rad]
    yawRate=*in[3];         // vehicle yaw rate [rad/s]
    curvature=*in[4];       // [1/m]
    segType=(int)*in[5];    // current segment type
    segNumber=(int)*in[6];  // current segment number
    startCurvature=*in[7];  // current segment start curvature
    endCurvature=*in[8];    // current segment end curvature
    length=*in[9];          // current segment length
    axDesired=*in[10];      // desired axCG
        
    // calculating state
    vyCG=vxCG*tan(sideSlip);
    sDot=vxCG;
    //sDotDot=axCG+vyCG*yawRate;
    
    // for simulation
    vyCGsim=ssGetRWorkValue(S, 0);  // get UyCG
    rSim=ssGetRWorkValue(S, 1);     // get r
    
    vyCGerror = vyCG - vyCGsim;
    rError = yawRate - rSim;
    
    if (vxCG > vThreshold){    // prevent from rear alpha blow up, when the car standstill
        alphaRsim=atan(vyCGsim/vxCG-b*rSim/vxCG);
    }
    else{
        alphaRsim=0;
    }
    
    //printf("alphaRsim %f, vxCG %f \n", alphaRsim*180/3.147,vxCG);
    
    // add logic in, so that it doesn't go crazy when car standstill
    if (vxCG < vThreshold){ // car doesn't move, so sDotDotSim should be zero
        sDotDotSim=0;
    }
    else{
        sDotDotSim=axDesired+vyCGsim*rSim;
    }
    
    // finding FyF 
    if (segType == 1 || segType == 3) {  // in clothoid
        //slope=(endCurvature[segNumber]-startCurvature[segNumber])/length[segNumber];    // how fast curvature change
        slope=(endCurvature-startCurvature)/length;    // how fast curvature change
        kDot=sDot*slope;        
    }
    else{
        kDot=0;
    }
    
    // calculating FFW force at front axle
    FFW_FyFsim=m*b/L*curvature*(sDot*sDot)+Iz/L*(curvature*sDotDotSim+kDot*sDot);
        
    //printf("FFW_FyFsim %f, curvature %f, sDot %f, sDotDotSim %f \n", FFW_FyFsim, curvature, sDot, sDotDotSim);
    //printf("kDot %f \n", kDot);
    
    // output value
    Y[0]=FFW_FyFsim;            //Output FFW force at the front axle [N]
    Y[1]=vyCGsim;            //Output Uy at CG [m/s]
    Y[2]=rSim;            //Output yaw rate (rad/s)
    //Y[3]=rError;         //Output Error between yaw rate measurement and estimate
    //Y[4] = vyCGerror;    //Output error between Uy measurement and estimate
    
    
    // calculating FFW force at rear axle, using nonlinear tire
    alphaRslide = fabs( atan(3*muPeak*FzR/Car) );
    if (fabs(alphaRsim)<alphaRslide){ //Not slide, use Fiala equations
        FFW_FyRsim = -Car*tan(alphaRsim)+(Car*Car)/(3*muPeak*FzR)*(2-muSlide/muPeak)*abs(tan(alphaRsim))*tan(alphaRsim)-(Car*Car*Car)/(9*(muPeak*muPeak)*(FzR*FzR))*pow(tan(alphaRsim),3)*(1-2*muSlide/(3*muPeak) );
    }
    else{   //Sliding on the surface
        if (alphaRsim>0){
            FFW_FyRsim = -muSlide*FzR;
        }
        else{
            FFW_FyRsim = muSlide*FzR;
        }
    }
                           
    // update states
        
    vyCGpureSim = (FFW_FyFsim+FFW_FyRsim)/m-rSim*vxCG;
    rPureSim = (a*FFW_FyFsim-b*FFW_FyRsim)/Iz;
    vyCGsimDot = (FFW_FyFsim+FFW_FyRsim)/m-rSim*vxCG + lam3*rError + lam4*vyCGerror;
    rSimDot = (a*FFW_FyFsim-b*FFW_FyRsim)/Iz + lam1*rError + lam2*vyCGerror;     
    Y[3] = vyCGpureSim;
    Y[4] = rPureSim;
            
    vyCGsim=vyCGsim+Ts*vyCGsimDot;
    rSim=rSim+Ts*rSimDot;
    // reset state values
    ssSetRWorkValue(S, 0, vyCGsim);   // set UyCG
    ssSetRWorkValue(S, 1, rSim);   // set r
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
