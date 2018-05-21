/*
 * File: Envelope_Controller.c
 *
 * Author: Yung-Hsiang Judy Hsu
 * (C)2008 Dynamic Design Laboratory -- Stanford University
 *
 * $Revision: 1$ $Date: September 1, 2008 $
 *
 * This file implements an s-function version of a basic envelope controller created in SimulateObserver_full_envelope_control.m for use with Simulink.  
 * 
 *
 * This controller outputs: 
 * 1. steer angle addition (rad)
 * 2. controller status (off = -1, no_saturation = 0, front_saturation = 1, rear_saturation = 2)
 * 
 */

#define S_FUNCTION_NAME  Envelope_Controller      /* must match name of c-file */
#define S_FUNCTION_LEVEL 2                      /* usually this value works, do not modify */

#include <math.h>
#include "simstruc.h"                           /* need this for SimStruct later */

/* parameter list:
 * Ifalphaparam.Caf, Ifalphaparam.Car, param.a, EnvCtrlparams.deltatc, param.m, Ts, Ifalphaparam.Caf_shoreline_sf, EnvCtrlparams.safetyfactor_f, param.b,
 * EnvCtrlparams.Vxfloor, EnvCtrlparams.Kpf, Ifalphaparam.Car_shoreline_sf, EnvCtrlparams.Kpr, EnvCtrlparams.safetyfactor_r
 *
 */

/* global constants */
#define NUMOFPARAMS 14                      
#define NUMOFSTATES	0
#define NUMOFINPUTS 8                     
#define NUMOFOUTPUTS 2
#define g 9.81
#define NO_SATURATION 0.0                         /* indicates neither axles are saturated, steering addition is zero */
#define FRONT_SATURATION 1.0                      /* indicates front axle is saturated, countersteer to keep front slip angle near peak */
#define REAR_SATURATION 2.0                       /* indicates rear axle is saturated, countersteer to keep rear slip angle near peak */
#define CONTROLLER_OFF -1.0                       /* indicates controller is not enabled and off */

static int INPUTWIDTHS[]={1,1,1,1,1,1,1,1};       
static int OUTPUTWIDTHS[]={1,1};
/* global variables */
// storage variables, initialize to zero 
static real_T deltaprev = 0;
static real_T rprev = 0;

/* function prototypes */ 
static real_T GetSlipAngleThreshold(real_T safetyfactor, real_T Ca, real_T Iforce);
static real_T Sign(real_T x);
static real_T GetControllerSteerAngleFront(real_T alpha_HW, real_T delta_HW, real_T alphahat, real_T delta_prev, real_T delta_SBW_prev, real_T alpha_sat, real_T deltatc, real_T ts, real_T Kpf);
static real_T GetControllerSteerAngleRear(real_T alpha_HW, real_T delta_HW, real_T alphahat, real_T delta_prev, real_T delta_SBW_prev, real_T alpha_sat, real_T deltatc, real_T ts, real_T Kpr);

static char msg[80];

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

#undef MDL_INITIALIZE_CONDITIONS

/* ============================================================================================== *
 * Set outputs as a function of the states
 * ============================================================================================== */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    /* rename outputs */
    real_T *steeringaddition=ssGetOutputPortRealSignal(S,0);           
    real_T *controller_flag=ssGetOutputPortRealSignal(S,1);
    
    /* get inputs */
//     InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
//     // Parse input vector into specific input variables
//     const real_T mufhat = *uPtrs[0];
//     const real_T afhat = *uPtrs[1];
//     const real_T delta_HW = *uPtrs[2];
//     const real_T r = *uPtrs[3];
//     const real_T Vx = *uPtrs[4];
//     const real_T AtShoreline = *uPtrs[5];
//     const real_T delta = *uPtrs[6];
//     const real_T ECenable = *uPtrs[7];
    
    /*ssGetInputPortRealSignalPtrs returns a pointer referencing where the inputs are stored */
    const real_T mufhat=**ssGetInputPortRealSignalPtrs(S,0);
    const real_T afhat=**ssGetInputPortRealSignalPtrs(S,1);
    const real_T delta_HW=**ssGetInputPortRealSignalPtrs(S,2);
    const real_T r=**ssGetInputPortRealSignalPtrs(S,3);
    const real_T Vx=**ssGetInputPortRealSignalPtrs(S,4);
    const int AtShoreline=**ssGetInputPortRealSignalPtrs(S,5);
    const real_T delta=**ssGetInputPortRealSignalPtrs(S,6);
    const int ECenable=**ssGetInputPortRealSignalPtrs(S,7);
    
    /* name parameters */
    /* ssGetSFcnParam returns parameters as a complex number, mxGetPr gets the real part */
    const real_T Cafasphalt=*mxGetPr(ssGetSFcnParam(S,0));
    const real_T Carasphalt=*mxGetPr(ssGetSFcnParam(S,1));
    const real_T a=*mxGetPr(ssGetSFcnParam(S,2));
    const real_T deltatc=*mxGetPr(ssGetSFcnParam(S,3));
    const real_T m=*mxGetPr(ssGetSFcnParam(S,4));
    const real_T ts=*mxGetPr(ssGetSFcnParam(S,5));
    const real_T Caf_shoreline_sf=*mxGetPr(ssGetSFcnParam(S,6));
    const real_T safetyfactor_f=*mxGetPr(ssGetSFcnParam(S,7));
    const real_T b=*mxGetPr(ssGetSFcnParam(S,8));
    const real_T Vxfloor=*mxGetPr(ssGetSFcnParam(S,9));
    const real_T Kpf=*mxGetPr(ssGetSFcnParam(S,10));
    const real_T Car_shoreline_sf=*mxGetPr(ssGetSFcnParam(S,11));
    const real_T Kpr=*mxGetPr(ssGetSFcnParam(S,12));
    const real_T safetyfactor_r=*mxGetPr(ssGetSFcnParam(S,13));
    
    real_T Caf, Car;
    real_T Fnf, Fnr;
    real_T Ifronthat, Irearhat, arhat, betahat;
    real_T alphaFront_sat, alphaRear_sat, alphaf_HW;
    real_T deltaFront, deltaRear;
    
    if ((ECenable) && (Vx > Vxfloor)) // if envelope control is enabled (through software & hardware switches), calculate steering addition
    {
        if (AtShoreline) // if at shoreline, then scale cornering stiffnesses
        {
            Caf = Caf_shoreline_sf*Cafasphalt;
            Car = Car_shoreline_sf*Carasphalt;
        } else {
            Caf = Cafasphalt;
            Car = Carasphalt;
        }
//         printf("Caf = %f\n",Caf);
//         printf("Car = %f\n",Car);
        
        /* Calculate normal load on front L & R tires */
        Fnf = m*g*b/(a+b);      // front axle normal force
        Fnr = m*g - Fnf;        // rear axle normal force
        
        /* calculate betahat (assume betahat(i-1) ~= betahat(i)), arhat and Irearhat */
        betahat = tan(afhat + deltaprev) - a*rprev/Vx;
        arhat = afhat + deltaprev - (a+b)*rprev/Vx;
        Ifronthat = 1/(mufhat*Fnf);
        Irearhat = 1/(mufhat*Fnr);
        
        /* get slip angle saturation threshold for each axle (if an axle exceeds it, we allow controller to intervene */
        alphaFront_sat = GetSlipAngleThreshold(safetyfactor_f,Caf,Ifronthat);
        alphaRear_sat = GetSlipAngleThreshold(safetyfactor_r,Car,Irearhat);
              
        /* calculate driver commanded front slip angle */
        alphaf_HW = betahat + a*r/Vx - delta_HW;
        
        /* calculate controller steer angle for each axle */
        /* FRONT AXLE saturation: */
        deltaFront = GetControllerSteerAngleFront(alphaf_HW,delta_HW,afhat,deltaprev,deltaprev,alphaFront_sat,deltatc,ts,Kpf);
        /* REAR AXLE saturation: */
        deltaRear = GetControllerSteerAngleRear(arhat,delta_HW,arhat,deltaprev,deltaprev,alphaRear_sat,deltatc,ts,Kpr);
        
        /* If rear tires are saturated we countersteer to prevent spin out */
        /* Uncomment second condition if we want to ensure actuator authority in the front (i.e. front tires are not saturated) */
        if ((fabs(arhat) > alphaRear_sat)) // && (fabs(afhat) < alphaFront_sat))
        {
            controller_flag[0] = REAR_SATURATION;
            /* calculate steer angle addition */
            steeringaddition[0] = deltaRear - delta_HW;
        /* If front tires are saturated, we countersteer to correct to reduce front slip angle.
        In the case NEITHER are saturated, we pass through driver commanded steer angle
        (which is already determined by GetControllerSteerAngle function earlier) */
        } else {
            if (fabs(alphaf_HW) < alphaFront_sat)
            {
                controller_flag[0] = NO_SATURATION;
            } else {
                controller_flag[0] = FRONT_SATURATION;
            }
            /* calculate steer angle addition */
            steeringaddition[0] = deltaFront - delta_HW;
//             printf("deltaFront = %f\n",deltaFront);
//             printf("delta_HW = %f\n",delta_HW);
//             printf("steeringaddition = %f\n",steeringaddition[0]);
        }
    } else {
        steeringaddition[0] = 0;
        controller_flag[0] = CONTROLLER_OFF;
    }
    
    /* update storage variables for next time step */
    deltaprev = delta;
    rprev = r;
  
}


/*================*
 * help functions *
 *================*/

/* Calculate slip angle threshold */
static real_T GetSlipAngleThreshold(real_T safetyfactor, real_T Ca, real_T Iforce)
{
    /* GetSlipAngleThreshold calculates the slip angle at which the tire is fulling sliding, and based on 
     * the safety factor (value between 0 and 1, depending on how confident your tire parametrization 
     * estimates Ca and If are) returns the saturation slip angle.  The envelope controller intervenes after the slip 
     * angle exceeds the saturation slip angle.
     *
     * Inputs:
     * safetyfactor - scalar between 0-1 (.)
     * Ca - cornering stiffness (N/rad)
     * Iforce - inverted peak lateral force estimate (1/N)
     *
     * Output:
     * saturation slip angle (rad) 
     */
    
     return safetyfactor*atan(3/(Ca*Iforce));
}

/* Calculate envelope controller steer angle for rear */
static real_T GetControllerSteerAngleRear(real_T alpha_HW, real_T delta_HW, real_T alphahat, real_T delta_prev, real_T delta_SBW_prev, real_T alpha_sat, real_T deltatc, real_T ts, real_T Kpr)
{
    /* GetControllerSteerAngle outputs SBW steer angle command determined by envelope controller.  If driver commanded slip
     * angle results in tire going past the peak, the steer angle is reduced as to keep the tire in a
     * safe operating region. 
     * 
     * Inputs:
     * alpha_HW - driver commanded slip angle (rad)
     * delta_HW - driver commanded steer angle (rad)
     * alphahat - previous time step's slip angle estimate (rad)
     * delta_prev - previous time step's AXLE steer angle (rad)
     * delta_SBW_prev - previous time step's steer angle outputted from the envelope controller, which may be for a single tire (rad)
     * alpha_sat - saturation slip angle calculated from GetSlipAngleThreshold.m function (rad)
     * deltatc - first-order time constant for steer angle dynamics (1/s)
     * ts - sampling time (s)
     * Kpr - proportional control gain (.)
     *
     * Output:
     * controller outputted steer-by-wire steer angle command (rad)
     */
    
    real_T alpha_des, delta_des;
    
//     printf("1. alpha_HW = %f\n",alpha_HW);
//     printf("2. alpha_sat = %f\n",alpha_sat);
//     printf("3. alphahat = %f\n",alphahat);
    
    if (fabs(alpha_HW) < alpha_sat) 
    {
        return delta_HW;
    } else {
        alpha_des = Sign(alpha_HW)*3*alpha_sat/(3 - alpha_HW*Sign(alpha_HW) + alpha_sat);
        delta_des = alphahat + delta_prev - alpha_des;
//     	return delta_SBW_prev + 1/deltatc*(delta_des - delta_SBW_prev)*ts;
        /* P-control only */
        return Kpr*(delta_des - delta_SBW_prev);
    }
}

/* Calculate envelope controller steer angle for front */
static real_T GetControllerSteerAngleFront(real_T alpha_HW, real_T delta_HW, real_T alphahat, real_T delta_prev, real_T delta_SBW_prev, real_T alpha_sat, real_T deltatc, real_T ts, real_T Kpf)
{
    /* GetControllerSteerAngle outputs SBW steer angle command determined by envelope controller.  If driver commanded slip
     * angle results in tire going past the peak, the steer angle is reduced as to keep the tire in a
     * safe operating region. 
     * 
     * Inputs:
     * alpha_HW - driver commanded slip angle (rad)
     * delta_HW - driver commanded steer angle (rad)
     * alphahat - previous time step's slip angle estimate (rad)
     * delta_prev - previous time step's AXLE steer angle (rad)
     * delta_SBW_prev - previous time step's steer angle outputted from the envelope controller, which may be for a single tire (rad)
     * alpha_sat - saturation slip angle calculated from GetSlipAngleThreshold.m function (rad)
     * deltatc - first-order time constant for steer angle dynamics (1/s)
     * ts - sampling time (s)
     * Kp - proportional control gain (.)
     *
     * Output:
     * controller outputted steer-by-wire steer angle command (rad)
     */
    
    real_T alpha_des, delta_des;
    
//     printf("1. alpha_HW = %f\n",alpha_HW);
//     printf("2. alpha_sat = %f\n",alpha_sat);
//     printf("3. alphahat = %f\n",alphahat);
    
    if (fabs(alpha_HW) < alpha_sat) 
    {
        return delta_HW;
    } else {
        alpha_des = Sign(alpha_HW)*3*alpha_sat/(3 - alpha_HW*Sign(alpha_HW) + alpha_sat);
        delta_des = delta_prev + Kpf*(alphahat - alpha_des);
        /* Hold at desired steer angle */
        return delta_des;
//         return Kpf*(delta_des - delta_SBW_prev);
    }
}

/* signum function */
static real_T Sign(real_T x)
{
    if (x > 0) {
        return 1;
    }
    else if (x < 0) {
        return -1;
    }
    else return 0;
}

#undef MDL_DERIVATIVES  /* This is a discrete-time filter. */

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
