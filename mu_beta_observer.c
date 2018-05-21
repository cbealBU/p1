/*
 * File: mu_beta_observer.c
 *
 * Author: Yung-Hsiang Judy Hsu
 * (C)2007 Dynamic Design Laboratory -- Stanford University
 *
 * $Revision: 1$ $Date: Feb 03, 2007 $
 *
 * This file implements an s-function for use with Simulink.  The observer accepts 11 inputs of the
 * real-time measurements of steer angle, longitudinal velocity, yaw rate, roll angle,
 * lateral acceleration, left and right jacking arm lengths, left and right load cell steering torques, 
 * and left and right mechanical trails. 
 * The observer has 3 states: estimated tire-road friction, front slip angle, and estimation flag.
 * There are 21 parameters: initial trail, vehicle roll stiffness, effective front
 * and rear cornering stiffness, front tire cornering stiffness, 4 gain scale factors, an upper
 * threshold on alphahat to prevent instability, distances to front and rear from  CG, 
 * total vehicle mass, front roll center, front track width, moment of inertia, upper limit on friction estimate
 * sampling time Ts, upper limit on roll angle, lower limit on speed, and lower limit of lateral acceleration.
 * It outputs the real-time estimates of tire-road friction and front slip angle.
 *
 * Conceptually, the observer comprises three sections.  First, using the Fiala force, linear 
 * pneumatic trail and load transfer models, the observer computes estimates for front and rear 
 * axle lateral force and front axle total aligning moment. Second, it uses experimental
 * measurements to determine measurements of front total aligning moment and front axle force.
 * Lastly, the observer updates the state estimates by utilizing the bike model for the feedforward 
 * terms and the residuals between the experimental and theoretical forces and aligning torques to 
 * provide the feedback terms. It assumes that the inputs and parameters are specified in SI units, 
 * and the estimates it generates will also be in SI units.
 *
 * This file is based (very loosely) on $MATLAB/simulink/src/sfuntmpl_basic.c
 */

#define S_FUNCTION_NAME  mu_beta_observer   /* must match name of c-file */
#define S_FUNCTION_LEVEL 2                  /* usually this value works, do not modify */

#include <math.h>
#include "simstruc.h"                       /* need this for SimStruct later */

#define NUMOFPARAMS 21
#define NUMOFSTATES	3
#define NUMOFINPUTS 11
#define NUMOFOUTPUTS 3
#define g 9.81
#define MAX(x,y) (x > y ? x : y)

static int INPUTWIDTHS[]={1,1,1,1,1,1,1,1,1,1,1};
static int OUTPUTWIDTHS[]={1,1,1};
int count = 0;
/* function prototypes */
static real_T Trail(real_T tp0, real_T mu, real_T Fz, real_T alpha, real_T Ca, real_T tm);
static real_T Fiala(real_T mu, real_T Fz, real_T alpha, real_T Ca);
static real_T Sign(real_T x);

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
        ssSetInputPortDirectFeedThrough(S, i, 0);   /* IS SET TO 1 FOR DEBUGGING!! set to 0 if ith input is NOT fed thru */
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

#define MDL_INITIALIZE_CONDITIONS
#ifdef MDL_INITIALIZE_CONDITIONS
/* ============================================================================================== *
 * Set initial conditions for the states
 * ============================================================================================== */
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *x=ssGetRealDiscStates(S);           /* declare x as a pointer referencing the states */

    x[0]=1;         /* Initial guess of mu is 1 (remember c indexes starting at 0!)*/
    x[1]=0;         /* Initial alpha_f is zero */
    x[2]=0;         /* Initial estimation_flag is off, is set to one once estimation begins */
}
#endif /* MDL_INITIALIZE_CONDITIONS */

/* ============================================================================================== *
 * Set outputs as a function of the states
 * ============================================================================================== */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    /* rename outputs */
    real_T *muhat=ssGetOutputPortRealSignal(S,0);
    real_T *afhat=ssGetOutputPortRealSignal(S,1);
    real_T *estimation_flag=ssGetOutputPortRealSignal(S,2);
    /* get states. 'const' doesn't allow state modification */
    const real_T *x=ssGetRealDiscStates(S);    
    
    /* specify the value for each output */
    *muhat=x[0];
    *afhat=x[1];
    *estimation_flag=x[2];
   
}

/* ============================================================================================== *
 * UPDATE STATE EQUATIONS (i.e. this is x(k+1) = A*x(k))
 * ============================================================================================== */
#define MDL_UPDATE 
#ifdef MDL_UPDATE
/* Function: mdlUpdate ======================================================
 * Abstract:
 *    This function is called once for every major integration time step.
 *    Discrete states are typically updated here, but this function is useful
 *    for performing any tasks that should only take place once per
 *    integration step.
 */

static void mdlUpdate(SimStruct *S, int_T tid)
{
    /* name inputs */
    /*ssGetInputPortRealSignalPtrs returns a pointer referencing where the inputs are stored */
    const real_T delta=**ssGetInputPortRealSignalPtrs(S,0);
    const real_T V=**ssGetInputPortRealSignalPtrs(S,1);
    //real_T V = 10.5;
    const real_T r=**ssGetInputPortRealSignalPtrs(S,2);
    const real_T phi=**ssGetInputPortRealSignalPtrs(S,3); // SET TO BE MODIFIABLE BECAUSE PHI_0 = PI
    const real_T ay=**ssGetInputPortRealSignalPtrs(S,4);
    const real_T jal=**ssGetInputPortRealSignalPtrs(S,5);
    const real_T jar=**ssGetInputPortRealSignalPtrs(S,6);
    const real_T taulcl=**ssGetInputPortRealSignalPtrs(S,7);
    const real_T taulcr=**ssGetInputPortRealSignalPtrs(S,8);
    const real_T tml=**ssGetInputPortRealSignalPtrs(S,9);
    const real_T tmr=**ssGetInputPortRealSignalPtrs(S,10);
    /* name parameters */
    /* ssGetSFcnParam returns parameters as a complex number, mxGetPr gets the real part */
    const real_T tp0=*mxGetPr(ssGetSFcnParam(S,0));
    const real_T Kphif=*mxGetPr(ssGetSFcnParam(S,1));
    const real_T Cafeff=*mxGetPr(ssGetSFcnParam(S,2));
    const real_T Careff=*mxGetPr(ssGetSFcnParam(S,3));
    const real_T Caf=*mxGetPr(ssGetSFcnParam(S,4));
    const real_T Katau0=*mxGetPr(ssGetSFcnParam(S,5));
    const real_T Kaf0=*mxGetPr(ssGetSFcnParam(S,6));
    const real_T Kmutau0=*mxGetPr(ssGetSFcnParam(S,7));
    const real_T Kmuf0=*mxGetPr(ssGetSFcnParam(S,8));
    const real_T afhatceil=*mxGetPr(ssGetSFcnParam(S,9));
    const real_T a=*mxGetPr(ssGetSFcnParam(S,10));
    const real_T b=*mxGetPr(ssGetSFcnParam(S,11));
    const real_T m=*mxGetPr(ssGetSFcnParam(S,12));
    const real_T hf=*mxGetPr(ssGetSFcnParam(S,13));
    const real_T tw_m=*mxGetPr(ssGetSFcnParam(S,14)); 
    const real_T Iz=*mxGetPr(ssGetSFcnParam(S,15));
    const real_T muhatceil=*mxGetPr(ssGetSFcnParam(S,16)); 
    const real_T ts=*mxGetPr(ssGetSFcnParam(S,17)); 
    const real_T phiceil=*mxGetPr(ssGetSFcnParam(S,18));  
    const real_T Vfloor=*mxGetPr(ssGetSFcnParam(S,19));
    const real_T ayfloor=*mxGetPr(ssGetSFcnParam(S,20));
    
    /* assign x to point to the states */
    real_T *x=ssGetRealDiscStates(S);

    real_T *muhat, *afhat, *estimation_flag;
    real_T arhat;
    real_T Fnf, Fnr, delta_Fzf, Fz_fl, Fz_fr;
    real_T taujl, taujr, tau_al, tau_ar, tau_aLC, tauhat;
    real_T Fyfhat, Fyrhat, FyfFialal, FyfFialar, Fyfmeas;
    real_T Cafront, traill, trailr, trail; 
    real_T Dtpalpha, Dfyalpha, Dtaualpha, Dtaumu, Dtpmu, Dfymu;
    real_T inv_theta, alpha_sl, sec2afhat, tan2afhat;
    real_T Katau, Kaf, Kmuf, Kmutau;
    real_T time;

    int i;
    Cafront = Cafeff/1.07;          // lumped Caf for front axle
    time = count/ts;
    count++;
   
    muhat=x;
    afhat=&(x[1]);
    estimation_flag=&(x[2]);
    // update estimates if estimates are reasonable, V is above Vfloor, roll angle is reasonable and we have some lateral dynamics
	if ((fabs(*afhat)<afhatceil) && (fabs(*muhat)<muhatceil) && (V>Vfloor)  && (*muhat>0) && (fabs(phi)<phiceil) && (fabs(ay)>ayfloor))
	{
        // calculate normal load on front L & R tires
        Fnf = m*g*b/(a+b);                                              // front axle normal force
        Fnr = m*g - Fnf;                                                // rear axle normal force
        delta_Fzf = fabs(1/tw_m*Kphif*phi+1/tw_m*1600.0/1724.0*hf*Fnf/g*ay);   // change in normal load on each front tire
        Fz_fl = Fnf/2 - Sign(delta)*delta_Fzf;                          // roll compensated normal load on front LEFT tire
        Fz_fr = Fnf/2 + Sign(delta)*delta_Fzf;                          // roll compensated normal load on front RIGHT tire

        // find jacking torques based on jacking arm lengths & normal load
        taujl = jal*Fz_fl;
        taujr = jar*Fz_fr;
        // include jacking torque with load-cell steering torque measurements to get total aligning moment
        tau_al = -(taulcl + taujl);
        tau_ar = -(taulcr + taujr);
        // sum both sides to get front axle total aligning torque <-- does not include friction, damping or inertia terms
        tau_aLC = tau_al + tau_ar;
        
        // calculate alpharhat
        arhat = *afhat + delta - (a+b)/V*r;
        // calculate Fyfhat, Fyrhat using Fiala model based on estimated values
        Fyfhat = Fiala(*muhat, Fnf, *afhat, Cafeff);
        Fyrhat = Fiala(*muhat, Fnr, arhat, Careff);
        
        // calculate taufhat based on estimated values & physical Caf (includes lateral wt transfer)
        // LEFT front
        FyfFialal = Fiala(*muhat, Fz_fl, *afhat, Caf);
        traill = Trail(tp0, *muhat, Fz_fl, *afhat, Caf, tml);
        // RIGHT front
        FyfFialar = Fiala(*muhat, Fz_fr, *afhat, Caf);
        trailr = Trail(tp0, *muhat, Fz_fr, *afhat, Caf, tmr);
        // sum left & right contributions to make front axle total aligning moment
        tauhat =  -traill*FyfFialal-trailr*FyfFialar;
        
        // find measured side force based on bicycle model
        Fyfmeas = m*ay - Fyrhat;
        
        /* Calculate nonlinear gains */
        trail = Trail(tp0, *muhat, Fnf, *afhat, Cafront, (tml+tmr)/2.0);  // find lumped total trail length
        inv_theta = 3*(*muhat)*Fnf/Cafront;
        alpha_sl = atan(inv_theta);
        sec2afhat = 1/(cos(*afhat)*cos(*afhat));
        tan2afhat = tan(*afhat)*tan(*afhat);
        if (fabs(*afhat) < alpha_sl) {                                  // less than full slip region
            Dtpalpha = -tp0*sec2afhat*Sign(*afhat)/inv_theta;
            Dfyalpha = -Cafront*sec2afhat + 2*Sign(*afhat)*Cafront/inv_theta*tan(*afhat)*sec2afhat - Cafront/inv_theta/inv_theta*tan2afhat*sec2afhat;
            Dtpmu = tp0*Sign(*afhat)*tan(*afhat)/inv_theta/(*muhat);
            Dfymu = -Cafront*tan2afhat*Sign(*afhat)/inv_theta/(*muhat) + 2*Cafront*tan(*afhat)*tan2afhat/(3*(*muhat)*inv_theta*inv_theta);
        }
        else {                                                          // full slip region
            Dtpalpha = 0;
            Dfyalpha = 0;
            Dtpmu = 0;
            Dfymu = -Fnf*Sign(*afhat);
        }
        Dtaualpha = -Dtpalpha*Fyfhat - trail*Dfyalpha;
        Dtaumu = -Dtpmu*Fyfhat - trail*Dfymu;
        // Katau
        Katau = -Katau0*Dtaualpha/fabs(trail*Cafront*sec2afhat);
        // Kaf
        Kaf = -Kaf0*Dfyalpha/Cafront;
        // Kmuf
        Kmuf = -Kmuf0*Dfymu/Fnf;
        // Kmutau
        Kmutau = -Kmutau0*Dtaumu;
        /* Update estimates */
        muhat[0]+= Kmutau*(tauhat-tau_aLC)+Kmuf*(Fyfhat-Fyfmeas);
        // assuming deltadot ~= 0:
        afhat[0]+= ((1/(m*V)+a*a/(Iz*V))*Fyfhat+(1/(m*V)-a*b/(Iz*V))*Fyrhat-r)*ts+Kaf*(Fyfhat-Fyfmeas)+Katau*(tauhat-tau_aLC);
        // set estimation_flag to one (estimation is ON)
        estimation_flag[0] = 1;
	}
    else { // default estimate when inputs are bogus or estimates have gone unstable
        muhat[0]=1;
        afhat[0]=0;
        // set estimation_flag to zero (estimation is OFF)
        estimation_flag[0]=0;
        //printf("t = %f: unstable af estimate or speed is too slow\n");
    }
    
}        
#endif /* MDL_UPDATE */

/*================*
 * help functions *
 *================*/
/* Fiala Force model */
static real_T Fiala(real_T mu, real_T Fz, real_T alpha, real_T Ca)
{
	real_T theta;
    
    theta = Ca/(3*mu*Fz);
    if (fabs(alpha) < atan(1/theta)) {
        return -3*mu*Fz*theta*tan(alpha)*(1 - theta*fabs(tan(alpha)) + 1/3.0*theta*theta*tan(alpha)*tan(alpha));
    }
    else return (-mu*Fz*Sign(alpha));
}
/* linearly decreasing pneumatic trail model */
static real_T Trail(real_T tp0, real_T mu, real_T Fz, real_T alpha, real_T Ca, real_T tm)
{
	real_T theta;
    
    theta = Ca/(3*mu*Fz);
    if (fabs(alpha) < atan(1/theta)) {
        return (tm+tp0-Sign(alpha)*tp0*theta*tan(alpha));
    }
    else return tm;
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
