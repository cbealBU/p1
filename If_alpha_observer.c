/*
 * File: If_alpha_observer.c
 *
 * Author: Yung-Hsiang Judy Hsu
 * (C)2008 Dynamic Design Laboratory -- Stanford University
 *
 * $Revision: 1$ $Date: May 31, 2008 $
 *
 * This file implements an s-function version of If_alpha_tp_observer.m for use with Simulink.  This nonlinear observer outputs 
 * estimates in real-time of the front tire slip angle and the front peak achievable lateral force.  Conceptually, the observer uses an estimate of
 * pneumatic trail from total aligning moment measurements to algebraically solve for peak lateral force.  This peak lateral force estimate
 * is used to update our front slip angle.  Of course, given front slip angle, we can kinematically solve for rear slip angle.
 *
 * The observer has 3 states: 
 * 1. estimated front slip angle
 * 2. estimated front peak lateral force (mu*Fz)
 * 3. binary estimation flag
 * 
 * It assumes that the inputs and parameters are specified in SI units, 
 * and the estimates it generates will also be in SI units.
 *
 * This file is based (very loosely) on $MATLAB/simulink/src/sfuntmpl_basic.c
 */

#define S_FUNCTION_NAME  If_alpha_observer      /* must match name of c-file */
#define S_FUNCTION_LEVEL 2                      /* usually this value works, do not modify */

#include <math.h>
#include "simstruc.h"                           /* need this for SimStruct later */

/* List of parameters:
 * Ifalphaparam.tp_window, Ifalphaparam.FIRwindow, Ifalphaparam.IFwindow, param.Fz_f, Ifalphaparam.mu_peak_nom, param.fl.fw, param.fl.Jw, param.fl.bw,
param.fr.fw, param.fr.Jw, param.fr.bw, param.a, param.b, param.m, Ifalphaparam.hf, param.tw, param.Iz, Ifalphaparam.Kaf0, Ts, Ifalphaparam.phiceil,
Ifalphaparam.Vfloor, Ifalphaparam.ayfloor, Ifalphaparam.tp0, Ifalphaparam.Caf, Ifalphaparam.Car, Ifalphaparam.Caf_shoreline_sf, Ifalphaparam.mu_slip_nom, 
Ifalphaparam.alpha_thres, Ifalphaparam.tpthres_sf, Ifalphaparam.Kphif, Ifalphaparam.afhatceil, Ifalphaparam.Kaf_sf_Shoreline, Ifalphaparam.Kaf_sf_asphalt,
Ifalphaparam.Car_shoreline_sf, Ifalphaparam.alphafoffset, Ifalphaparam.alpharoffset
 *
 */

/* global constants */
#define NUMOFPARAMS 36                      
#define NUMOFSTATES	3
#define NUMOFINPUTS 13                      
#define NUMOFOUTPUTS 3
#define g 9.81
#define ON 1
#define OFF 0
#define MAX(x,y) (x > y ? x : y)
#define TPWINDOW_INT   5         /* ssGetSFcnParam(S,0)  length of pneumatic trail window filter (in sample times, based on ts = 0.002s) */
#define FIRWINDOW_INT  50          /* ssGetSFcnParam(S,1) length of window filter used for total aligning moment, ay, etc. 
                                                 (in sample times, based on ts = 0.002s & tire hop mode) */
#define IFWINDOW_INT   250         /*  ssGetSFcnParam(S,2) length of window filter used for peak force estimate only (longer than FIRwindow due to noise in estimates) */
#define TPWINDOW       5.0         /* ssGetSFcnParam(S,0)  length of pneumatic trail window filter (in sample times, based on ts = 0.002s) */
#define FIRWINDOW      50.0          /* ssGetSFcnParam(S,1) length of window filter used for total aligning moment, ay, etc. 
                                                 (in sample times, based on ts = 0.002s & tire hop mode) */
#define IFWINDOW       250.0         /*  ssGetSFcnParam(S,2) length of window filter used for peak force estimate only (longer than FIRwindow due to noise in estimates) */

static int INPUTWIDTHS[]={1,1,1,1,1,1,1,1,1,1,1,1};       
static int OUTPUTWIDTHS[]={1,1,1};
/* global variables */
static int count=0;
static real_T FIRay_buffer[FIRWINDOW_INT];          /* lateral acceleration buffer, FIR filtered */
static real_T FIRtau_al_buffer[FIRWINDOW_INT];      /* left total aligning moment buffer, FIR filtered */
static real_T FIRtau_ar_buffer[FIRWINDOW_INT];      /* right total aligning moment buffer, FIR filtered */
static real_T FIRdeltadot_buffer[FIRWINDOW_INT];    /* steer angle derivative buffer, FIR filtered */
static real_T afhat_buffer[FIRWINDOW_INT];          /* estimated front slip angle buffer, FIR filtered */
static real_T tpL_buffer[FIRWINDOW_INT];            /* estimated front left pneumatic trail buffer, FIR filtered */
static real_T tpR_buffer[FIRWINDOW_INT];            /* estimated front right pneumatic trail buffer, FIR filtered */
static real_T If_frontbuffer[IFWINDOW_INT];        /* estimated front peak lateral force buffer, FIR filtered */
/* initialize buffered previous values to zero */
static real_T tpLprev;
static real_T tpRprev;
static real_T MzLprev;
static real_T MzRprev;
static real_T Fyflefthatprev;
static real_T Fyfrighthatprev;
static real_T If_leftprev;
static real_T If_rightprev;
static real_T deltaprev;
static real_T deltaLprev;
static real_T deltaRprev;
static real_T dangleLprev;
static real_T dangleRprev;
static real_T tm_lprev;
static real_T tm_rprev;
// static real_T afhat_noFIRprev;

/* function prototypes */
static real_T Fiala(real_T mu_ratio, real_T If_val, real_T alpha, real_T Ca);
static real_T Sign(real_T x);
static real_T CreateFIRBuffer(real_T window_length, real_T xcurrent, real_T xnew, real_T x_buffer_shifted, int for_loop_index);
// static real_T FIRfilter(real_T signal_buffer[], real_T window_length);
static real_T GetPneumaticTrail(real_T tp_window, real_T tp0, real_T tpprev[], real_T Fyhatprev, real_T Mzprev, real_T tmprev, int tp_window_int);
static real_T PeakForceEstimator(real_T tp, real_T tpthres, real_T alphahat, real_T alphathres, real_T If_nom, real_T Ca, real_T Iforce_prev, real_T tp0, real_T tm, real_T Mz, real_T mu_ratio);
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
    const real_T Fz_f=*mxGetPr(ssGetSFcnParam(S,3));
    const real_T mu_peak_nom=*mxGetPr(ssGetSFcnParam(S,4));
    int i;
    
    /* initialize all buffer arrays */
    for (i = 0; i<FIRWINDOW; i++) {
        FIRay_buffer[i] = 0;
    }
    for (i = 0; i<FIRWINDOW; i++) {
        FIRtau_al_buffer[i] = 0;
    }
    for (i = 0; i<FIRWINDOW; i++) {
        FIRtau_ar_buffer[i] = 0;
    }
    for (i = 0; i<FIRWINDOW; i++) {
        FIRdeltadot_buffer[i] = 0;
    }
    for (i = 0; i<FIRWINDOW; i++) {
        afhat_buffer[i] = 0;
    }
    for (i = 0; i<FIRWINDOW; i++) {
        tpL_buffer[i] = 0;
    }
    for (i = 0; i<FIRWINDOW; i++) {
        tpR_buffer[i] = 0;
    }
    for (i = 0; i<IFWINDOW; i++) {
        If_frontbuffer[i] = 0;
    }
    /* initialize all storage variables to zero */
    tpLprev = 0;
    tpRprev = 0;
    MzLprev = 0;
    MzRprev = 0;
    Fyflefthatprev = 0;
    Fyfrighthatprev = 0;
    If_leftprev = 0;
    If_rightprev = 0;
    deltaprev = 0;
    deltaLprev = 0;
    deltaRprev = 0;
    dangleLprev = 0;
    dangleRprev = 0;
    tm_lprev = 0;
    tm_rprev = 0;
//     afhat_noFIRprev = 0;
    
    x[0]=0;                             /* Initial alpha_f is zero (remember c indexes starting at 0!) */
    x[1]=mu_peak_nom*Fz_f*2.0;          /* Initial guess of front peak lateral force */
    x[2]=0;                             /* Initial estimation_flag is zero, is set to one once estimation begins */
}
#endif /* MDL_INITIALIZE_CONDITIONS */

/* ============================================================================================== *
 * Set outputs as a function of the states
 * ============================================================================================== */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    /* rename outputs */
    real_T *afhat=ssGetOutputPortRealSignal(S,0);           // FIR filtered afhat
    real_T *peakFyf=ssGetOutputPortRealSignal(S,1);
    real_T *estimation_flag=ssGetOutputPortRealSignal(S,2);
    
    /* get offset parameter */
    const real_T alphafoffset=*mxGetPr(ssGetSFcnParam(S,34));

    /* get states. 'const' doesn't allow state modification */
    const real_T *x=ssGetRealDiscStates(S);    
    
    /* specify the value for the outputs */
    // Account for offset of tire curve due to unevenly worn tires
    *afhat=x[0] + alphafoffset;
    *peakFyf=x[1];
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
    const real_T r=**ssGetInputPortRealSignalPtrs(S,2);
    real_T phi=**ssGetInputPortRealSignalPtrs(S,3); // SET TO BE MODIFIABLE BECAUSE PHI_0 = PI
    const real_T ay=**ssGetInputPortRealSignalPtrs(S,4);
    const real_T jal=**ssGetInputPortRealSignalPtrs(S,5);
    const real_T jar=**ssGetInputPortRealSignalPtrs(S,6);
    const real_T taulcl=**ssGetInputPortRealSignalPtrs(S,7);
    const real_T taulcr=**ssGetInputPortRealSignalPtrs(S,8);
    const real_T tml=**ssGetInputPortRealSignalPtrs(S,9);
    const real_T tmr=**ssGetInputPortRealSignalPtrs(S,10);
    const int AtShoreline=**ssGetInputPortRealSignalPtrs(S,11);
    const int EnableObserver=**ssGetInputPortRealSignalPtrs(S,12);
    /* temporarily assume L/R steer angles are equal to average steer angle */
//     const real_T deltaL=**ssGetInputPortRealSignalPtrs(S,11);
//     const real_T deltaR=**ssGetInputPortRealSignalPtrs(S,12);

    /* name parameters */
    /* ssGetSFcnParam returns parameters as a complex number, mxGetPr gets the real part */
    const real_T Fz_f=*mxGetPr(ssGetSFcnParam(S,3));
    const real_T mu_peak_nom=*mxGetPr(ssGetSFcnParam(S,4));
    const real_T fwfl=*mxGetPr(ssGetSFcnParam(S,5));
    const real_T Jwfl=*mxGetPr(ssGetSFcnParam(S,6));
    const real_T bwfl=*mxGetPr(ssGetSFcnParam(S,7));
    const real_T fwfr=*mxGetPr(ssGetSFcnParam(S,8));
    const real_T Jwfr=*mxGetPr(ssGetSFcnParam(S,9));
    const real_T bwfr=*mxGetPr(ssGetSFcnParam(S,10));
    const real_T a=*mxGetPr(ssGetSFcnParam(S,11));
    const real_T b=*mxGetPr(ssGetSFcnParam(S,12));
    const real_T m=*mxGetPr(ssGetSFcnParam(S,13));
    const real_T hf=*mxGetPr(ssGetSFcnParam(S,14));
    const real_T tw_m=*mxGetPr(ssGetSFcnParam(S,15)); 
    const real_T Iz=*mxGetPr(ssGetSFcnParam(S,16));
    const real_T Kaf0=*mxGetPr(ssGetSFcnParam(S,17)); 
    const real_T ts=*mxGetPr(ssGetSFcnParam(S,18)); 
    const real_T phiceil=*mxGetPr(ssGetSFcnParam(S,19));  
    const real_T Vfloor=*mxGetPr(ssGetSFcnParam(S,20));
    const real_T ayfloor=*mxGetPr(ssGetSFcnParam(S,21));
    const real_T tp0=*mxGetPr(ssGetSFcnParam(S,22));
    const real_T Cafasphalt=*mxGetPr(ssGetSFcnParam(S,23));
    const real_T Carasphalt=*mxGetPr(ssGetSFcnParam(S,24));
    const real_T Caf_shoreline_sf=*mxGetPr(ssGetSFcnParam(S,25));
    const real_T mu_slip_nom=*mxGetPr(ssGetSFcnParam(S,26));
    const real_T alpha_thres=*mxGetPr(ssGetSFcnParam(S,27));
    const real_T tpthres_sf=*mxGetPr(ssGetSFcnParam(S,28));
    const real_T Kphif=*mxGetPr(ssGetSFcnParam(S,29));
    const real_T afhatceil=*mxGetPr(ssGetSFcnParam(S,30)); 
    const real_T Kaf_sf_Shoreline=*mxGetPr(ssGetSFcnParam(S,31)); 
    const real_T Kaf_sf_asphalt=*mxGetPr(ssGetSFcnParam(S,32));
    const real_T Car_shoreline_sf=*mxGetPr(ssGetSFcnParam(S,33));
    const real_T alphafoffset=*mxGetPr(ssGetSFcnParam(S,34));
    const real_T alpharoffset=*mxGetPr(ssGetSFcnParam(S,35));
    
    int i;

    /* assign x to point to the states */
    real_T *x=ssGetRealDiscStates(S);
    real_T Caf, Car;
    real_T *peakFyf, *afhat, *estimation_flag;          // assign as pointers
    real_T arhat, FIRafhat, afhat_noFIR;
    real_T Fnf, Fnr, delta_Fzf, Fz_fl, Fz_fr;
    real_T taujl, taujr, tau_al, tau_ar, MzL, MzR;
    real_T Fyfhat, Fyrhat, Fyflefthat, Fyfrighthat, Fyfmeas, FIRIf_front;
    real_T tpL, tpR, tpthres;
    real_T If_left, If_right, If_front, If_rear, mu_p_front, If_nom;
    real_T FIRtau_al, FIRtau_ar, FIRay, FIRdeltadot; 
    real_T deltadot, dangleL, dangleR, ddangleL, ddangleR;
    real_T omega_dirL, omega_dirR;
    real_T Kaf, mu_ratio;
    real_T time;  
    real_T deltaL, deltaR; /* REMOVE this if deltaL, deltaR are inputs! */
    real_T Kaf_sf;
    
    if (AtShoreline) // if at shoreline, then scale Caf, Car
    {
        Caf = Caf_shoreline_sf*Cafasphalt;
        Car = Car_shoreline_sf*Carasphalt;
        Kaf_sf = Kaf_sf_Shoreline;
    } else {
        Caf = Cafasphalt;
        Car = Carasphalt;
        Kaf_sf = Kaf_sf_asphalt;
    }

    /* REMOVE this if deltaL, deltaR are inputs! */
    deltaL = deltaR = delta;

    time = count*ts;
    count++;
    // printf("count = %i\n",count);
    mu_ratio = mu_slip_nom/mu_peak_nom;                 // ratio of slip to peak friction (.)
    If_nom = 1/(mu_peak_nom*Fz_f);                      // nominal peak FRONT TIRE force
    tpthres = tpthres_sf*tp0;
    
    // assign our pointers to point to our states
    afhat=x;
    peakFyf=&(x[1]);
    estimation_flag=&(x[2]);
    // update estimates if estimates are reasonable, V is above Vfloor, roll angle is reasonable and we have some lateral dynamics
    if ((fabs(*afhat)<afhatceil) && (V>Vfloor) && (fabs(phi)<phiceil) && EnableObserver) // && (fabs(ay)>ayfloor))
	{
        // Differentiate steer angle 
        if ((count > 1) && (*estimation_flag == ON)) {
            deltadot = (delta-deltaprev)/ts;
            dangleL = (deltaL-deltaLprev)/ts;
            dangleR = (deltaR-deltaRprev)/ts;
            ddangleL = (dangleL-dangleLprev)/ts;
            ddangleR = (dangleR-dangleRprev)/ts;
        }
        else if (*estimation_flag == OFF) {// we weren't estimating in previous time step
            // set derivatives to zero
            deltadot = dangleL = dangleR = ddangleL = ddangleR = 0;
        }
        // Store delta's for next time step
        deltaprev = delta;
        deltaLprev = deltaL;
        deltaRprev = deltaR;
        dangleLprev = dangleL;
        dangleRprev = dangleR;
        
        // Window filter ay
        FIRay = 0;
        for (i = 0; i<FIRWINDOW; i++) {
            FIRay_buffer[i] = CreateFIRBuffer(FIRWINDOW, FIRay_buffer[i], ay, FIRay_buffer[i+1], i);
            FIRay += FIRay_buffer[i]/FIRWINDOW;
        }
                    
        // Calculate normal load on front L & R tires
        Fnf = m*g*b/(a+b);                                                      // front axle normal force
        Fnr = m*g - Fnf;                                                        // rear axle normal force
        delta_Fzf = 1/tw_m*Kphif*phi+1/tw_m*1600.0/1724.0*hf*Fnf/g*FIRay;       // change in normal load on each front tire
        Fz_fl = Fnf/2 - delta_Fzf;                                              // roll compensated normal load on front LEFT tire
        Fz_fr = Fnf/2 + delta_Fzf;                                              // roll compensated normal load on front RIGHT tire
        // Find jacking torques based on jacking arm lengths & modeled normal load
        taujl = jal*Fz_fl;
        taujr = jar*Fz_fr;
        // Include jacking torque with load-cell steering torque measurements and steering system model to get total aligning moment
        omega_dirL = Sign(dangleL);
        omega_dirR = Sign(dangleR); 
        tau_al = -(taulcl - omega_dirL*fwfl + taujl - Jwfl*ddangleL - bwfl*dangleL);
        tau_ar = -(taulcr - omega_dirR*fwfr + taujr - Jwfr*ddangleR - bwfr*dangleR);
        
        // Calculate estimated rear slip angle
        arhat = atan(tan(*afhat) + delta - (a+b)/V*r);
         
        // Create buffer of last FIRwindow samples of: taulc, deltadot_sub, afhat, arhat
        FIRtau_al = FIRtau_ar = FIRdeltadot = 0;
        for (i = 0; i<FIRWINDOW; i++) {
            FIRtau_al_buffer[i] = CreateFIRBuffer(FIRWINDOW, FIRtau_al_buffer[i], tau_al, FIRtau_al_buffer[i+1], i);
            FIRtau_al += FIRtau_al_buffer[i]/FIRWINDOW;
        }
        for (i = 0; i<FIRWINDOW; i++) {
            FIRtau_ar_buffer[i] = CreateFIRBuffer(FIRWINDOW, FIRtau_ar_buffer[i], tau_ar, FIRtau_ar_buffer[i+1], i);
            FIRtau_ar += FIRtau_ar_buffer[i]/FIRWINDOW;
        }
        for (i = 0; i<FIRWINDOW; i++) {
            FIRdeltadot_buffer[i] = CreateFIRBuffer(FIRWINDOW, FIRdeltadot_buffer[i], deltadot, FIRdeltadot_buffer[i+1], i);
            FIRdeltadot += FIRdeltadot_buffer[i]/FIRWINDOW;
        }
        
        // Use tire hop windowed Mz
        MzL = FIRtau_al;
        MzR = FIRtau_ar;
        
        // Use estimated tp to solve for If for each front tire
        /* Front LEFT */
        tpL = GetPneumaticTrail(TPWINDOW,tp0,tpL_buffer,Fyflefthatprev,MzLprev,tm_lprev,TPWINDOW_INT);

        // Create buffer of tpL
        if (count > TPWINDOW + 1) {
            for (i = 0; i<TPWINDOW-1; i++) {
                tpL_buffer[i] = CreateFIRBuffer(TPWINDOW-1, tpL_buffer[i], tpL, tpL_buffer[i+1], i);
            }
        }
        // Check if pneumatic trail is > tp0 or slip angle is below slip angle estimation threshold
        if ((tpL >= tpthres) || (fabs(*afhat) < alpha_thres)) {
            If_left = If_nom;
            Fyflefthat = -Caf/2*(*afhat);
        }
        else {
            If_left = PeakForceEstimator(tpL,tpthres,*afhat,alpha_thres,If_nom,Caf/2,If_leftprev,tp0,tml,MzL,mu_ratio);
            // Solve for current lateral force estimate (if no lateral excitation, then use linear tire model)
            Fyflefthat = Fiala(mu_ratio, If_left, *afhat, Caf/2);
        }
        // Update values for next time step
        MzLprev = MzL;
        Fyflefthatprev = Fyflefthat;
        tm_lprev = tml;
        tpLprev = tpL;
        If_leftprev = If_left;
        
        /* Front RIGHT */
        tpR = GetPneumaticTrail(TPWINDOW,tp0,tpR_buffer,Fyfrighthatprev,MzRprev,tm_rprev,TPWINDOW_INT);
        // Create buffer of tpR
        if (count > TPWINDOW + 1) {
            for (i = 0; i<TPWINDOW-1; i++) {
                tpR_buffer[i] = CreateFIRBuffer(TPWINDOW-1, tpR_buffer[i], tpR, tpR_buffer[i+1], i);
            }
        }
        // Check if pneumatic trail is > tp0 or slip angle is below slip angle estimation threshold
        if ((tpR >= tpthres) || (fabs(*afhat) < alpha_thres)) {
            If_right = If_nom;
            Fyfrighthat = -Caf/2*(*afhat);
        }
        else {
            If_right = PeakForceEstimator(tpR,tpthres,*afhat,alpha_thres,If_nom,Caf/2,If_rightprev,tp0,tmr,MzR,mu_ratio);
            // Solve for current lateral force estimate (if no lateral excitation, then use linear tire model)
            Fyfrighthat = Fiala(mu_ratio, If_right, *afhat, Caf/2);
        }
        // Update values for next time step
        MzRprev = MzR;
        Fyfrighthatprev = Fyfrighthat;
        tm_rprev = tmr;
        tpRprev = tpR;
        If_rightprev = If_right;

        // Find front axle If value
        If_front = 1/(1/If_left+1/If_right);
        mu_p_front = 1/(If_front*Fnf);
        // Assuming friction is the same for front/rear, find If value for rear axle
        If_rear = 1/(mu_p_front*Fnr);
        
        // Calculate estimated front/rear tire forces
        Fyrhat = Fiala(mu_ratio, If_rear, arhat, Car);
        Fyfhat = Fyflefthat + Fyfrighthat;
        
        // Find side force measurement based on bicycle model
        Fyfmeas = m*FIRay - Fyrhat;
        
        /* ---------------------------- Update Estimates------------------------------- */
        // Observer gain
        Kaf = Kaf_sf*Kaf0;     // hand-tuned gain
        
        // Update front slip angle estimate
        afhat[0] += ((1/(m*V)+a*a/(Iz*V))*Fyfhat+(1/(m*V)-a*b/(Iz*V))*Fyrhat-r-FIRdeltadot)*ts+Kaf*(Fyfhat-Fyfmeas);
        
        // FIR filter afhat (not outputted, but can be in the future)
        FIRafhat = 0;
        for (i = 0; i<FIRWINDOW; i++) {
            afhat_buffer[i] = CreateFIRBuffer(FIRWINDOW, afhat_buffer[i], *afhat, afhat_buffer[i+1], i);
            FIRafhat += afhat_buffer[i]/FIRWINDOW;
        }

        // Update peak force estimates using FIR filtered value
        FIRIf_front = 0;
        for (i = 0; i<IFWINDOW; i++) {
            If_frontbuffer[i] = CreateFIRBuffer(IFWINDOW, If_frontbuffer[i], If_front, If_frontbuffer[i+1], i);
            FIRIf_front += If_frontbuffer[i]/IFWINDOW;
        }
        if ((FIRIf_front <= 1/(mu_peak_nom*Fz_f*2.0)) && (count > 1)) {
            peakFyf[0] = 1/(If_nom/2.0);
        }
        else {
            peakFyf[0] = 1/FIRIf_front;
        }
        // Set estimation_flag to 1
        estimation_flag[0] = ON;
	}
    else { 
        // Still store values in buffers
        if (count > 1)  {
            deltadot = (delta-deltaprev)/ts;
            dangleL = (deltaL-deltaLprev)/ts;
            dangleR = (deltaR-deltaRprev)/ts;
            ddangleL = (dangleL-dangleLprev)/ts;
            ddangleR = (dangleR-dangleRprev)/ts;
        }
        FIRay = 0;
        for (i = 0; i<FIRWINDOW; i++) {
            FIRay_buffer[i] = CreateFIRBuffer(FIRWINDOW, FIRay_buffer[i], ay, FIRay_buffer[i+1], i);
            FIRay += FIRay_buffer[i]/FIRWINDOW;
        }
        // Calculate normal load on front L & R tires if phi is reasonable
        if (fabs(phi)<phiceil) {
            Fnf = m*g*b/(a+b);                                                      // front axle normal force
            Fnr = m*g - Fnf;                                                        // rear axle normal force
            delta_Fzf = 1/tw_m*Kphif*phi+1/tw_m*1600.0/1724.0*hf*Fnf/g*FIRay;       // change in normal load on each front tire
        } else {
            delta_Fzf = 0;
        }
        Fz_fl = Fnf/2 - delta_Fzf;                                              // roll compensated normal load on front LEFT tire
        Fz_fr = Fnf/2 + delta_Fzf;                                              // roll compensated normal load on front RIGHT tire
        // Find jacking torques based on jacking arm lengths & modeled normal load
        taujl = jal*Fz_fl;
        taujr = jar*Fz_fr;
        // Include jacking torque with load-cell steering torque measurements and steering system model to get total aligning moment
        omega_dirL = Sign(dangleL);
        omega_dirR = Sign(dangleR);
        tau_al = -(taulcl - omega_dirL*fwfl + taujl - Jwfl*ddangleL - bwfl*dangleL);
        tau_ar = -(taulcr - omega_dirR*fwfr + taujr - Jwfr*ddangleR - bwfr*dangleR);
        for (i = 0; i<FIRWINDOW; i++) {
            FIRtau_al_buffer[i] = CreateFIRBuffer(FIRWINDOW, FIRtau_al_buffer[i], tau_al, FIRtau_al_buffer[i+1], i);
        }
        for (i = 0; i<FIRWINDOW; i++) {
            FIRtau_ar_buffer[i] = CreateFIRBuffer(FIRWINDOW, FIRtau_ar_buffer[i], tau_ar, FIRtau_ar_buffer[i+1], i);
        }
        for (i = 0; i<FIRWINDOW; i++) {
            FIRdeltadot_buffer[i] = CreateFIRBuffer(FIRWINDOW, FIRdeltadot_buffer[i], deltadot, FIRdeltadot_buffer[i+1], i);
        }
        // Store delta's for next time step
        deltaprev = delta;
        deltaLprev = deltaL;
        deltaRprev = deltaR;
        dangleLprev = dangleL;
        dangleRprev = dangleR;
        
        // default estimates when inputs are bogus or estimates have gone unstable
        afhat[0] = 0;
        peakFyf[0] = mu_peak_nom*Fz_f*2.0;
        // set estimation_flag to zero (estimation is OFF)
        estimation_flag[0] = OFF;
        // printf("count = %i, not estimating\n",count);
    }
}
#endif /* MDL_UPDATE */

/*================*
 * help functions *
 *================*/

/* Fiala Force model */
static real_T Fiala(real_T mu_ratio, real_T If_val, real_T alpha, real_T Ca)
{
    if (If_val > 0) {
        if (fabs(alpha) < atan(3/(Ca*If_val))) {
            return -Ca*tan(alpha) + Ca*Ca*If_val/3.0*(2-mu_ratio)*fabs(tan(alpha))*tan(alpha) - Ca*Ca*Ca*If_val*If_val/9.0*(1-2.0/3.0*mu_ratio)*tan(alpha)*tan(alpha)*tan(alpha);
        }   
        else return (-mu_ratio/If_val*Sign(alpha));
    }
    else return 0;
}

/* Create buffer of previous signals */
static real_T CreateFIRBuffer(real_T window_length, real_T xcurrent, real_T xnew, real_T x_buffer_shifted, int for_loop_index)
{
    // build buffer at the beginning with previous values, new value, and zeros
    if (count < window_length) {      
        if (for_loop_index == count) {
            // Enter update
            return xnew;
        } 
        else if (for_loop_index < count) {
            // Keep original value
            return xcurrent;
        }
        else {
            // Append with zeros
            return 0;
        }
    }
    // Otherwise, move signal up one and add new signal
    else {
        if (for_loop_index < window_length-1) {
            // Fill the buffer with previous value, shifted one up
            return x_buffer_shifted;
        } 
        else {
            // Enter update
            return xnew;
        }
    }
}

/* Get pneumatic trail estimate from total aligning moment, mechanical trail and estimated lateral force */
static real_T GetPneumaticTrail(real_T tp_window, real_T tp0, real_T tpprev[], real_T Fyhatprev, real_T Mzprev, real_T tmprev, int tp_window_int)
{
    /* GetPneumaticTrail returns the window-averaged pneumatic trail calculated from total aligning moment,
    lateral force estimate and mechanical trail.  If the index of the simulation is too early to have enough
    previous tp's to average over (or if lateral force estimate is zero), the function defaults to nominal tp0. */
    int i;
    real_T tpnew, tp;
    
    if ((count>tp_window+1) && (fabs(Fyhatprev)>0)) {
        // Calculate new tp value
        if (fabs(Fyhatprev) > 0) {
            // If Fyhatprev is nonzero, then calculate new tp value
            tpnew = -(Mzprev/Fyhatprev+tmprev);
        }
        // otherwise, use last tp value as current value
        else tpnew = tpprev[tp_window_int-1]; 
        
        // Average over a window of length tp_window
        tp = 0;
        for (i=0; i<tp_window-1; i++) {
            tp += tpprev[i]/tp_window;
        }
        tp += tpnew/tp_window;
    }
    else tp = tp0;
    
    return tp;    
}   

/* Peak lateral force estimator */
static real_T PeakForceEstimator(real_T tp, real_T tpthres, real_T alphahat, real_T alphathres, real_T If_nom, real_T Ca, real_T Iforce_prev, real_T tp0, real_T tm, real_T Mz, real_T mu_ratio) 
{
/*  PeakForceEstimator returns inverted peak lateral force estimate.  The
    estimator is purely algebraic, using a simple linearly modeled relationship between inverted peak force and
    pneumatic trail.  If tires are saturated (alphahat > alpha_sl), then peak force can be determined
    purely from total aligning moment and mechanical trail.
    Inputs:
     tp - pneumatic trail estimate outputted from GetPneumaticTrail.m (m)
     tpthres - pneumatic trail threshold (m)
     alphahat - estimated slip angle (rad)
     alphathres - slip angle threshold for when Iforce estimation begins (rad)
     If_nom - nominal inverted peak force (1/N)
     Ca - cornering stiffness (N/rad)
     Iforce_prev - previous time step's inverted peak force (1/N)
     tp0 - nominal pneumatic trail (m)
     tm - mechanical trail (m)
     Mz - total aligning moment (Nm)
     mu_ratio - ratio of mu_slide/mu_peak (.)
    Outputs:
     Iforce - inverted peak lateral force estimate (1/N) */

    if ((tp >= tpthres) || (fabs(alphahat) < alphathres)) {
        return If_nom;
    }
    else {
        // Before full slide, algebraically solve for inverted force
        if (fabs(alphahat) < atan(3.0/(Ca*Iforce_prev))) {
            return (-3.0*(tp - tp0)/(tp0*Ca*fabs(tan(alphahat))));
        }
        // Since tp = 0 at full slide, use Mz info to solve for If
        else {
            return (Sign(alphahat)*tm*mu_ratio/Mz);
        }
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
