/*
 * File: friction_observer_beal.c
 *
 * Author: Craig Beal
 * (C)2011 Dynamic Design Laboratory -- Stanford University
 *
 * Date: June 9, 2011
 *
 * This file implements a nonlinear observer which estimates in real-time
 * the front tire slip angle and the front peak achievable lateral force.  
 * Conceptually, the observer uses an estimate of pneumatic trail from 
 * steering aligning moment measurements to algebraically solve for peak
 * lateral force.  This peak lateral force estimate is used to update the
 * front slip angle estimate.  Of course, given front slip angle, it is
 * possible to kinematically solve for rear slip angle.
 *
 * The observer has 3 states: 
 * 1. estimated front slip angle
 * 2. estimated front peak lateral force (mu*Fz)
 * 3. binary estimation flag
 * 
 * It assumes that the inputs and parameters are specified in SI units, 
 * and the estimates it generates will also be in SI units.
 *
 * This file is based (loosely) on $MATLAB/simulink/src/sfuntmpl_basic.c
 */

#define S_FUNCTION_NAME  friction_observer_beal  /* must match filename */
#define S_FUNCTION_LEVEL 2

#include <math.h>
#include "simstruc.h"                  /* need this for SimStruct later */

/* List of parameters with indices
 * (0)  Ts                              (1)  param.a
 * (2)  param.b                         (3)  param.m
 * (4)  param.Iz                        (5)  Ifalphaparam.tp0L
 * (6)  Ifalphaparam.tp0R               (7)  Ifalphaparam.Caf
 * (8)  Ifalphaparam.Car                (9)  Ifalphaparam.Caf_shoreline_sf
 * (10) Ifalphaparam.Car_shoreline_sf   (11) Ifalphaparam.alphafoffset
 * (12) Ifalphaparam.alpharoffset       (13) Ifalphaparam.mu_peak_nom
 * (14) Ifalphaparam.Kaf0               (15) Ifalphaparam.Kaf_sf_Shoreline
 * (16) Ifalphaparam.Kaf_sf_asphalt     (17) Ifalphaparam.Vfloor
 * (18) Ifalphaparam.ayfloor            (19) Ifalphaparam.alpha_thres
 * (20) Ifalphaparam.afhatceil          (21) Ifalphaparam.kappaceil */

/* a useful function */
#define MAX(x,y) (x > y ? x : y)

/* global constants */
#define NUMOFPARAMS 22
#define NUMOFSTATES 5
#define NUMOFINPUTS 11
#define NUMOFOUTPUTS 5
#define g 9.81

/* estimation flags */
#define OPTIMAL 1
#define RIGHT_ONLY 0.8
#define LEFT_ONLY 0.7
#define LINEAR_REG 0.5
#define DEGRADED 0.25
#define SANITY_FAIL 0.1
#define OFF 0

/* peak force flag */
#define IF_NORMAL 0
#define IF_SLIDE 1

/* pneumatic trail flags */
#define TP_SANITY_FAIL 3
#define TP_RANGE_HIGH 2
#define TP_RANGE_LOW 1
#define NO_ERROR 0

/* filter lengths */
#define TPWINDOW_INT   5
#define IFWINDOW_INT   300
#define MUWINDOW_INT   25

static int INPUTWIDTHS[]={1,1,1,1,1,4,2,2,2,1,1};       
static int OUTPUTWIDTHS[]={2,1,5,2,2};

/* global FIR filtering variables */
static real_T FIRafhat_buffer[TPWINDOW_INT];        /* estimated front slip angle buffer, FIR filtered */
static real_T FIRafhat = 0;                         /* FIR filtered estimated value of front slip angle */
static int FIRafhat_index = 0;                      /* front slip angle estimate filter index */
static real_T FIRtpL_buffer[TPWINDOW_INT];          /* estimated front left pneumatic trail buffer, FIR filtered */
static real_T FIRtpL = 0;                           /* FIR filtered left pneumatic trail value */
static int FIRtpL_index = 0;                        /* left pneumatic trail filter index */
static real_T FIRtpR_buffer[TPWINDOW_INT];          /* estimated front right pneumatic trail buffer, FIR filtered */
static real_T FIRtpR = 0;                           /* FIR filtered right pneumatic trail value */
static int FIRtpR_index = 0;                        /* right pneumatic trail filter index */
static real_T FIRIf_frontbuffer[IFWINDOW_INT];      /* estimated front peak lateral force buffer, FIR filtered */
static real_T FIRIf_front = 0;                      /* FIR filtered front peak lateral force value */
static int FIRIf_front_index = 0;                   /* front peak lateral force filter index */
static real_T FIRmu_buffer[MUWINDOW_INT];           /* output friction coefficient buffer, FIR filtered */
static real_T FIRmu = 0;                            /* FIR filtered friction coefficient value */
static int FIRmu_index = 0;                         /* friction coefficient filter index */

// Buffers for calculating derivatives
static real_T deltaDeriv[3];

// CRAIG: added this debug array
static real_T debug[3];

/* initialize buffered previous values to zero */
static real_T MzLprev = 0;
static real_T MzRprev = 0;
static real_T Fyflefthatprev = 0;
static real_T Fyfrighthatprev = 0;
static real_T Fyfhatprev = 0;
static real_T If_leftprev = 0;
static real_T If_rightprev = 0;
static real_T tm_lprev = 0;
static real_T tm_rprev = 0;
static int tpL_error = 0;
static int tpR_error = 0;
static int ifL_mode = 0;
static int ifR_mode = 0;
static real_T alphadot = 0;

/* function prototypes */
static real_T LocalStiffness(real_T mu_ratio, real_T If_val, real_T alpha, real_T Ca);
static real_T Fiala(real_T mu_ratio, real_T If_val, real_T alpha, real_T Ca);
static void CoupledFiala(real_T mu_ratio, real_T If_val, real_T alpha, real_T kappa, real_T Ca, real_T Cx, real_T *Fy, real_T *Fx);
static real_T Sign(real_T x);
static void FIR_update(int aveLen, real_T newVal, real_T *FIRave, real_T *buffer, int *currentIndex);
static int GetPneumaticTrail(real_T * tp, const real_T tp0, real_T Fyhatprev, real_T Mzprev, real_T tmprev);
static int PeakForceEstimator(real_T * If_calc, real_T tp, real_T tpthres, real_T alphahat, real_T alphathres, real_T If_nom, real_T Ca, real_T Iforce_prev, real_T tp0, real_T tm, real_T Mz, real_T mu_ratio);

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
		ssSetInputPortRequiredContiguous(S, i, 1);    /* Set this for all input ports so that only a single pointer is needed to locate the data */
    }                                          
    
    if (!ssSetNumOutputPorts(S, NUMOFOUTPUTS))      /* set # of outputs */
    	return;
    for(i=0;i<NUMOFOUTPUTS;i++)
    {
        ssSetOutputPortWidth(S, i, OUTPUTWIDTHS[i]);
    }
    
    ssSetNumSampleTimes(S, 1);
    
    /* Extra storage variables, unnecessary for this observer. */
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
    /* Use the sampling time from the model */
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);   
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS
#ifdef MDL_INITIALIZE_CONDITIONS
/* ============================================================================================== *
 * Set initial conditions for the states
 * ============================================================================================== */
static void mdlInitializeConditions(SimStruct *S)
{
    /* declare x as a pointer referencing the states */
    real_T *x=ssGetRealDiscStates(S);
    /* grab the necessary parameters */
    const real_T a=*mxGetPr(ssGetSFcnParam(S,1));
    const real_T b=*mxGetPr(ssGetSFcnParam(S,2));
    const real_T m=*mxGetPr(ssGetSFcnParam(S,3));
    const real_T tp0L=*mxGetPr(ssGetSFcnParam(S,5));
    const real_T tp0R=*mxGetPr(ssGetSFcnParam(S,6));
    const real_T mu_peak_nom=*mxGetPr(ssGetSFcnParam(S,13));
    int i;
    
    /* Initialize the states */
    x[0]=0;                             /* Initial alpha_f is zero (remember c indexes starting at 0!) */
    x[1]=mu_peak_nom*m*g*b/(b+a);       /* Initial guess of front peak lateral force */
    x[2]=0;                             /* Initial estimation_flag is zero, is set to one once estimation begins */
    x[3] = x[4] = 0;                    /* Initial pneumatic trail (l,r) storage values */

    for (i = 0; i<TPWINDOW_INT; i++) {
        FIRafhat_buffer[i] = 0;
    }
    for (i = 0; i<TPWINDOW_INT; i++) {
        FIRtpL_buffer[i] = tp0L;
    }
    for (i = 0; i<TPWINDOW_INT; i++) {
        FIRtpR_buffer[i] = tp0R;
    }
    for (i = 0; i<IFWINDOW_INT; i++) {
        FIRIf_frontbuffer[i] = 1/x[1];
    }
    for (i = 0; i<MUWINDOW_INT; i++) {
        FIRmu_buffer[i] = x[1];
    }
    /* initialize the FIR averages to tp0 for the pneumatic trail FIRs */
    FIRtpL = tp0L;
    FIRtpR = tp0R;
    /* initialize the inverted peak force to the inverse of the guessed initial peak force */
    FIRIf_front = 1/x[1];
    FIRmu = x[1];
    
    /* initialize all storage variables to zero */
    MzLprev = 0;
    MzRprev = 0;
    Fyflefthatprev = 0;
    Fyfrighthatprev = 0;
    Fyfhatprev = 0;
    If_leftprev = 0;
    If_rightprev = 0;
    tm_lprev = 0;
    tm_rprev = 0;
}
#endif /* MDL_INITIALIZE_CONDITIONS */

/* ============================================================================================== *
 * Set outputs as a function of the states
 * ============================================================================================== */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    /* rename outputs */
    real_T *ahats=ssGetOutputPortRealSignal(S,0);    // FIR filtered afhat
    real_T *peakFyf=ssGetOutputPortRealSignal(S,1);
    real_T *estimation_flags=ssGetOutputPortRealSignal(S,2);
    real_T *pneumatic_trail=ssGetOutputPortRealSignal(S,3);
    real_T *peak_forces=ssGetOutputPortRealSignal(S,4);
    
    /* get offset parameter */
    const real_T alphafoffset=*mxGetPr(ssGetSFcnParam(S,11));
    
    /* get states. 'const' doesn't allow state modification */
    const real_T *x=ssGetRealDiscStates(S);    
    
    /* specify the value for the outputs */
    // Account for offset of tire curve due to unevenly worn tires
    ahats[0]=x[0] + alphafoffset;
    ahats[1]=debug[2];
    
    // Output the peak force value
    //*peakFyf=x[1];
    // CRAIG: use the next line for the conditionally filtered value
    *peakFyf=FIRmu;
    
    // Output all the estimation flags
    estimation_flags[0]=x[2]; /* Main estimation flag */
    estimation_flags[1]=tpL_error;
    estimation_flags[2]=tpR_error;
    estimation_flags[3]=ifL_mode;
    estimation_flags[4]=ifR_mode;
    
    /* CRAIG: Added these outputs for debugging purposes. If any of these
     * change, then the plotFrictionEst.m file should also be updated
     * since it references these outputs by order. */
    pneumatic_trail[0]=x[3];
    pneumatic_trail[1]=x[4];
    
    peak_forces[0] = debug[0];
    peak_forces[1] = debug[1];
}

/* ==================================================================== *
 * UPDATE STATE EQUATIONS (i.e. this is x(k+1) = A*x(k))
 * ==================================================================== */
#define MDL_UPDATE 
#ifdef MDL_UPDATE
/* Function: mdlUpdate =================================================
 * Abstract:
 *    This function is called once for every major integration time step.
 *    Discrete states are typically updated here, but this function is 
 *    useful for performing any tasks that should only take place once per
 *    integration step.
 */
static void mdlUpdate(SimStruct *S, int_T tid)
{
    /* name inputs */
    /*ssGetInputPortRealSignal returns a pointer referencing where the inputs are stored */
    const real_T delta=*ssGetInputPortRealSignal(S,0);
    const real_T V=*ssGetInputPortRealSignal(S,1);
    const real_T r=*ssGetInputPortRealSignal(S,2);
    const real_T beta=*ssGetInputPortRealSignal(S,3);
    const real_T ay=*ssGetInputPortRealSignal(S,4);
    const real_T *w_wheel=ssGetInputPortRealSignal(S,5);
    const real_T *Fz=ssGetInputPortRealSignal(S,6);
    const real_T *sat=ssGetInputPortRealSignal(S,7);
    const real_T *tm=ssGetInputPortRealSignal(S,8);
    const int AtShoreline=(int)*ssGetInputPortRealSignal(S,9);
    const int EnableObserver=(int)*ssGetInputPortRealSignal(S,10);

    /* name parameters */
    /* ssGetSFcnParam returns parameter array, mxGetPr gets a pointer to it */
    /* sample time */
    const real_T ts=*mxGetPr(ssGetSFcnParam(S,0));
    /* parameters for the vehicle */
    const real_T a=*mxGetPr(ssGetSFcnParam(S,1));
    const real_T b=*mxGetPr(ssGetSFcnParam(S,2));
    const real_T m=*mxGetPr(ssGetSFcnParam(S,3));
    const real_T Iz=*mxGetPr(ssGetSFcnParam(S,4));
    const real_T tp0L=*mxGetPr(ssGetSFcnParam(S,5));
    const real_T tp0R=*mxGetPr(ssGetSFcnParam(S,6));
    const real_T Cafasphalt=*mxGetPr(ssGetSFcnParam(S,7));
    const real_T Carasphalt=*mxGetPr(ssGetSFcnParam(S,8));
    const real_T Caf_shoreline_sf=*mxGetPr(ssGetSFcnParam(S,9));
    const real_T Car_shoreline_sf=*mxGetPr(ssGetSFcnParam(S,10));
    const real_T alphafoffset=*mxGetPr(ssGetSFcnParam(S,11));
    const real_T alpharoffset=*mxGetPr(ssGetSFcnParam(S,12));
    /* known or nominal surface properties */
    const real_T mu_peak_nom=*mxGetPr(ssGetSFcnParam(S,13));
    /* estimator gains and thresholds */
    const real_T Kaf0=*mxGetPr(ssGetSFcnParam(S,14)); 
    const real_T Kaf_sf_Shoreline=*mxGetPr(ssGetSFcnParam(S,15)); 
    const real_T Kaf_sf_asphalt=*mxGetPr(ssGetSFcnParam(S,16));
    const real_T Vfloor=*mxGetPr(ssGetSFcnParam(S,17));
    const real_T ayfloor=*mxGetPr(ssGetSFcnParam(S,18));
    const real_T alpha_thres=*mxGetPr(ssGetSFcnParam(S,19));
    const real_T afhatceil=*mxGetPr(ssGetSFcnParam(S,20));
    const real_T kappaceil=*mxGetPr(ssGetSFcnParam(S,21));
    
    /* assign x to point to the states */
    real_T *x=ssGetRealDiscStates(S);
    
    /* Pointers for convenient reference to state variables */
    real_T *peakFyf, *afhat, *estimation_flag, *pneumatic_trail;
    
    // Local variables for the update function
    real_T Caf, Car, Cx;         // Cornering stiffnesses used locally
    real_T arhat;            // Rear axle slip angle estimate
    real_T Fnf, Fnr;         // Front and rear normal forces
    real_T Fyfhat, Fyrhat, Fyflefthat, Fyfrighthat, Fyfmeas;
    real_T tpL, tpR, tpthres;
    real_T If_nom_l, If_nom_r, If_nom;
    real_T If_left, If_right, If_front, If_rear, mu_p_front;
    real_T deltadot;
    real_T Kaf, mu_ratio;
    real_T Kaf_sf;
    real_T VwheelL, VwheelR, kappaL, kappaR, FyL, FyR, FxL, FxR;
    
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
    Cx = 30000;
    
    // Calculate normal load on front and rear axles
    Fnf = m*g*b/(a+b);               // nominal front axle normal force
    Fnr = m*g - Fnf;                 // nominal rear axle normal force
    
    // Assume a single-coefficient model (for now, can change)
    mu_ratio = 1.0;                   // ratio of slip to peak friction (.)
    If_nom = 1/(mu_peak_nom*Fnf);     // nominal peak FRONT AXLE force
    
    // assign pointers to point to the states
    afhat=x;
    peakFyf=(x+1);
    estimation_flag=(x+2);
    pneumatic_trail=(x+3);

    /********************************************************************
     * Common tasks that are done regardless of whether estimates will
     * be updated on this time step.
     ********************************************************************/
    
    /* Update the derivative buffer */
    deltaDeriv[0] = delta;
    /* Calculate the derivative of the steering angle */
    if (*estimation_flag != OFF) {      // "if" condition
        // Using a simple difference
        //deltadot = (deltaDeriv[0] - deltaDeriv[1])/ts;
        // Using a 3-point backward differentiation formula
        deltadot = (1.5*deltaDeriv[0] - 2*deltaDeriv[1] + 0.5*deltaDeriv[2])/ts;
    }
    else if (*estimation_flag == OFF) {// we weren't estimating in previous time step
        // set derivatives to zero
        deltadot = 0;
    }
    // Shift deltas for next time step
    deltaDeriv[1] = deltaDeriv[0];
    deltaDeriv[2] = deltaDeriv[1];
    
    // Calculate If_nom values for the right and left sides of
    // car based on the calculated normal loads (basically just the
    // proportion of front axle load inverted)
    If_nom_l = If_nom*(Fz[0]+Fz[1])/Fz[0]; 
    If_nom_r = If_nom*(Fz[0]+Fz[1])/Fz[1];
    
    // Calculate the previous front force estimate for each side
    If_leftprev = FIRIf_front*(Fz[0]+Fz[1])/Fz[0];
    If_rightprev = FIRIf_front*(Fz[0]+Fz[1])/Fz[1];
                
    // Calculate the velocities of the right and left rear wheel centers
    VwheelL = V - (0.81)*r;
    VwheelR = V + (0.81)*r;
            
    // Calculate the slips on the right and left rear wheels
    kappaL = (0.31*w_wheel[2] - VwheelL)/VwheelL;
    kappaR = (0.31*w_wheel[3] - VwheelR)/VwheelR;
            
    /********************************************************************
     * Conditional code, depending on whether or not the estimates can
     * be updated accurately at this time step.
     ********************************************************************/
    // update estimates if estimates are reasonable, V is above Vfloor, 
    // and the observer is turned on, otherwise, hold slip angle at zero
    // and friction coefficient at last good value
    if ((fabs(*afhat) > afhatceil) || (V < Vfloor) || !EnableObserver)
    {
        // set estimation_flag to zero (estimation is OFF)
        estimation_flag[0] = OFF;
        // default estimates when inputs are bogus or estimates have gone unstable
        afhat[0] = 0;
        // CRAIG: the next line sets the estimator to hold its last
        // estimate when it cannot generate a new one
        peakFyf[0] = 1/FIRIf_front; //mu_peak_nom*Fz_f*2.0;
    }
    // If the execution reaches this point, the observer should be
    // doing some sort of estimation. However, it may be degraded if the
    // vehicle dynamics are not a decent estimation range.
    else
    {
        // Set estimation_flag to nominal
        estimation_flag[0] = OPTIMAL;

        /****************************************************************
         * Observer Phase 1: Estimate pneumatic trail and lateral forces
         * for both the L&R wheels
         ****************************************************************/
       
        // Check to see if there is enough excitation to the system
        // to develop a decent estimation. The front slip angle seems to
        // be a good enough indicator of this so that it is not necessary
        // to look at other values.
        if ( fabs(*afhat) < alpha_thres ) // Not enough excitation
        {
            // Indicate that the estimator is using a linear estimate
            estimation_flag[0] = LINEAR_REG;
            
            // The pneumatic trail should go back to the initial value in
            // this case since if there isn't much slip angle, there isn't 
            // much force and therefore no loss of tire grip.
            FIR_update(TPWINDOW_INT, tp0L, &FIRtpL, FIRtpL_buffer, &FIRtpL_index);
            FIR_update(TPWINDOW_INT, tp0R, &FIRtpR, FIRtpR_buffer, &FIRtpR_index);
            tpL_error = NO_ERROR;
            tpR_error = NO_ERROR;
            
            // Wheel peak force estimates
            // To hold the last estimate before reaching this condition,
            // use this next line.
            If_left = If_leftprev;
            If_right = If_rightprev;
            // To reset to nominal, use the next lines.
            //If_left = If_nom_l;
            // If_right = If_nom_r;
            
            // Calculate a lateral force update
            Fyfrighthat = Fiala(mu_ratio, If_right, *afhat, Caf/2);
            Fyflefthat = Fiala(mu_ratio, If_left, *afhat, Caf/2);
        }
        else
        {
            // Use estimated tp to solve for If for each front tire
            /* Front LEFT */
            tpL_error = GetPneumaticTrail(&tpL, tp0L, Fyflefthatprev, MzLprev, tm_lprev);
            // Check the validity flag to see if the pneumatic trail estimate
            // is good enough to use for an update
            /*if(tpL_error == TP_RANGE_HIGH)
                tpL = tp0L;*/
            FIR_update(TPWINDOW_INT, tpL, &FIRtpL, FIRtpL_buffer, &FIRtpL_index);
            
            // Check if pneumatic trail is > tp0L or slip angle is below slip angle estimation threshold
            if ( tpL_error > 1) {
                // To hold the last estimate before reaching this condition,
                // use this next line.
                If_left = If_leftprev;
                //If_left = If_nom_l;
                Fyflefthat = -Caf/2*(*afhat);
                //printf("Fails tp/alpha test for left wheel\r\n");
            }
            else {
                // CRAIG: hacked the threshold value (2nd param)
                ifL_mode = PeakForceEstimator(&If_left, FIRtpL, tp0L, *afhat, alpha_thres, If_nom_l, Caf/2, If_leftprev, tp0L, tm[0], sat[0], mu_ratio);
                // CRAIG: Do some sanity checking
                /*if (If_left < If_leftprev*0.2)
                {
                    If_left = If_leftprev*0.2;
                    tpL_error = TP_SANITY_FAIL;
                }*/
                //If_left = max(If_nom_l,If_left);
                // Solve for current lateral force estimate (if no lateral excitation, then use linear tire model)
                Fyflefthat = Fiala(mu_ratio, If_left, *afhat, Caf/2);
            }
            
            /* Front RIGHT */
            tpR_error = GetPneumaticTrail(&tpR, tp0R, Fyfrighthatprev, MzRprev, tm_rprev);
            // Check the validity flag to see if the pneumatic trail estimate
            // is good enough to use for an update
            /*if(tpR_error == TP_RANGE_HIGH)
                tpR = tp0R;*/
            FIR_update(TPWINDOW_INT, tpR, &FIRtpR, FIRtpR_buffer, &FIRtpR_index);
            
            
            // Check if pneumatic trail is > tp0R or slip angle is below slip angle estimation threshold
            if ( tpR_error > 1 ) 
            {
                // To hold the last estimate before reaching this condition,
                // use this next line.
                If_right = If_rightprev;
                // Or, to reset to nominal, use this next line.
                // If_right = If_nom_r;
                Fyfrighthat = -Caf/2*(*afhat);
            }
            else {
                // CRAIG: hacked the threshold value (2nd param)
                ifR_mode = PeakForceEstimator(&If_right, FIRtpR, tp0R, *afhat, alpha_thres, If_nom_r, Caf/2, If_rightprev, tp0R, tm[1], sat[1], mu_ratio);
                // CRAIG: Do some sanity checking
                /*if (If_right < If_rightprev*0.2)
                {
                    If_right = If_rightprev*0.2;
                    tpR_error = TP_SANITY_FAIL;
                }*/
                // If_right = max(If_nom_r,If_right);
                // Solve for current lateral force estimate (if no lateral excitation, then use linear tire model)
                Fyfrighthat = Fiala(mu_ratio, If_right, *afhat, Caf/2);
            }
        }
        
        /****************************************************************
         * Observer Phase 2: Combine L&R values to get vehicle level
         * force estimates.
         ****************************************************************/
        
        if (estimation_flag[0] == LINEAR_REG) {
            // Average together the estimates of actual front force.
            Fyfhat = Fyflefthat + Fyfrighthat;
        }
        else {
            // Find front axle If value, using the flags to determine whether
            // estimates from the left and right sides are good
            
            // In this case, both sides are less than tp0, so there should
            // be good estimates to work with here.
            if(tpL_error <= 1 && tpR_error <= 1){
                // Combine left and right sides peak and actual forces
                If_front = 1/(1/If_left + 1/If_right);
                Fyfhat = Fyflefthat + Fyfrighthat;
                // Update the FIR filtered value of peak front force
                FIR_update(IFWINDOW_INT, If_front, &FIRIf_front, FIRIf_frontbuffer, &FIRIf_front_index);
            }
            // If only the left side is OK, use the left side to estimate
            // peak force and a linear estimate for force on the right
            else if (tpL_error <= 1){
                estimation_flag[0] = LEFT_ONLY;
                If_front = If_left*Fz[0]/(Fz[0]+Fz[1]);
                Fyfhat = Fyflefthat*(Fz[0]+Fz[1])/Fz[0];
                // Update the FIR filtered value of peak front force
                FIR_update(IFWINDOW_INT, If_front, &FIRIf_front, FIRIf_frontbuffer, &FIRIf_front_index);
            }
            else if (tpR_error <= 1){
                estimation_flag[0] = RIGHT_ONLY;
                If_front = If_right*Fz[1]/(Fz[0]+Fz[1]);
                Fyfhat = Fyfrighthat*(Fz[0]+Fz[1])/Fz[1];
                // Update the FIR filtered value of peak front force
                FIR_update(IFWINDOW_INT, If_front, &FIRIf_front, FIRIf_frontbuffer, &FIRIf_front_index);
            }
            else {
                // This case is because neither side of the car produced
                // a good estimate of pneumatic trail. The next line sets
                // the flag appropriately
                estimation_flag[0] = DEGRADED;
                
                // By not adding a new value with FIR_update, the estimate
                // remains unaltered for the inverted peak force estimate
                
                // If the estimates are bad, this assumes that averaging 
                // them together helps. Produce an estimate of the lateral
                // force for use in the next time step
                Fyfhat = Fyflefthat + Fyfrighthat;
            }
        }
        Fyfhatprev = Fyfhat;
        
        // Calculate front friction coefficient with filtered value
        mu_p_front = 1/(FIRIf_front*Fnf);
        // Assuming friction is the same for front/rear, find If value for rear axle
        If_rear = 1/(mu_p_front*Fnr);
        // Calculate estimated rear slip angle
        arhat = atan(tan(*afhat) + delta - (a+b)/V*r) + alpharoffset;
        debug[2] = arhat;
        // Calculate estimated front/rear tire forces
        // CRAIG: Trying a coupled model
        //CoupledFiala(mu_ratio, If_rear*2, arhat, kappaL, Car/2, Cx, &FyL, &FxL);
        //CoupledFiala(mu_ratio, If_rear*2, arhat, kappaR, Car/2, Cx, &FyR, &FxR);
        //Fyrhat = FyL + FyR;
        // CRAIG: Old way
        Fyrhat = Fiala(mu_ratio, If_rear, arhat, Car);
        
        /****************************************************************
         * Observer Phase 3: Update state estimates
         ****************************************************************/
        
        // Observer gain: the sum of the local cornering stiffness divided
        // by twice the sum of the nominal cornering stiffnesses
        //Kaf = LocalStiffness(mu_ratio, FIRIf_front, *afhat, Caf);// + LocalStiffness(mu_ratio, FIRIf_front, arhat, Car);
        //Kaf *= Kaf_sf/pow(Caf,2.0);     // hand-tuned gain
        Kaf = Kaf0*Kaf_sf;
        
        // Update front slip angle estimate
        afhat[0] += ((1/(m*V)+a*a/(Iz*V))*Fyfhat+(1/(m*V)-a*b/(Iz*V))*Fyrhat-r-deltadot)*ts+Kaf*(Fyfhat + Fyrhat - m*ay);
         
        /****************************************************************
         * Final Phase: update outputs and store values for next step
         ****************************************************************/
               
        // Conditionally update mu for output
        if(fabs(deltadot) <= 0.0005 ){
            FIR_update(MUWINDOW_INT, 1/FIRIf_front, &FIRmu, FIRmu_buffer, &FIRmu_index);
        }
        
        // Store the peak force value
        *peakFyf = 1/FIRIf_front;
        
        // Store the pneumatic trail values for output
        pneumatic_trail[0] = FIRtpL;
        pneumatic_trail[1] = FIRtpR;
            
        // CRAIG: store the independent peak force estimates
        debug[0] = kappaL; //1/If_left*(Fz[0]+Fz[1])/Fz[0];
        debug[1] = kappaR; //1/If_right*(Fz[0]+Fz[1])/Fz[1];
        
        // Update left side values for next time step
        MzLprev = sat[0];
        Fyflefthatprev = Fyflefthat;
        tm_lprev = tm[0];
        If_leftprev = If_left;
        
        // Update right side values for next time step
        MzRprev = sat[1];
        Fyfrighthatprev = Fyfrighthat;
        tm_rprev = tm[1];
        If_rightprev = If_right;
        
	}
}
#endif /* MDL_UPDATE */

/*================*
 * help functions *
 *================*/

/* Local Cornering Stiffness */
static real_T LocalStiffness(real_T mu_ratio, real_T If_val, real_T alpha, real_T Ca)
{
    real_T C1, C2, C3, secalph, tanalph;
    
    /* Define some convenient coefficients */
    C1 = Ca;
    C2 = -pow(Ca, 2.0)*If_val/3.0;
    C3 = pow(Ca, 3.0)*pow(If_val,2)/27.0;
    
    /* Work only in positive slip angles */
    alpha = fabs(alpha);
    
    /* Calculate the angle of full tire sliding */
    if(alpha < atan(3.0/(If_val*Ca))) {
        /* Precalculate the slow trig operations that get used multiple times */
        tanalph = tan(alpha);
        secalph = 1.0/cos(alpha);
        /* Calculate the tire slope from the slip angle */
        return (C1*pow(secalph, 2.0) + 2.0*C2*tanalph*pow(secalph, 2.0) + 3*C3*pow(tanalph, 2.0)*pow(secalph, 2.0));
    }
    else {
        /* The slope after full sliding is zero */
        return 0;
    }
   
}

/* Fiala Force model */
static real_T Fiala(real_T mu_ratio, real_T If_val, real_T alpha, real_T Ca)
{
    // Save a few cpu cycles by computing this value once
    real_T tanalpha = tan(alpha);
    
    // Double check to make sure the peak force value is OK
    if (If_val > 0) {
        // Calculate the force value before or after the angle of full sliding
        if (fabs(alpha) < atan(3/(Ca*If_val))) {
            return -Ca*tanalpha + Ca*Ca*If_val/3.0*(2-mu_ratio)*fabs(tanalpha)*tanalpha - Ca*Ca*Ca*If_val*If_val/9.0*(1-2.0/3.0*mu_ratio)*tanalpha*tanalpha*tanalpha;
        }   
        else return (-mu_ratio/If_val*Sign(alpha));
    }
    else return 0;
}

/* Coupled Fiala Force model */
static void CoupledFiala(real_T mu_ratio, real_T If_val, real_T alpha, real_T kappa, real_T Ca, real_T Cx, real_T *Fy, real_T *Fx)
{
    // Intermediate Variables
    real_T f, F;
    
    // Save a few cpu cycles by computing this value once
    real_T tanalpha = tan(alpha);
    
    // Double check to make sure the peak force value is OK
    if (If_val > 0) {
        // Calculate total requested force
        f = sqrt(Cx*Cx*pow(kappa/(1+kappa),2.0) + Ca*Ca*pow(tanalpha/(1+kappa),2.0));
        // Calculate the force value before or after reaching combined sliding
        if (f <= 3/If_val) {
            F = f - 1/3*If_val*(2-mu_ratio)*pow(f,2.0) + 1/9*pow(If_val,2.0)*(1 - 2/3*mu_ratio)*pow(f,3.0);
        }   
        else 
            F = 1/If_val;
        // Calculate the individual forces
        *Fx = Cx*(kappa/(1+kappa))/f*F;
        *Fy = -Ca*(tanalpha/(1+kappa))/f*F;
    }
    // If no peak force, then output zero for each force
    *Fx = 0;
    *Fy = 0;
    
    return;
}

/* An efficient FIR filter implementation */
static void FIR_update(int aveLen, double newVal, double *FIRave, double *buffer, int *currentIndex) 
{
    // Save the buffer value about to be over-written
    double oldVal = buffer[*currentIndex];
    // Overwrite the buffer with the new value
    buffer[*currentIndex] = newVal;
    // Update the index of the ring buffer
    *currentIndex = (*currentIndex + 1) % aveLen;
    // Now update the average by subtracting out the weighted value being removed
    // and adding in the weighted value being added
    *FIRave = *FIRave + (newVal - oldVal)/((double)aveLen);
}

/* Get pneumatic trail estimate from total aligning moment, mechanical trail and estimated lateral force */
static int GetPneumaticTrail(real_T *tp, const real_T tp0, real_T Fyhatprev, real_T Mzprev, real_T tmprev)
{
    /* GetPneumaticTrail returns the pneumatic trail calculated from 
     * steering aligning moment, lateral force estimate and mechanical
     * trail. */
    
    // CRAIG: This condition should now be handled by not calling this
    // function when the slip angle is too low (hence too little force)
    /*if ( fabs(Fyhatprev) > 200 )*/{
    // if ((count > tp_window+1) && (fabs(Fyhatprev) > 0)) {
        // CRAIG: This if-else does nothing, since the condition is
        // satisfied just by getting to this point.
        // Calculate new tp value
        /*if (fabs(Fyhatprev) > 0) {
            // If Fyhatprev is nonzero, then calculate new tp value
            tpnew = -(Mzprev/Fyhatprev+tmprev);
        }
        // otherwise, use last tp value as current value
        else tpnew = tpprev[tp_window_int-1]; 
        */
        // CRAIG: this might be wrong, but it seems like no matter which
        // direction the force is in, the pneumatic trail has to stay
        // positive. Therefore, instead of allowing the signs to cancel
        // in normal cases, just take the absolute value and subtract
        // the mechanical trail.
        // tp = abs(Mzprev/Fyhatprev) - tmprev;
        *tp = -Mzprev/Fyhatprev - tmprev;
        // CRAIG: Set error flags if the pneumatic trail is out of range
        if (*tp > tp0) {
            *tp = tp0;
            return TP_RANGE_HIGH;
        }
        else if (*tp < 0) {
            *tp = 0;
            return TP_RANGE_LOW;
            //printf("Getting a negative pneumatic trail\n");
        }
        return NO_ERROR;
    }
    // If lateral force is too low, return the initial pneumatic trail
    /*else
    {
        *tp = tp0;
    }*/
}   

/* Peak lateral force estimator */
static int PeakForceEstimator(real_T *If_calc, real_T tp, real_T tpthres, real_T alphahat, real_T alphathres, real_T If_nom, real_T Ca, real_T Iforce_prev, real_T tp0, real_T tm, real_T Mz, real_T mu_ratio) 
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

    // CRAIG: Because of if-then in the calling function code, this case
    // should never be reached. Therefore, the flagging that is done for
    // this case in the calling code will handle this as well. In the long
    // run, this should be taken out.
    /*if (tp >= tpthres){
        printf("Gets to pneumatic trail check in function\n");
        return If_nom;
    }
    if (fabs(alphahat) < alphathres) {
        printf("Gets to slip angle check in function\n");
        return If_nom;
    }
    else*/ {
        // Before full slide, algebraically solve for inverted force
        if (fabs(alphahat) < atan(3.0/(Ca*Iforce_prev))) {
            *If_calc = (-3.0*(tp - tp0)/(tp0*Ca*fabs(tan(alphahat))));
            return IF_NORMAL;
        }
        // Since tp = 0 at full slide, use Mz info to solve for If
        else {
            *If_calc = (Sign(alphahat)*tm*mu_ratio/Mz);
            return IF_SLIDE;
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
