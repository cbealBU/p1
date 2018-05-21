/*drift_speed_controller.c
 *Rami Y. Hindiyeh
 *DDL
 *
 *Executes steering and throttle commands for open loop initiation and closed loop control of a drift
*/
#define S_FUNCTION_NAME  drift_speed_controller
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>
#include <stdio.h>

#define NUMOFPARAMS (24)
#define NUMOFINPUTS (12)
#define NUMOFOUTPUTS (5)

//Controller Sequence States
#define USER_CONTROL (0)
#define ACCELERATE (1)
#define HOLD (2)
#define AT_LIMITS (3)
#define DESTABILIZE (4)
#define DRIFT (5)

//Minimum speed for slip angle computation
#define MIN_SPEED (0.5)
//Voltage Characteristics of Traction Command
#define ZERO_TORQUE_OFFSET (1.95)
#define REGEN_VOLTAGE (1.1)
#define MAX_VOLTAGE (2.8)

#define PI (3.1415926535897932384626433832795)
#define GRAVACC (9.81)

#define TRU (1)
#define FALS (0)

//Module level variables
static real_T tStart;
static real_T tStartHold;
static real_T tStartLimits;
static int_T NextState;
static real_T r_des0;
static real_T tDrift;

/*
 *HELPER FUNCTIONS
 */

static real_T CalcNonLinearTireForce(real_T Fz, real_T Fx, real_T mu_p, real_T mu_s, real_T Calpha, real_T alpha){
    real_T alpha_sl;
    real_T eta; 
    real_T tan_alpha;
    real_T Fy;
    

    eta = sqrt(fabs(pow(mu_s,2)*pow(Fz,2) - pow(Fx,2)))/(mu_s*Fz);

    alpha_sl = atan2(3*eta*mu_p*Fz, Calpha);
    
//     printf("difference = %f\r\n", pow(mu_s,2)*pow(Fz,2) - pow(Fx,2));
    
    if (eta == 0) {
        Fy = 0;
    }
    else {
        if (fabs(alpha) < alpha_sl) {
            tan_alpha = tan(alpha);
            Fy = -Calpha*tan_alpha + pow(Calpha,2)*(2 - mu_s/mu_p)*fabs(tan_alpha)*tan_alpha/(3*eta*mu_p*Fz) - pow(Calpha,3)*pow(tan_alpha,3)*(1-2*mu_s/(3*mu_p))/(9*pow(eta,2)*pow(mu_p,2)*pow(Fz,2));

        }
        else {
            if (alpha > 0) {
                Fy = -eta*mu_s*Fz; 
            }
            else if (alpha < 0) {
                Fy = eta*mu_s*Fz;
            }
        }
    }
    
    return Fy;
}

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUMOFPARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        ssSetErrorStatus(S,"Incorrect number of parameters.");
        return;
    }
    
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 1)) {
    	return;
    }
    
    ssSetInputPortWidth(S, 0, NUMOFINPUTS);

    
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    
    if (!ssSetNumOutputPorts(S, 1)) {
    	return;
    }
    ssSetOutputPortWidth(S, 0, NUMOFOUTPUTS);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, 0);

}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    const real_T Ts = mxGetPr(ssGetSFcnParam(S,23))[0];
    
    ssSetSampleTime(S, 0, Ts);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
static void mdlInitializeConditions(SimStruct *S)
{
    tStart = 0;
    tStartHold = 0;
    tStartLimits = 0;
    NextState = USER_CONTROL;
    r_des0 = 0;
    tDrift = 0;
}

#undef MDL_DERIVATIVES


static void mdlOutputs(SimStruct *S, int_T tid)
{
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    //Openloop params
    const real_T axInit = mxGetPr(ssGetSFcnParam(S,0))[0];
    const real_T tHold = mxGetPr(ssGetSFcnParam(S,1))[0];
    const real_T UxLimits = mxGetPr(ssGetSFcnParam(S,2))[0];
    const real_T deltaLimits = mxGetPr(ssGetSFcnParam(S,3))[0]; 
    const real_T tLimits = mxGetPr(ssGetSFcnParam(S,4))[0]; 
    const real_T KUx_N = mxGetPr(ssGetSFcnParam(S,5))[0]; 
    //Drift initiation params
    const real_T betaThreshold = mxGetPr(ssGetSFcnParam(S,6))[0]; 
    const real_T rThreshold = mxGetPr(ssGetSFcnParam(S,7))[0]; 
    //Vehicle params
    const real_T VoltsToTorque = mxGetPr(ssGetSFcnParam(S,8))[0];
    const real_T VoltsToTorqueRegen = mxGetPr(ssGetSFcnParam(S, 9))[0];
    const real_T gr = mxGetPr(ssGetSFcnParam(S,10))[0];
    const real_T r_w = mxGetPr(ssGetSFcnParam(S,11))[0];
    const real_T M = mxGetPr(ssGetSFcnParam(S, 12))[0];
    const real_T Iz = mxGetPr(ssGetSFcnParam(S, 13))[0];
    const real_T a = mxGetPr(ssGetSFcnParam(S, 14))[0];
    const real_T b  = mxGetPr(ssGetSFcnParam(S, 15))[0];
    const real_T c = mxGetPr(ssGetSFcnParam(S, 16))[0];
    const real_T L = a+b;
    const real_T Clf = mxGetPr(ssGetSFcnParam(S, 17))[0];
    const real_T Crf = mxGetPr(ssGetSFcnParam(S, 18))[0];
    const real_T Clr = mxGetPr(ssGetSFcnParam(S, 19))[0];
    const real_T Crr = mxGetPr(ssGetSFcnParam(S, 20))[0];
    const real_T Cf = Clf + Crf;
    const real_T Cr = Clr + Crr;
    const real_T Fzf = (b/L)*M*GRAVACC; //Nominal front axle normal load
    const real_T Fzr = (a/L)*M*GRAVACC; //Nominal rear axle normal load
    const real_T TorqueToVolts = 1/VoltsToTorque;
    const real_T KUx_V = 0.5*KUx_N*r_w*(1/gr)*TorqueToVolts;
    //Assumed friction params
    const real_T mu_p_assumed = mxGetPr(ssGetSFcnParam(S,21))[0];
    const real_T mu_s_assumed = mxGetPr(ssGetSFcnParam(S,22))[0];
    //Sampling time
    const real_T Ts = mxGetPr(ssGetSFcnParam(S,23))[0];
    //Get current time
    real_T tCurrent = ssGetT(S);

    //Define input quantities
    real_T Enable, Ux, Uy, beta, r, delta_driver, gopedal, delta_meas; 
    real_T delta_meas_l, delta_meas_r,UseTwoWheel, UseLongitudinalCoord, UseGalvezSequence; 
    
    //Define controller quantities
    real_T Fyf, Fylf, Fyrf, Fyr, Fxr, alpha_f, alpha_lf, alpha_rf, alpha_r, deltaFyf;
    //Define output quantities
    real_T delta_command,traction_command; 
    
    //Other quantities used in computation
    real_T tAccelerating, tHolding, tAtLimits, UxDes;
    int_T CurrentState; 
    
    //Inputs
    Enable = *uPtrs[0];
    Ux = *uPtrs[1];
    Uy = *uPtrs[2];
    beta = *uPtrs[3];
    r = *uPtrs[4];
    delta_driver = *uPtrs[5];
    gopedal = *uPtrs[6];
    delta_meas_l = *uPtrs[7];
    delta_meas_r = *uPtrs[8];
    Fxr = *uPtrs[9];
    UseGalvezSequence = *uPtrs[10];
    UseTwoWheel = *uPtrs[11];

       
    //State machine goes here
    CurrentState = NextState;
    
    if (CurrentState == USER_CONTROL) {
        delta_command = delta_driver;
        traction_command = gopedal;
        
        if (Enable == 1) {
            NextState = ACCELERATE;
            tStart = tCurrent;
        }
        else {
            NextState = USER_CONTROL;
        }
    }
    else if (CurrentState == ACCELERATE) {
        tAccelerating = tCurrent - tStart;
        delta_command = 0;
        if (UseGalvezSequence == 1) {
            delta_command = deltaLimits;
        }
        UxDes = axInit*tAccelerating + 0.5;  //0.5 is only for simulation;
        traction_command = ZERO_TORQUE_OFFSET + KUx_V*(Ux - UxDes);
        Fyf = 0;
        Fyr = 0;

        if (Enable == 0) {
            NextState = USER_CONTROL;
        }
        else if (UxDes >= UxLimits) {
            NextState = HOLD;
            tStartHold = tCurrent;
        }
        else {
            NextState = ACCELERATE;
        }
    }
    else if (CurrentState == HOLD) {
        tHolding = tCurrent - tStartHold;
        delta_command = 0;
        if (UseGalvezSequence == 1) {
            delta_command = deltaLimits;
        }
        UxDes = UxLimits;
        traction_command = ZERO_TORQUE_OFFSET + KUx_V*(Ux - UxDes);
        Fyf = 0;
        Fyr = 0;
        
        if (Enable == 0) {
            NextState = USER_CONTROL;
        }
        else if (tHolding >= tHold) {
            NextState = AT_LIMITS;
            tStartLimits = tCurrent;
        }
        else {
            NextState = HOLD;
        }
    }
    else if (CurrentState == AT_LIMITS) {
        tAtLimits = tCurrent - tStartLimits;
        delta_command = deltaLimits;
        UxDes = UxLimits;
        traction_command = ZERO_TORQUE_OFFSET + KUx_V*(Ux - UxDes);
        Fyf = 0;
        Fyr = 0;
       
        if (Enable == 0) {
            NextState = USER_CONTROL;
        }
        else if (tAtLimits >= tLimits) {
            NextState = DESTABILIZE;
        }
        else {
            NextState = AT_LIMITS;
        }
    }
    else if (CurrentState == DESTABILIZE) {
        delta_command = deltaLimits;
        traction_command = REGEN_VOLTAGE;
        Fyf = 0;
        Fyr = 0;

        if (Enable == 0) {
            NextState = USER_CONTROL;
        }
/*
        else if (beta <= betaThreshold) {
*/
        else if (beta <= betaThreshold ) {
            NextState = DRIFT;
            tDrift = tCurrent;
        }
        else {
            NextState = DESTABILIZE;
        }   
    }
    else if (CurrentState == DRIFT) {
        //Set open loop commands to zero
        delta_command = 0;
        traction_command = 0;
      
        //Compute front and rear tire slip angles
        if (Ux >= MIN_SPEED) {
            if (UseTwoWheel == 1) {
                delta_meas = 0.5*(delta_meas_l + delta_meas_r);
                alpha_f = atan2(Uy + a*r, Ux)-delta_meas;
            }
            else {
                alpha_lf = atan2(Uy + a*r, Ux - c*r) - delta_meas_l;
                alpha_rf = atan2(Uy + a*r, Ux + c*r) - delta_meas_r; 
            }
            alpha_r = atan2(Uy - b*r, Ux);   
        }
        else {
            alpha_f = 0;
            alpha_lf = 0;
            alpha_rf = 0;
            alpha_r = 0;
        }
        
        //Now compute rear lateral force
        Fyr = CalcNonLinearTireForce(Fzr, Fxr, mu_p_assumed, mu_s_assumed, Cr, alpha_r);
        
        //Compute front lateral force (just for debugging and comparison to Fyf command)
        if (UseTwoWheel == 1) {
            Fyf = CalcNonLinearTireForce(Fzf, 0, mu_p_assumed, mu_s_assumed, Cf, alpha_f);
        }
        else {
            Fylf = CalcNonLinearTireForce(Fzf/2, 0, mu_p_assumed, mu_s_assumed, Cf/2, alpha_lf);
            Fyrf = CalcNonLinearTireForce(Fzf/2, 0, mu_p_assumed, mu_s_assumed, Cf/2, alpha_rf);
            Fyf = Fylf + Fyrf;
        }
              
        if (Enable == 0) {
            NextState = USER_CONTROL;
        }
        else {
            NextState = DRIFT;
        }
    }
        
    y[0] = delta_command;
    y[1] = traction_command;
    y[2] = CurrentState;
    y[3] = Fyf;
    y[4] = Fyr;
}


static void mdlTerminate(SimStruct *S)
{

}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
