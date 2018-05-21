/*drift_controller_SDSC.c
 *Rami Y. Hindiyeh
 *DDL
 *
 *Executes steering and throttle commands for open loop initiation and closed loop control of a drift with a simplified dynamic
 *surface control scheme
*/
#define S_FUNCTION_NAME  drift_controller_SSC
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>
#include <stdio.h>

#define NUMOFPARAMS (30)
#define NUMOFINPUTS (8)
#define NUMOFOUTPUTS (9)

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
#define ZERO_TORQUE_OFFSET (1.7)
#define REGEN_VOLTAGE (0.9)
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
static real_T tDrift;

/*
 *HELPER FUNCTIONS
 */

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
    const real_T Ts = mxGetPr(ssGetSFcnParam(S,29))[0];
    
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
    const real_T KUxOL = mxGetPr(ssGetSFcnParam(S,5))[0]; 
    //Drift initiation params
    const real_T betaThreshold = mxGetPr(ssGetSFcnParam(S,6))[0]; 
    const real_T rThreshold = mxGetPr(ssGetSFcnParam(S,7))[0]; 
    //Equilibrium parameters
    const real_T UyEq = mxGetPr(ssGetSFcnParam(S,8))[0]; 
    const real_T rEq = mxGetPr(ssGetSFcnParam(S,9))[0]; 
    const real_T UxEq = mxGetPr(ssGetSFcnParam(S,10))[0]; 
    const real_T deltaEq = mxGetPr(ssGetSFcnParam(S,11))[0]; 
    const real_T FxrEq = mxGetPr(ssGetSFcnParam(S,12))[0];
    const real_T mu = mxGetPr(ssGetSFcnParam(S,13))[0];
    //Sliding surface parameters
    const real_T KUy = mxGetPr(ssGetSFcnParam(S,14))[0];
    const real_T Kr = mxGetPr(ssGetSFcnParam(S,15))[0];
    //Matrix elements for input solution
    const real_T k11 = mxGetPr(ssGetSFcnParam(S,16))[0];
    const real_T k12 = mxGetPr(ssGetSFcnParam(S,17))[0];
    const real_T k13 = mxGetPr(ssGetSFcnParam(S,18))[0];
    const real_T k21 = mxGetPr(ssGetSFcnParam(S,19))[0];
    const real_T k22 = mxGetPr(ssGetSFcnParam(S,20))[0];
    const real_T k23 = mxGetPr(ssGetSFcnParam(S,21))[0];
    //Assemble matrices
    //Vehicle params
    const real_T a = mxGetPr(ssGetSFcnParam(S,22))[0];
    const real_T b = mxGetPr(ssGetSFcnParam(S,23))[0];
    const real_T m = mxGetPr(ssGetSFcnParam(S,24))[0];
    const real_T L = a+b;
    const real_T Fzf = (b/L)*m*GRAVACC;
    const real_T Fzr = (a/L)*m*GRAVACC;
    const real_T VoltsToTorque = mxGetPr(ssGetSFcnParam(S,25))[0];
    const real_T VoltsToTorqueRegen = mxGetPr(ssGetSFcnParam(S, 26))[0];
    const real_T TorqueToVolts = 1/VoltsToTorque;
    const real_T TorqueToVoltsRegen = 1/VoltsToTorqueRegen;
    const real_T r_w = mxGetPr(ssGetSFcnParam(S, 27))[0];
    const real_T gr = mxGetPr(ssGetSFcnParam(S, 28))[0];
    const real_T KUxOL_V = 0.5*KUxOL*r_w*(1/gr)*TorqueToVolts;
    //Sampling time
    const real_T Ts = mxGetPr(ssGetSFcnParam(S,29))[0];
    //Get current time
    real_T tCurrent = ssGetT(S);

    //Define input quantities
    real_T Enable, Ux, Uy, beta, r, delta_driver, gopedal, UseGalvezSequence;
    
    //Controller quantities
    real_T eUy, er, eUx;
    //Define output quantities
    real_T delta_command, traction_command, Fxr, Ddelta, DFxr, s;
    
    //Other quantities used in computation
    real_T tAccelerating, tHolding, tAtLimits, UxDes;
    int_T CurrentState;
    
    //Inputs
    Enable = *uPtrs[0];
    Ux = *uPtrs[1];
    Uy = *uPtrs[2];
    r = *uPtrs[3];
    beta = *uPtrs[4];
    delta_driver = *uPtrs[5];
    gopedal = *uPtrs[6];
    UseGalvezSequence = *uPtrs[7];
       
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
        UxDes = axInit*tAccelerating + 0.21;  //0.5 is only for simulation;
        traction_command = ZERO_TORQUE_OFFSET + KUxOL_V*(Ux - UxDes);
        
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
        traction_command = ZERO_TORQUE_OFFSET + KUxOL_V*(Ux - UxDes);
              
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
        traction_command = ZERO_TORQUE_OFFSET + KUxOL_V*(Ux - UxDes);
        
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

        if (Enable == 0) {
            NextState = USER_CONTROL;
        }
        else if (beta <= betaThreshold) { 
            NextState = DRIFT;
            tDrift = tCurrent;
        }
        else {
            NextState = DESTABILIZE;
        } 

    }
    else if (CurrentState == DRIFT) {
        //Compute state errors
        eUy = Uy - UyEq;
        er = r - rEq;
        eUx = Ux - UxEq;
        
        //Compute value of s
        s = Kr*er + KUy*eUy;

        //Compute steering and Fxr input deltas
        Ddelta = k11*eUy + k12*er + k13*eUx;
        DFxr = k21*eUy + k22*er + k23*eUx;
/*
        printf("k11 = %f\r\n", k11);
        printf("k12 = %f\r\n", k12);
        printf("k13 = %f\r\n", k13);
        printf("k21 = %f\r\n", k21);
        printf("k22 = %f\r\n", k22);
        printf("k23 = %f\r\n", k23);
*/
       
       //Compute total steering and rear longitudinal force commands
       delta_command = deltaEq + Ddelta;
       Fxr = FxrEq + DFxr;
       
       //Apply saturation limits to rear longitudinal force
       if (Fxr < 0) {
           Fxr = 0;
       }
       else if (Fxr > mu*Fzr) {
           Fxr = mu*Fzr;
       }
       
       //Map to traction command voltage;
       traction_command = ZERO_TORQUE_OFFSET + Fxr*0.5*r_w*TorqueToVolts/gr;
            
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
    if (CurrentState == DRIFT) {
        y[3] = Fxr;
        y[4] = Ddelta;
        y[5] = DFxr;
        y[6] = deltaEq;
        y[7] = FxrEq;
        y[8] = s;
    }
    else {
        y[3] = y[4] = y[5] = y[6] = y[7] = y[8] = 0;
    }
}



static void mdlTerminate(SimStruct *S)
{

}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
