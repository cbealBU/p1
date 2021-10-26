/*drift_controller_SDSC.c
 *Rami Y. Hindiyeh
 *DDL
 *
 *Executes steering and throttle commands for open loop initiation and closed loop control of a drift with a simplified dynamic
 *surface control scheme
*/
#define S_FUNCTION_NAME  drift_controller_SDSC
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>
#include <stdio.h>

#define NUMOFPARAMS (33)
#define NUMOFINPUTS (13)
#define NUMOFOUTPUTS (22)

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

static real_T CalcNonLinearTireForce(real_T Fz, real_T Fx, real_T mu_p, real_T mu_s, real_T Calpha, real_T alpha){
    real_T alpha_sl;
    real_T eta; 
    real_T tan_alpha;
    real_T Fy;
    

    eta = sqrt(fabs(pow(mu_s,2)*pow(Fz,2) - pow(Fx,2)))/(mu_s*Fz);

    alpha_sl = atan2(3*eta*mu_p*Fz, Calpha);
        
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
    const real_T Ts = mxGetPr(ssGetSFcnParam(S,32))[0];
    
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
    //Drift initiation params
    const real_T betaThreshold = mxGetPr(ssGetSFcnParam(S,5))[0]; 
    const real_T rThreshold = mxGetPr(ssGetSFcnParam(S,6))[0]; 
    //Steering and longitudinal control params
    const real_T betaDes = mxGetPr(ssGetSFcnParam(S,7))[0]; 
    const real_T UxEq = mxGetPr(ssGetSFcnParam(S,8))[0]; 
    const real_T FxrEq = mxGetPr(ssGetSFcnParam(S,9))[0]; 
    const real_T FyfEq = mxGetPr(ssGetSFcnParam(S,10))[0];
    const real_T FyrEq = mxGetPr(ssGetSFcnParam(S,11))[0];
    const real_T Kbeta = mxGetPr(ssGetSFcnParam(S,12))[0]; 
    const real_T Kr = mxGetPr(ssGetSFcnParam(S,13))[0];
    const real_T KUx_N = mxGetPr(ssGetSFcnParam(S,14))[0];
    const real_T KUx_N_Drift = mxGetPr(ssGetSFcnParam(S,15))[0];
    //Vehicle params
    const real_T VoltsToTorque = mxGetPr(ssGetSFcnParam(S,16))[0];
    const real_T VoltsToTorqueRegen = mxGetPr(ssGetSFcnParam(S, 17))[0];
    const real_T gr = mxGetPr(ssGetSFcnParam(S,18))[0];
    const real_T r_w = mxGetPr(ssGetSFcnParam(S,19))[0];
    const real_T M = mxGetPr(ssGetSFcnParam(S, 20))[0];
    const real_T Iz = mxGetPr(ssGetSFcnParam(S, 21))[0];
    const real_T a = mxGetPr(ssGetSFcnParam(S, 22))[0];
    const real_T b  = mxGetPr(ssGetSFcnParam(S, 23))[0];
    const real_T c = mxGetPr(ssGetSFcnParam(S, 24))[0];
    const real_T L = a+b;
    const real_T Clf = mxGetPr(ssGetSFcnParam(S, 25))[0];
    const real_T Crf = mxGetPr(ssGetSFcnParam(S, 26))[0];
    const real_T Clr = mxGetPr(ssGetSFcnParam(S, 27))[0];
    const real_T Crr = mxGetPr(ssGetSFcnParam(S, 28))[0];
    const real_T Cf = Clf + Crf;
    const real_T Cr = Clr + Crr;
    const real_T Fzf = (b/L)*M*GRAVACC; //Nominal front axle normal load
    const real_T Fzr = (a/L)*M*GRAVACC; //Nominal rear axle normal load
    const real_T TorqueToVolts = 1/VoltsToTorque;
    const real_T TorqueToVoltsRegen = 1/VoltsToTorqueRegen;
    const real_T KUx_V = 0.5*KUx_N*r_w*(1/gr)*TorqueToVolts;
    const real_T KUx_V_Drift = 0.5*KUx_N_Drift*r_w*(1/gr)*TorqueToVolts;
    const real_T VoltageEq = 0.5*FxrEq*r_w*(1/gr)*TorqueToVolts;
    //Assumed friction params
    const real_T mu_p_assumed = mxGetPr(ssGetSFcnParam(S,29))[0];
    const real_T mu_s_assumed = mxGetPr(ssGetSFcnParam(S,30))[0];
    const real_T Fyf_sat = mxGetPr(ssGetSFcnParam(S,31))[0];
    //Sampling time
    const real_T Ts = mxGetPr(ssGetSFcnParam(S,32))[0];
    //Get current time
    real_T tCurrent = ssGetT(S);

    //Define input quantities
    real_T Enable, Ux, Uy, beta, r, delta_driver, gopedal, delta_meas;
    real_T delta_meas_l, delta_meas_r, UseTwoWheel, UseLongitudinalCoord, UseGalvezSequence, UseDiffDrive; 
    
    //Define controller quantities
    real_T r_des, r_desDot, Fyf, Fyfl, Fyfr, Fyr, Fxr, alpha_f, alpha_fl, alpha_fr, alpha_r, deltaFyf, Fyr_des, Fyr_prev, Fxr_prev;
    //Define output quantities
    real_T delta_command, Fyf_command, traction_command, traction_addition, traction_commandInit, traction_Feedforward; 
    real_T traction_command_l, traction_command_r, Fxr_l, Fxr_r, M_diff; //Quantities related to differential drive control.
    real_T traction_ForceError, traction_SpeedError, deltaFyr_command, deltaFyr_est, deltaFxr_des, deltaFxr_command, deltaFxr_est;
    
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
    UseTwoWheel = *uPtrs[9];
    UseLongitudinalCoord = *uPtrs[10];
    UseGalvezSequence = *uPtrs[11];
    UseDiffDrive = *uPtrs[12];
       
    //State machine goes here
    CurrentState = NextState;
    
    if (CurrentState == USER_CONTROL) {
        delta_command = delta_driver;
        traction_command = gopedal;
        traction_command_l = traction_command;
        traction_command_r = traction_command;
        traction_Feedforward = 0;
        traction_ForceError = 0;
        traction_SpeedError = 0;
        
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
        traction_command = ZERO_TORQUE_OFFSET + KUx_V*(Ux - UxDes);
        traction_command_l = traction_command;
        traction_command_r = traction_command;
        traction_Feedforward = 0;
        traction_ForceError = 0;
        traction_SpeedError = KUx_V*(Ux - UxDes);
        
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
        traction_command_l = traction_command;
        traction_command_r = traction_command;
        traction_Feedforward = 0;
        traction_ForceError = 0;
        traction_SpeedError = KUx_V*(Ux - UxDes);
        
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
        traction_command_l = traction_command;
        traction_command_r = traction_command;
        traction_Feedforward = 0;
        traction_ForceError = 0;
        traction_SpeedError = KUx_V*(Ux - UxDes);
        
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
        traction_command_l = REGEN_VOLTAGE;
        traction_command_r = REGEN_VOLTAGE;
        traction_Feedforward = REGEN_VOLTAGE;
        traction_ForceError = 0;
        traction_SpeedError = 0;

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
//         printf("In drift state, t = %f\r\n", tCurrent);
        //Determine longitudinal control input (feed-forward + speed control)
        UxDes = UxEq;
        traction_command = ZERO_TORQUE_OFFSET + VoltageEq + KUx_V_Drift*(Ux - UxDes);
        traction_command_l = traction_command;
        traction_command_r = traction_command;
        traction_Feedforward = VoltageEq;
        traction_ForceError = 0;
        traction_SpeedError = KUx_V_Drift*(Ux - UxDes);
        deltaFyr_command = 0;
        deltaFyr_est = 0;
        deltaFxr_des = 0;
        deltaFxr_command = 0;
        deltaFxr_est = 0;
        M_diff = 0;
               
        //Set delta_command to zero since we are computing Fyf in DSC
        delta_command = 0;
        //Compute front and rear tire slip angles
        if (Ux >= MIN_SPEED) {
            if (UseTwoWheel == 1) {
                delta_meas = 0.5*(delta_meas_l + delta_meas_r);
                alpha_f = atan2(Uy + a*r, Ux)-delta_meas;
            }
            else {
                alpha_fl = atan2(Uy + a*r, Ux - c*r) - delta_meas_l;
                alpha_fr = atan2(Uy + a*r, Ux + c*r) - delta_meas_r; 
            }
            alpha_r = atan2(Uy - b*r, Ux);   
        }
        else {
            alpha_f = 0;
            alpha_fl = 0;
            alpha_fr = 0;
            alpha_r = 0;
        }
        
        //Compute Fxr based upon current pedal position/traction command in volts
//         Fxr = (2*(traction_command - ZERO_TORQUE_OFFSET - KUx_V*(Ux - UxDes))*VoltsToTorque*gr)/r_w;
        if (traction_command >= ZERO_TORQUE_OFFSET) {
            Fxr = 2*(traction_command - ZERO_TORQUE_OFFSET)*VoltsToTorque*gr/r_w;
        }
        else {
            Fxr = 2*(traction_command - ZERO_TORQUE_OFFSET)*VoltsToTorqueRegen*gr/r_w;
        }
        
        //If result is more than available force, set to max available force
        if (fabs(Fxr) >= mu_s_assumed*Fzr) {
            if (Fxr > 0) {
                Fxr = mu_s_assumed*Fzr;
            }
            else {
                Fxr = -mu_s_assumed*Fzr;
            }
        }
        Fxr_l = Fxr/2;
        Fxr_r = Fxr/2;

        //Now compute rear lateral force
        Fyr = CalcNonLinearTireForce(Fzr, Fxr, mu_p_assumed, mu_s_assumed, Cr, alpha_r);
        
        //Compute front lateral force (just for debugging and comparison to Fyf command)
        if (UseTwoWheel == 1) {
            Fyf = CalcNonLinearTireForce(Fzf, 0, mu_p_assumed, mu_s_assumed, Cf, alpha_f);
        }
        else {
            Fyfl = CalcNonLinearTireForce(Fzf/2, 0, mu_p_assumed, mu_s_assumed, Cf/2, alpha_fl);
            Fyfr = CalcNonLinearTireForce(Fzf/2, 0, mu_p_assumed, mu_s_assumed, Cf/2, alpha_fr);
            Fyf = Fyfl + Fyfr;
        }
        
        //Compute r_des based on equilibrium forces and current beta error
        r_des = (FyfEq + FyrEq)/(M*UxEq) + Kbeta*(beta - betaDes);

        //Compute required front tire force for dynamic surface control
        Fyf_command = ((b/Iz + Kbeta/(M*Ux))*Fyr - (Kbeta+Kr)*r + Kr*((FyfEq + FyrEq)/(M*UxEq)) + Kr*Kbeta*(beta-betaDes))/(a/Iz - Kbeta/(M*Ux));

        //Based upon commanded Fyf, compute r_des and r_desDot for this time step
        r_desDot = Kbeta*((Fyf_command + Fyr)/(M*Ux) - r);
        
        //If we want to use coordinated longitudinal control
        if (UseLongitudinalCoord == 1) {
            //Determine if front tire force is above saturation threshold. If not, don't do anything
            //If yes....
            if (Fyf_command > Fyf_sat) {
                //Set initial Fyr_prev to initial Fyr
                Fyr_prev = Fyr;
                Fxr_prev = Fxr;
                // Set front force to saturation level
                Fyf_command = Fyf_sat;
                //Compute desired rear force level assuming above front force command
                Fyr_des = ((a/Iz - Kbeta/(M*Ux))*Fyf_command + (Kbeta+Kr)*r - Kr*((FyfEq + FyrEq)/(M*UxEq)) - Kr*Kbeta*(beta-betaDes))/(b/Iz + Kbeta/(M*Ux));
                if (Fyr_des*Fyr_prev < 0) {
                    Fyr_des = 0;
                }
                deltaFyr_command = Fyr_des - Fyr_prev;
                Fxr = sqrt(pow(mu_s_assumed, 2)*pow(Fzr, 2) - pow(Fyr_des, 2));
                Fxr_l = Fxr/2;
                Fxr_r = Fxr/2;
                deltaFxr_command = Fxr - Fxr_prev;
                //Compute exact deltaFxr we'd need from nonlinear relationship (should be the same as deltaFxr_command in this case)
                deltaFxr_des = sqrt(pow(mu_s_assumed, 2)*pow(Fzr, 2)-pow(Fyr_prev+deltaFyr_command, 2))-Fxr_prev;
                
                //Convert quantities to traction commands in volts
                traction_addition = deltaFxr_command*0.5*r_w*TorqueToVolts/gr;
                traction_command = ZERO_TORQUE_OFFSET + Fxr*0.5*r_w*TorqueToVolts/gr;
                traction_command_l = traction_command;
                traction_command_r = traction_command;
                traction_ForceError = traction_addition;
                
                //Now compute rear lateral force
                Fyr = CalcNonLinearTireForce(Fzr, Fxr, mu_p_assumed, mu_s_assumed, Cr, alpha_r);
                //Compute estimates of change in Fyr and Fxr for output
                deltaFyr_est = Fyr - Fyr_prev;
                deltaFxr_est = Fxr - Fxr_prev;
                
                //Re-compute r_desDot based on new solution
                r_desDot = Kbeta*((Fyf_command + Fyr)/(M*Ux) - r);
            }
        }
        
        else if (UseDiffDrive == 1) {
            if (Fyf_command > Fyf_sat) {
                //Set steering to saturation level
                Fyf_command = Fyf_sat;
                //Compute required moment input at the rear axle
                M_diff = Iz*(-1*(a/Iz - Kbeta/(M*Ux))*Fyf_command + (b/Iz + Kbeta/(M*Ux))*Fyr - (Kbeta+Kr)*r + Kr*((FyfEq + FyrEq)/(M*UxEq)) + Kr*Kbeta*(beta-betaDes));
                //Now determine the left and right rear longitudinal commands
                Fxr_r = Fxr/2 + (1/(2*c))*M_diff;
                Fxr_l = Fxr/2 - (1/(2*c))*M_diff;
                //Now make sure that longitudinal force commands are within friction limits for each tire
                if (fabs(Fxr_r) > 0.5*mu_s_assumed*Fzr) {
                    if (Fxr_r < 0) {
                        Fxr_r = -0.5*mu_s_assumed*Fzr;
                    }
                    else if (Fxr_r > 0) {
                        Fxr_r = 0.5*mu_s_assumed*Fzr;
                    }
                }
                if (fabs(Fxr_l) > 0.5*mu_s_assumed*Fzr) {
                    if (Fxr_l < 0) {
                        Fxr_l = -0.5*mu_s_assumed*Fzr;
                    }
                    else if (Fxr_l > 0) {
                        Fxr_l = 0.5*mu_s_assumed*Fzr;
                    }
                }
                
                //Compute left and right traction commands in volts
                if (Fxr_r >= 0){
                    traction_command_r = ZERO_TORQUE_OFFSET + Fxr_r*r_w*TorqueToVolts/gr;
                }
                else {
                    traction_command_r = ZERO_TORQUE_OFFSET + Fxr_r*r_w*TorqueToVoltsRegen/gr;
                }
                if (Fxr_l >= 0){
                    traction_command_l = ZERO_TORQUE_OFFSET + Fxr_l*r_w*TorqueToVolts/gr;
                }
                else{
                    traction_command_l = ZERO_TORQUE_OFFSET + Fxr_l*r_w*TorqueToVoltsRegen/gr;
                }
            }
            
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
    y[2] = traction_command_l;
    y[3] = traction_command_r;
    y[4] = CurrentState;
    y[14] = traction_Feedforward;
    y[15] = traction_SpeedError;
    y[16] = traction_ForceError;
    if (CurrentState == DRIFT) {
        y[5] = Fyf_command;
        y[6] = r_des;
        y[7] = r_desDot;
        y[8] = Fyf;
        y[9] = Fyr;
        y[10] = Fxr;
        y[11] = Fxr_l;
        y[12] = Fxr_r;
        y[13] = M_diff;
        y[17] = deltaFyr_command;
        y[18] = deltaFyr_est;
        y[19] = deltaFxr_des;
        y[20] = deltaFxr_command;
        y[21] = deltaFxr_est;
    }
    else {
        y[5] = y[6] = y[7] = y[8] = y[9] = y[10] = y[11] = y[12] = y[13]= y[17] = y[18] = y[19] = y[20] = y[21] = 0;
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
