/*driftkeeping.c
 *Rami Y. Hindiyeh
 *DDL
 *
 *Executes steering and throttle commands for open loop initiation and closed loop control of a drift
*/
#define S_FUNCTION_NAME driftkeeping
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>
#include <stdio.h>

#define NUMOFPARAMS (27)
#define NUMOFINPUTS (22)
#define NUMOFOUTPUTS (18)

//Segment Type
#define STRAIGHT (0)
#define CLOTHOID_IN (1)
#define CONST_RADIUS (2)
#define CLOTHOID_OUT (3)

//Controller state
#define USER_CONTROL (1)
#define AUTONOMOUS (2)

//Minimum speed for slip angle computation
#define MIN_SPEED (0.5)

//Voltage Characteristics of Traction Command
#define ZERO_TORQUE_OFFSET (1.7)
#define REGEN_VOLTAGE (0.9)
#define MAX_VOLTAGE (2.8)

//Distance step for entry speed and brake point backsolve
#define DELTA_S (0.05)
//Maximum countersteer
#define MAX_COUNTERSTEER (-0.1745)
//Braking deceleration in straights
#define STRAIGHT_DECEL (-1.5)

//Important constants
#define PI (3.1415926535897932384626433832795)
#define GRAVACC (9.81)

#define TRU (1)
#define FALS (0)

//Module level varaibles
static real_T r_desTilde0;
static real_T axStraight;
static real_T NextState;
static real_T tStart;
static real_T UxInit;
static real_T UxEntry;
static real_T UxEntryIdeal;
static real_T sDotArc;
static real_T kappaArc;
static real_T EntryClothoidBrakingPoint;
static real_T PrevSegType;
static real_T PrevSegProg;
static real_T EntryClothoidBrakingActive;
static real_T StraightBrakingActive;
static real_T ExitClothoidFxr0;
static real_T FxrRampRate;
static real_T LapCount;
static real_T Fxr_prevTimestep;

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
    const real_T Ts = mxGetPr(ssGetSFcnParam(S,26))[0];
    
    ssSetSampleTime(S, 0, Ts);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
static void mdlInitializeConditions(SimStruct *S)
{    
    NextState = USER_CONTROL;
    r_desTilde0 = 0;
    UxInit = 0;
    PrevSegType = CLOTHOID_OUT;
    LapCount = 0;
    Fxr_prevTimestep = 0;
    UxEntry = 0;
    EntryClothoidBrakingPoint = 0;
}

#undef MDL_DERIVATIVES


static void mdlOutputs(SimStruct *S, int_T tid)
{
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    //Longitudinal Control Parameters
    const real_T KUx = mxGetPr(ssGetSFcnParam(S,0))[0];
    const real_T UxEntryScaleFactor = mxGetPr(ssGetSFcnParam(S,1))[0];
    const real_T UxConst = mxGetPr(ssGetSFcnParam(S,2))[0];
    const real_T UxExit = mxGetPr(ssGetSFcnParam(S,3))[0];
    //Controller Parameters 
    const real_T xla = mxGetPr(ssGetSFcnParam(S,4))[0];
    const real_T Ke = mxGetPr(ssGetSFcnParam(S,5))[0];
    const real_T Kr = mxGetPr(ssGetSFcnParam(S,6))[0];
    const real_T tau = mxGetPr(ssGetSFcnParam(S,7))[0];
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
    const real_T TorqueToVoltsRegen = 1/VoltsToTorqueRegen;
    //Regen force data
    const real_T Fxr_regen = 2*(REGEN_VOLTAGE - ZERO_TORQUE_OFFSET)*gr*VoltsToTorqueRegen/r_w;
    const real_T MaxRegenDecel = 0.8*Fxr_regen/M;
    const real_T MaxRegenDecelIdeal = Fxr_regen/M;
    //Assumed friction params
    const real_T mu_p_assumed_f = mxGetPr(ssGetSFcnParam(S,21))[0];
    const real_T mu_s_assumed_f = mxGetPr(ssGetSFcnParam(S,22))[0];
    const real_T mu_p_assumed_r = mxGetPr(ssGetSFcnParam(S, 23))[0];
    const real_T mu_s_assumed_r = mxGetPr(ssGetSFcnParam(S,24))[0];
    const real_T ExitClothoidFxrF = mu_s_assumed_r*Fzr;
    const real_T Fyf_sat = mxGetPr(ssGetSFcnParam(S,25))[0];
    //Sampling time
    const real_T Ts = mxGetPr(ssGetSFcnParam(S,26))[0];
    //Get current time
    real_T tCurrent = ssGetT(S);

    //Define input quantities
    real_T Enable, Ux, Uy, beta, r, delta_driver, gopedal, delta_meas;
    real_T delta_meas_l, delta_meas_r,UseTwoWheel,e,deltaPsi,kappa; 
    real_T SegType, SegLength, SegProg, Curv0, CurvF, NextSegLength, NextCurv0, NextCurvF;
    
    //Define controller quantities
    real_T UxDes, r_des, r_desTilde_p, r_desTilde_h, r_desTilde, r_desTildeDot, Fyf, sdot, psi_rDot;
    real_T Fyfl, Fyfr, Fyr, Fxr, alpha_f, alpha_fl, alpha_fr, alpha_r, deltaFyf, Fyr_des, Fyr_prev, Fxr_prev;
    real_T alpha_f_MaxCountersteer, Fyf_min;
    //Define output quantities
    real_T delta_command, Fyf_command, traction_command, traction_addition, CurrentState; 
    real_T deltaFyr_command, deltaFyr_est, deltaFxr_command, ControllerStatus;
    //Define quantities for backsolving curve entry speed
    real_T AtBrakingPoint, AtClothoidEntry, AtCurrentLocation, SegPos, PrevSegPos, SegPos_sDoubleDot, SegPos_eDoubleDot;
    real_T PrevSegPos_sDoubleDot, SegPos_sDot, PrevSegPos_sDot, ClothoidRate, delta_s;
    //Define quantities for determing straight braking point
    real_T sBraking;
    //Define quantities for determing when to apply regen on corner entry
    real_T eta_des, foo;

    
    //Set current state to next state
    CurrentState = NextState;
        
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
    r_desTilde_p = *uPtrs[9];
    SegType = *uPtrs[10];
    SegLength = *uPtrs[11];
    SegProg = *uPtrs[12];
    Curv0 = *uPtrs[13];
    CurvF = *uPtrs[14];
    NextSegLength = *uPtrs[15];
    NextCurv0 = *uPtrs[16];
    NextCurvF = *uPtrs[17];
    e = *uPtrs[18];
    deltaPsi = *uPtrs[19];
    kappa = *uPtrs[20];
    UseTwoWheel = *uPtrs[21];
    
    if (CurrentState == USER_CONTROL){
        //Set steering force command to manual
        Fyf_command = 0;
        //Set steering and traction commands to manual
        delta_command = delta_driver;
        traction_command = gopedal;
        ControllerStatus = 0;
        if (Enable == 1) {
            NextState = AUTONOMOUS;
            tStart = tCurrent;
        }
        else {
            NextState = USER_CONTROL;
        }
    }
    else if (CurrentState == AUTONOMOUS) {
        //Calculate tire slip angles (used throughout)
        if (Ux >= MIN_SPEED) {
            if (UseTwoWheel == 1) {
                delta_meas = 0.5*(delta_meas_l + delta_meas_r);
                alpha_f = atan2(Uy + a*r, Ux)-delta_meas;
                alpha_f_MaxCountersteer = atan2(Uy + a*r, Ux)-MAX_COUNTERSTEER;
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
        //Set steering command to zero, as we are making force commands
        delta_command = 0;
        if (Enable == 0) {
            SegType = PrevSegType;
        }
        //Set controller behavior for the straights
        if (SegType == STRAIGHT) {
            if (PrevSegType == CLOTHOID_OUT) {
                LapCount = LapCount + 1;
                StraightBrakingActive = FALS;
                //AtBrakingPoint = FALS;
                AtClothoidEntry = FALS;
                ClothoidRate = (NextCurvF - NextCurv0)/NextSegLength;
                //Initialize segment position to end of clothoid
                PrevSegPos = NextSegLength;
                //Initialize velocity at previous segment position to Ux at end of clothoid
                //(Velocity for constant arc)
                PrevSegPos_sDot = sqrt(0.5*(mu_s_assumed_f + mu_s_assumed_r)*GRAVACC/NextCurvF);
                sDotArc = PrevSegPos_sDot;
                kappaArc = NextCurvF;
                PrevSegPos_sDoubleDot = 0;
                //Backsolve from end of clothoid to get ax and Ux until we reach entry point of clothoid
                while (AtClothoidEntry == FALS){
                    if (PrevSegPos - DELTA_S > 0) {
                    SegPos = PrevSegPos - DELTA_S;
                    delta_s = DELTA_S;
                    }
                    else {
                        SegPos = 0;
                        delta_s = PrevSegPos;
                    }
                    SegPos_sDot = PrevSegPos_sDot + delta_s*fabs(PrevSegPos_sDoubleDot)/PrevSegPos_sDot;
                    if (PrevSegPos_sDoubleDot > MaxRegenDecel) {
                        SegPos_sDoubleDot = -1*sqrt(pow(0.5*(mu_s_assumed_f+mu_s_assumed_r)*GRAVACC, 2)-pow(ClothoidRate*SegPos*pow(SegPos_sDot,2),2));
                    }
                    else if (PrevSegPos_sDoubleDot == MaxRegenDecel) {
                        SegPos_sDoubleDot = MaxRegenDecel;
                    }
                    
                    if (SegPos_sDoubleDot < MaxRegenDecel) {
                        SegPos_sDoubleDot = MaxRegenDecel;
                    }
          
                    if (SegPos > 0) {
                        PrevSegPos = SegPos;
                        PrevSegPos_sDot = SegPos_sDot;
                        PrevSegPos_sDoubleDot = SegPos_sDoubleDot;
                        
                    }
                    else if (SegPos == 0) {
                        AtClothoidEntry = TRU;
                        UxEntryIdeal = PrevSegPos_sDot;
                        UxEntry = UxEntryScaleFactor*UxEntryIdeal;
                        EntryClothoidBrakingPoint = 0;
                    }
                }
                if (LapCount >= 1) {
                    UxInit = Ux;
                    axStraight = (1/(2*((0.9*SegLength)-SegProg)))*(pow(UxEntry-UxInit, 2)+2*UxInit*(UxEntry-UxInit));
                    tStart = tCurrent;
                }
            }
            //Compute longitudinal command
            if (LapCount >= 1) {
                if (SegProg <= 0.9*SegLength){
                    UxDes = UxInit + axStraight*(tCurrent - tStart);
                    Fxr = M*axStraight - M*KUx*(Ux - UxDes);
                }
                else {
                    Fxr = Fxr_regen;
                }
            }
            else {
                if (StraightBrakingActive == FALS){
                    sBraking = SegProg + Ux*(UxEntry - Ux)/STRAIGHT_DECEL + 0.5*STRAIGHT_DECEL*pow((UxEntry-Ux)/STRAIGHT_DECEL, 2);
                    if (sBraking >= SegLength) {
                        StraightBrakingActive = TRU;
                        UxInit = Ux;
                        tStart = tCurrent;
                        UxDes = UxInit + STRAIGHT_DECEL*(tCurrent - tStart);
                        Fxr = M*STRAIGHT_DECEL - M*KUx*(Ux - UxDes);
                    }
                    else {
                        Fxr = ExitClothoidFxrF;    
                    }
                }
                else if (StraightBrakingActive == TRU) {
                    UxDes = UxInit + STRAIGHT_DECEL*(tCurrent - tStart);
                    Fxr = M*STRAIGHT_DECEL - M*KUx*(Ux - UxDes);
                }
                
            }
        }
        //Set controller behavior for entry clothoids
        else if (SegType == CLOTHOID_IN){
            if (PrevSegType == STRAIGHT) {
                EntryClothoidBrakingActive = FALS;
            }
            //Apply full regen after passing braking point
            if (SegProg < EntryClothoidBrakingPoint) {
                UxDes = UxEntry;
                Fxr = -M*KUx*(Ux-UxDes);
            }
            else if (SegProg >= EntryClothoidBrakingPoint) {
                if (PrevSegProg < EntryClothoidBrakingPoint) {
                    EntryClothoidBrakingActive = TRU;
                }
                //Fxr = Fxr_regen;
                AtCurrentLocation = FALS;
                ClothoidRate = (CurvF - Curv0)/SegLength;
                //Initialize segment position to end of clothoid
                PrevSegPos = SegLength;
                //Initialize velocity at previous segment position to Ux at end of clothoid
                //(Velocity for constant arc)
                PrevSegPos_sDot= sqrt(0.5*(mu_s_assumed_f + mu_s_assumed_r)*GRAVACC/CurvF);
                PrevSegPos_sDoubleDot = 0;
                while (AtCurrentLocation == FALS) {
                //Backsolve from end of clothoid to get ax, ay at entry point of clothoid
                    if (PrevSegPos - DELTA_S > SegProg) {
                    SegPos = PrevSegPos - DELTA_S;
                    delta_s = DELTA_S;
                    }
                    else {
                        SegPos = SegProg;
                        delta_s = PrevSegPos-SegProg;
                    }
                    SegPos_sDot = PrevSegPos_sDot + delta_s*fabs(PrevSegPos_sDoubleDot)/PrevSegPos_sDot;
                    if (PrevSegPos_sDoubleDot > MaxRegenDecelIdeal) {
                        SegPos_sDoubleDot = -1*sqrt(pow(0.5*(mu_s_assumed_f+mu_s_assumed_r)*GRAVACC, 2)-pow(ClothoidRate*SegPos*pow(SegPos_sDot,2),2));
                    }
                    else if (PrevSegPos_sDoubleDot == MaxRegenDecelIdeal) {
                        SegPos_sDoubleDot = MaxRegenDecelIdeal;
                    }
                    
                    if (SegPos_sDoubleDot < MaxRegenDecelIdeal) {
                        SegPos_sDoubleDot = MaxRegenDecelIdeal;
                    }  
                    SegPos_eDoubleDot = ClothoidRate*SegPos*pow(SegPos_sDot,2);
        
                    if (SegPos > SegProg) {
                        PrevSegPos = SegPos;
                        PrevSegPos_sDot = SegPos_sDot;
                        PrevSegPos_sDoubleDot = SegPos_sDoubleDot;
                        
                    }
                    else if (SegPos == SegProg) {
                        AtCurrentLocation = TRU;
                    }     
                }
                Fxr = M*(SegPos_sDoubleDot*cos(deltaPsi) + SegPos_eDoubleDot*sin(deltaPsi));
            }
        }
        //Set controller behavior for constant radius sections
        else if (SegType == CONST_RADIUS){
            if (PrevSegType == CLOTHOID_IN) {
                EntryClothoidBrakingActive = FALS;
            }
            SegPos_sDot = sDotArc;
            SegPos_eDoubleDot = pow(SegPos_sDot, 2)*kappaArc;
            SegPos_sDoubleDot = 0;
            Fxr = M*(SegPos_sDoubleDot*cos(deltaPsi) + SegPos_eDoubleDot*sin(deltaPsi));
            //Fxr = 0;
        }
            
        //Set controller behavior for exit clothoid
        else if (SegType == CLOTHOID_OUT) {
/*
            if (PrevSegType == CONST_RADIUS) {
                SegPos_sDot = sDotArc;
                SegPos_eDoubleDot = pow(SegPos_sDot, 2)*kappaArc;
                SegPos_sDoubleDot = 0;
                ExitClothoidFxr0 = Fxr_prevTimestep;
                FxrRampRate = (ExitClothoidFxrF - ExitClothoidFxr0)/SegLength;
            }
            Fxr = ExitClothoidFxr0 + FxrRampRate*SegProg;
*/
            Fxr = 0;
        }
        if (Fxr > mu_s_assumed_r*Fzr) {
            Fxr = mu_s_assumed_r*Fzr;
        }
        else if (Fxr < Fxr_regen) {
            Fxr = Fxr_regen;
        }
        ControllerStatus = 1;
        
        //Calculate rear tire lateral force
        Fyr = CalcNonLinearTireForce(Fzr, Fxr, mu_p_assumed_r, mu_s_assumed_r, Cr, alpha_r);
        deltaFyr_command = 0;
        deltaFyr_est = 0;
        deltaFxr_command = 0;
        
        //Check to see if we have full rear tire saturation when appropriate, and apply more Fxr as necessary
/*
        if ((SegType == CONST_RADIUS) | (SegType == CLOTHOID_OUT)){
            if (fabs(sqrt(pow(Fxr,2) + pow(Fyr,2)) - mu_s_assumed_r*Fzr) > 0.001) {
                Fxr_prev = Fxr;
                eta_des = tan(fabs(alpha_r))*Cr/(3*mu_p_assumed_r*Fzr);
                if (eta_des > 1) {
                    eta_des = 1;
                }
                else if (eta_des < 0) {
                    eta_des = 0;
                }
                if (Fxr_prev <= 0){
                    Fxr = -1*sqrt(pow(mu_s_assumed_r*Fzr, 2) -pow(eta_des*mu_s_assumed_r*Fzr, 2));
                }
                else if (Fxr_prev > 0){
                    Fxr = sqrt(pow(mu_s_assumed_r*Fzr, 2) -pow(eta_des*mu_s_assumed_r*Fzr, 2));
                }
                if (Fxr < Fxr_regen) {
                    Fxr = Fxr_regen;
                }
                deltaFxr_command = Fxr - Fxr_prev;
                //Calculate rear tire lateral force
                Fyr = CalcNonLinearTireForce(Fzr, Fxr, mu_p_assumed_r, mu_s_assumed_r, Cr, alpha_r);
                deltaFxr_command = Fxr - Fxr_prev;
                ControllerStatus = 2;
            }
        }
*/
        //Now compute controller command....
        
        //Calculate necessary path-related quantities
        sdot = Ux*cos(deltaPsi) - Uy*sin(deltaPsi);
        psi_rDot = sdot*kappa;
       
        //Calculate front tire lateral force for error checking
        if (UseTwoWheel == 1) {
            Fyf = CalcNonLinearTireForce(Fzf, 0, mu_p_assumed_f, mu_s_assumed_f, Cf, alpha_f);
        }
        else {
            Fyfl = CalcNonLinearTireForce(Fzf/2, 0, mu_p_assumed_f, mu_s_assumed_f, Cf/2, alpha_fl);
            Fyfr = CalcNonLinearTireForce(Fzf/2, 0, mu_p_assumed_f, mu_s_assumed_f, Cf/2, alpha_fr);
            Fyf = Fyfl + Fyfr;
        }
        //Compute yaw rate command and intermediate state
        r_des = psi_rDot - (Ke/xla)*e - Ke*deltaPsi - (1/xla)*Uy*cos(deltaPsi) - (1/xla)*Ux*sin(deltaPsi);
        r_desTilde_h = r_desTilde0*exp(-(1/tau)*(tCurrent-tStart));
        r_desTilde = r_desTilde_h + r_desTilde_p;
        r_desTildeDot = (1/tau)*(r_des - r_desTilde);
        //Assume we use steering alone initially
        Fyf_command = (Iz/a)*((b/Iz)*Fyr + (1/tau)*psi_rDot - (Ke/(tau*xla))*e - (Ke/tau)*deltaPsi - (1/(tau*xla))*Uy*cos(deltaPsi) - (1/(tau*xla))*Ux*sin(deltaPsi) + (Kr-1/tau)*r_desTilde - Kr*r);
        //Now check to see if command is within friction limits.  If not, determine new longitudinal command
        
        //Compute minium available front lateral force
        Fyf_min = CalcNonLinearTireForce(Fzf, 0, mu_p_assumed_f, mu_s_assumed_f, Cf, alpha_f_MaxCountersteer);
/*
        if ((Ux >= 4) & (Fyf_command > Fyf_sat) & (SegType != STRAIGHT) & (SegType != CLOTHOID_IN)) {
            //Set Fyr_prev and Fxr_prev to initial Fyr and Fxr, respectively
            Fyr_prev = Fyr;
            Fxr_prev = Fxr;
            
            //Calculate desired rear lateral force and change relative to original value
            Fyr_des = (Iz/b)*((a/Iz)*Fyf_sat - (1/tau)*psi_rDot + (Ke/(tau*xla))*e + (Ke/tau)*deltaPsi + (1/(tau*xla))*Uy*cos(deltaPsi) + (1/(tau*xla))*Ux*sin(deltaPsi) - (Kr-1/tau)*r_desTilde + Kr*r);
            
            if (Fyr_des < 0) {
                Fyr_des = 0;
            }
            else if (fabs(Fyr_des) > mu_s_assumed_r*Fzr) {
                if (Fyr_des > 0) {
                    Fyr_des = mu_s_assumed_r*Fzr;
                }
                else if (Fyr_des < 0) {
                    Fyr_des = -mu_s_assumed_r*Fzr;
                }
            }
            
            deltaFyr_command = Fyr_des - Fyr_prev;
            
            //Calculate longitudinal force required to achieved change in lateral force and associated change in longitudinal force
            if (Fxr_prev <= 0){
                Fxr = -1*sqrt(pow(mu_s_assumed_r, 2)*pow(Fzr, 2) - pow(Fyr_des, 2));
            }
            else if (Fxr_prev> 0){
                Fxr = sqrt(pow(mu_s_assumed_r, 2)*pow(Fzr, 2) - pow(Fyr_des, 2));
            }
            deltaFxr_command = Fxr - Fxr_prev;  
            
            if (Fxr < Fxr_regen) {
                Fxr = Fxr_regen;
            }

            //Now compute rear lateral force
            Fyr = CalcNonLinearTireForce(Fzr, Fxr, mu_p_assumed_r, mu_s_assumed_r, Cr, alpha_r);
            //Compute estimates of change in Fyr for debugging
            deltaFyr_est = Fyr - Fyr_prev;
            //Set front lateral force and traction commands based on above computations
            Fyf_command = (Iz/a)*((b/Iz)*Fyr + (1/tau)*psi_rDot - (Ke/(tau*xla))*e - (Ke/tau)*deltaPsi - (1/(tau*xla))*Uy*cos(deltaPsi) - (1/(tau*xla))*Ux*sin(deltaPsi) + (Kr-1/tau)*r_desTilde - Kr*r);
            ControllerStatus = 3;
        }
 */
        if ((Fyf_command < Fyf_min) & (SegType != STRAIGHT)) {
            //Set Fyr_prev and Fxr_prev to initial Fyr and Fxr, respectively
            Fyr_prev = Fyr;
            Fxr_prev = Fxr;
            
            //Calculate desired rear lateral force and change relative to original value
            Fyr_des = (Iz/b)*((a/Iz)*Fyf_min - (1/tau)*psi_rDot + (Ke/(tau*xla))*e + (Ke/tau)*deltaPsi + (1/(tau*xla))*Uy*cos(deltaPsi) + (1/(tau*xla))*Ux*sin(deltaPsi) - (Kr-1/tau)*r_desTilde + Kr*r);
            
            if (Fyr_des < 0) {
                Fyr_des = 0;
            }
            else if (fabs(Fyr_des) > mu_s_assumed_r*Fzr) {
                if (Fyr_des > 0) {
                    Fyr_des = mu_s_assumed_r*Fzr;
                }
                else if (Fyr_des < 0) {
                    Fyr_des = -mu_s_assumed_r*Fzr;
                }
            }
            
            
            deltaFyr_command = Fyr_des - Fyr_prev;
            
            //Calculate longitudinal force required to achieved change in lateral force and associated change in longitudinal force
            if (fabs(Fyr_des) < mu_s_assumed_r*Fzr){
                if (Fxr_prev < 0){
                    Fxr = -1*sqrt(pow(mu_s_assumed_r, 2)*pow(Fzr, 2) - pow(Fyr_des, 2));
                }
                else if (Fxr_prev > 0){
                    Fxr = sqrt(pow(mu_s_assumed_r, 2)*pow(Fzr, 2) - pow(Fyr_des, 2));
                }
                else if (Fxr_prev == 0) {
                    Fxr = 0;
                }
            }
            else {
                Fxr = 0;
            }
            
            deltaFxr_command = Fxr - Fxr_prev;
            
            if (Fxr < Fxr_regen) {
                Fxr = Fxr_regen;
            }
            
            //Now compute rear lateral force
            Fyr = CalcNonLinearTireForce(Fzr, Fxr, mu_p_assumed_r, mu_s_assumed_r, Cr, alpha_r);
            //Compute estimates of change in Fyr for debugging
            deltaFyr_est = Fyr - Fyr_prev;
            //Set front lateral force and traction commands based on above computations
            Fyf_command = (Iz/a)*((b/Iz)*Fyr + (1/tau)*psi_rDot - (Ke/(tau*xla))*e - (Ke/tau)*deltaPsi - (1/(tau*xla))*Uy*cos(deltaPsi) - (1/(tau*xla))*Ux*sin(deltaPsi) + (Kr-1/tau)*r_desTilde - Kr*r);
            ControllerStatus = 4;
        }

        if (Fxr >= 0) {
            traction_command = ZERO_TORQUE_OFFSET + Fxr*0.5*r_w*TorqueToVolts/gr;
        }
        else {
            traction_command = ZERO_TORQUE_OFFSET + Fxr*0.5*r_w*TorqueToVoltsRegen/gr;
        }
        Fxr_prevTimestep = Fxr;
        PrevSegType = SegType;
        
        if (Enable == 0) {
            NextState = USER_CONTROL;

        }
        else {
            NextState = AUTONOMOUS;
        }
    }
    
    y[0] = delta_command;
    y[1] = traction_command;
    y[2] = CurrentState;

    
    if (CurrentState == AUTONOMOUS) {
        y[3] = Fyf_command;
        y[4] = r_des;
        y[5] = r_desTildeDot;
        y[6] = r_desTilde;
        y[7] = Fyf;
        y[8] = Fyr;
        y[9] = Fxr;
        y[10] = deltaFyr_command;
        y[11] = deltaFyr_est;
        y[12] = deltaFxr_command;
        y[13] = sdot;
        y[14] = psi_rDot;
        y[15] = ControllerStatus;
        y[16] = EntryClothoidBrakingPoint;
        y[17] = UxEntry;
    }
    else {
        y[3] = y[4] = y[5] = y[6] = y[7] = y[8] = y[9] = y[10] = y[11] = y[12] = y[13] = y[14] = y[15] = y[16] = y[17] = 0;
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
