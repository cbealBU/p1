/* 
 * File: ssc_safe_area
 *
 * Author: Carrie Bobier
 * (C)2009 Dynamic Design Laboratory -- Stanford University
 *
 * $Revision: 1$ $Date: January 29, 2010 $
 *
 * This file implements an s-function to calculate the safe area and
 * corresponding safe states
 * 
 *	The model inputs are
 *		*uPtrs[0] = yaw rate in RADIANS/S
 *		*uPtrs[1] = sideslip in RADIANS
 *      *uPtrs[2] = steering angle in RADIANS
 *      *uPtrs[3] = speed in METER/S
 *      *uPtrs[4] = steering angle derivative in RADIANS/S
 *      *uPtrs[5] = accelerator pedal input in VOLTS
 *      *uPtrs[6] = longitudinal speed derivative in METER/S/S
 * The model outputs are
 *		y[0] = safe yaw rate (rad/s)
 *		y[1] = safe sideslip (rad)
 *      y[2] = safe yaw rate dot (rad/s/s)
 *      y[3] = safe sideslip dot (rad/s)
 *      y[4] = enable SSC
 *      y[5] = Fyr
 *      y[6] = Fyf_control
 *      y[7] = manifold_S
 */

#define S_FUNCTION_NAME  ssc_safe_area
#define S_FUNCTION_LEVEL 2

#include <math.h>
#include "simstruc.h"                           /* need this for SimStruct later */

/* parameter list:
 * a, b, CalphaF, CalphaR, Fzf, Fzr, mu, mu_s, m, Iz, bsafe_const, msafe_const, bsafe_slope,
 * q, Kcontrol
 */

#define NUMOFINPUTS  7 	 
#define NUMOFOUTPUTS 8	 
#define NUMOFSTATES	 0
#define NUMOFPARAMS  15

#define g         9.81

static int INPUTWIDTHS[]={1,1,1,1,1,1,1};       
static int OUTPUTWIDTHS[]={1,1,1,1,1,1,1,1};

// Helper Functions ========================================================================== *

static real_T getFialaTireForce(real_T Calpha, real_T alpha, real_T Fz, real_T mu, real_T mu_s){
    
    real_T tan_alpha;
    real_T Fy;
    
    // Define alpha slip
    const real_T alpha_slip = atan2(3*mu*Fz, Calpha);
    
    // If tire is not sliding
    if (fabs(alpha) < alpha_slip) {
        tan_alpha = tan(alpha);
        Fy = -Calpha*tan_alpha + pow(Calpha,2)*(2 - mu_s/mu)*fabs(tan_alpha)*tan_alpha/(3*mu*Fz) - pow(Calpha,3)*pow(tan_alpha,3)*(1-2*mu_s/(3*mu))/(9*pow(mu,2)*pow(Fz,2));
    }
    // If tire is sliding
    else {
        if (alpha > 0) {
            Fy = -mu_s*Fz;
        }
        else if (alpha < 0) {
            Fy = mu_s*Fz;
        }
    }
    // Return tire force value
    return Fy;    
}

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

/* ============================================================================================== *
 * Initialize block
 * ============================================================================================== */
#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */

/* ============================================================================================== *
 * Get model outputs
 * ============================================================================================== */
/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block. Generally outputs are placed in the output vector, ssGetY(S).
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T Bdot, rdot, alphaR, alphaF, Fyf, Fyr, Fxr, max_Fyr;
    real_T rsafe, Bsafe, rsafe_dot, Bsafe_dot, msafe, bsafe, enableSSC;
    real_T manifold_S, Fyf_control, b2_temp;
    real_T b0, b0_dot, b1;
    real_T alpha_slf, alpha_slr, dmax, B_C, r_C, B_D, r_D, b3, b4;
    real_T dmaxdot, B_Cdot, r_Cdot, B_Ddot, r_Ddot, b3dot, b4dot;
    real_T B_rc, r_rc, c1, qrc, c2, B_i, r_i, b5, b6;
    
    // y = Ix (feed states through as outputs)
    real_T  *y = ssGetOutputPortSignal(S,0);

    // Define current inputs - 
    // ssGetInputPortRealSignalPtrs() returns a pointer 
    // referencing where the inputs are stored 
    const real_T r         = **ssGetInputPortRealSignalPtrs(S,0);
    const real_T B         = **ssGetInputPortRealSignalPtrs(S,1);
    const real_T d         = **ssGetInputPortRealSignalPtrs(S,2);
    const real_T Vx        = **ssGetInputPortRealSignalPtrs(S,3);
    const real_T d_dot     = **ssGetInputPortRealSignalPtrs(S,4);
    const real_T pedal     = **ssGetInputPortRealSignalPtrs(S,5);
    const real_T Vx_dot    = **ssGetInputPortRealSignalPtrs(S,6);
    
    // Define parameters
    // ssGetSFcnParam() returns parameters as a complex number, 
    // mxGetPr() gets the real part
    const real_T a              = *mxGetPr(ssGetSFcnParam(S,0));
    const real_T b              = *mxGetPr(ssGetSFcnParam(S,1));
    const real_T CalphaF        = *mxGetPr(ssGetSFcnParam(S,2));
    const real_T CalphaR        = *mxGetPr(ssGetSFcnParam(S,3));
    const real_T Fzf            = *mxGetPr(ssGetSFcnParam(S,4));
    const real_T Fzr            = *mxGetPr(ssGetSFcnParam(S,5));
    const real_T mu             = *mxGetPr(ssGetSFcnParam(S,6));
    const real_T mu_s           = *mxGetPr(ssGetSFcnParam(S,7));
    const real_T m              = *mxGetPr(ssGetSFcnParam(S,8));
    const real_T Iz             = *mxGetPr(ssGetSFcnParam(S,9));  
    const real_T bsafe_const	= *mxGetPr(ssGetSFcnParam(S,10));
    const real_T msafe_const	= *mxGetPr(ssGetSFcnParam(S,11));
    const real_T bsafe_slope	= *mxGetPr(ssGetSFcnParam(S,12)); 
    const real_T q              = *mxGetPr(ssGetSFcnParam(S,13)); 
    const real_T Kcontrol       = *mxGetPr(ssGetSFcnParam(S,14)); 
    
    // Find the rear longitudinal tire forces

    if (pedal >= 1.7){
        Fxr = 5.5*2*190.5*(pedal-1.7)/(2.8-1.7)/.3; //divide by pedal range and tire radius
    }
    else {
        Fxr = 5.5*2*70.5*(pedal-1.7)/(0.9-1.7)/.3;
    }

/* // old map (before Aug 2011)
    if (pedal >= 1.9){
        Fxr = 5.5*2*210*(pedal-1.9)/(2.8-1.9)/.3; //divide by pedal range and tire radius
    }
    else {
        Fxr = 5.5*2*90*(pedal-1.7)/(0.9-1.9)/.3;
    }
*/
    // Get the maximum available rear lateral tire force
    if (Fxr > mu*Fzr){
        Fxr = mu*Fzr;
    }
    if (Fxr < -mu*Fzr){
        Fxr = -mu*Fzr;
    }
    max_Fyr = pow(fabs((mu*Fzr*mu*Fzr - Fxr*Fxr)), .5);
    
    // Find front and rear slip angles from previous states
    alphaF = -d + atan2(B*Vx + a*r, Vx);
    alphaR = atan2(B*Vx - b*r, Vx);
      
    // Find fiala tire forces
    Fyf = getFialaTireForce(CalphaF, alphaF, Fzf, mu, mu_s);
    Fyr = getFialaTireForce(CalphaR, alphaR, max_Fyr, mu, mu_s);
    
    // Solve for sideslip angle rate
    Bdot = (Fyf + Fyr)/m/Vx - r;
    // Solve for yaw rate rate
    rdot = (a*Fyf - b*Fyr)/Iz;
    
    // Set outputs to do nothing if controller is off
    rsafe = r;
    Bsafe = B;
    rsafe_dot = rdot;
    Bsafe_dot = Bdot;
    Fyf_control = Fyf;
    manifold_S = 0;
    enableSSC = 0;
    
    // Only turn on SSC when velocity is high enough
    // (no divide by zero)
    if (Vx > 2){
        
        // define quantities needed for yaw rate nullcline boundaries
        alpha_slf = atan2(3*mu*Fzf, CalphaF);                               // max front slip angle
        alpha_slr = atan2(3*mu*Fzr, CalphaR);                               // max rear slip angle
        dmax = atan((a+b)*mu*g/Vx/Vx - tan(alpha_slr)) + alpha_slf;         // max stable delta
        B_C = b*mu*g/Vx/Vx - tan(alpha_slr);                                // upper left corner of safe area                      
        r_C = mu*g/Vx;                                                      // upper left corner of safe area 
        B_D = (b/(a+b))*(tan(alpha_slf+dmax) - tan(alpha_slr)) + tan(alpha_slr); // upper right corner of safe area
        r_D = (Vx/(a+b))*(tan(alpha_slf+dmax) - tan(alpha_slr));            // upper right corner of safe area
        b3 = (r_D-r_C)/(B_D-B_C);                                           // slope of yaw rate boundary
        b4 = r_C - B_C*(r_D-r_C)/(B_D-B_C);                                 // intercept of yaw rate boundary
        
        // define derivative quantities needed for yaw rate nullcline boundaries
        dmaxdot = (-2*Vx*(a+b)*mu*g*Vx_dot)/(Vx*Vx*Vx*Vx + ((a+b)*mu*g-Vx*Vx*tan(alpha_slr))*((a+b)*mu*g-Vx*Vx*tan(alpha_slr)));
        B_Cdot = -2*b*mu*g*Vx_dot/(Vx*Vx*Vx);
        r_Cdot = -mu*g*Vx_dot/(Vx*Vx);
        B_Ddot = (b*dmaxdot)/(a+b)/(cos(alpha_slf+dmax))/(cos(alpha_slf+dmax));
        r_Ddot = Vx_dot*(tan(alpha_slf+dmax)-tan(alpha_slr))/(a+b) + Vx*dmaxdot/(a+b)/(cos(alpha_slf+dmax))/(cos(alpha_slf+dmax));
        b3dot = (r_Ddot-r_Cdot)/(B_D-B_C)- (r_D-r_C)*(B_Ddot-B_Cdot)/(B_D-B_C)/(B_D-B_C);
        b4dot = r_Cdot + (-r_Ddot*B_C-r_D*B_Cdot+r_Cdot*B_C+r_C*B_Cdot)/(B_D-B_C) + (r_D*B_C-r_C*B_C)*(B_Ddot-B_Cdot)/(B_D-B_C)/(B_D-B_C);
                
        // Define quantities needed for sideslip boundaries
        b0 = b/Vx;                          // slope of sideslip boundary
        b0_dot = -b*Vx_dot/(Vx*Vx);         // derivative of slope of sideslip boundary    
        b1 = tan(alpha_slr);                // intercept of sideslip boundary
        
        // Define quantities needed to match 'S' level curves at each boundary intersection
        B_rc = (1.6*mu*g/Vx-b4)/b3;             // point directly above corner and yaw boundary intersection (in yaw direction)
        r_rc = 10*1.6*mu*g/Vx;                  // point directly above corner and yaw boundary intersection (in yaw direction)
        c1 = (r_rc*(1+q*b3)-b3*B_rc-b4-q*B_rc*b3*b3-q*b3*b4)/(1+b3*b3); // to match level curves of yaw and corner boundaries
        qrc = c1/(r_rc-(1.6*mu*g/Vx));          // to match level curves of yaw and corner boundaries
        c2 = 10;                                // this needs to be a big value for an accurate r-B line 
        B_i = (c2*(1+q*b3+b0*b0*q*b3-q*b0-b3*b3*b0*b0-b3*b3*q*b0)+(b0+q)*(-b4*b0-q*b3*b4*b0)+(1+q*b3)*(b1*b0-q*b1))/(b0+q)/(b3*b0+q*b0*b3*b3-1-q*b3);   // define line between intersecting S level curves for yaw and beta boundaries
        r_i = (c2*(1+b3*b3)+b3*B_i+b4+q*B_i*b3*b3+q*b3*b4)/(1+q*b3);    // define line between intersecting S level curves for yaw and beta boundaries

        b5 = (r_i-r_C)/(B_i-B_C);               // define line between intersecting S level curves for yaw and beta boundaries
        b6 = r_C - B_C*(r_i-r_C)/(B_i-B_C);     // define line between intersecting S level curves for yaw and beta boundaries
   
        // If states are above the sideslip limit, find safe point
        if (B >= (b1 + b0*r)) {
            rsafe = (1/(1+b0*b0))*(r+b0*B - b1*b0);
            Bsafe = (1/(1+b0*b0))*(b0*r+b0*b0*B + b1);

            manifold_S = r - rsafe - q*B + q*Bsafe;
            Fyf_control = (Fyr*((b/Iz)*(b0*b0 + q*b0)+(1/m/Vx)*(q+b0))-Kcontrol*manifold_S*(1+b0*b0) - B*(2*q*b0*b0_dot-b0_dot+2*b0*b0*b0_dot*(1-q*b0)/(1+b0*b0)) - r*(q*b0_dot+q+b0+2*b0*b0_dot*(1-q*b0)/(1+b0*b0)) + b1*(-b0_dot+2*b0*b0_dot*(b0+q)/(1+b0*b0)))/((a/Iz)*(b0*b0+ q*b0)-(1/m/Vx)*(q+b0));

            enableSSC = 1;
        }   
        if (B <= (-b1 + b0*r)) {
            rsafe = (1/(1+b0*b0))*(r+b0*B + b1*b0);     //sign of b1 is changed from other case
            Bsafe = (1/(1+b0*b0))*(b0*r+b0*b0*B - b1);  //sign of b1 is changed from other case

            manifold_S = r - rsafe - q*B + q*Bsafe;
            Fyf_control = (Fyr*((b/Iz)*(b0*b0 + q*b0)+(1/m/Vx)*(q+b0))-Kcontrol*manifold_S*(1+b0*b0) - B*(2*q*b0*b0_dot-b0_dot+2*b0*b0*b0_dot*(1-q*b0)/(1+b0*b0)) - r*(q*b0_dot+q+b0+2*b0*b0_dot*(1-q*b0)/(1+b0*b0)) - b1*(-b0_dot+2*b0*b0_dot*(b0+q)/(1+b0*b0)))/((a/Iz)*(b0*b0+ q*b0)-(1/m/Vx)*(q+b0)); //sign of b1 is changed from other case

            enableSSC = 1;
        }
        if((r >= b3*B + b4) && (r > b5*B+b6) && (B < b1+b0*r)) {
            rsafe = (1/(1+b3*b3))*(b3*b3*r + b3*B + b4);
            Bsafe = (1/(1+b3*b3))*(b3*r + B - b3*b4);
 
            manifold_S = r - rsafe - q*B + q*Bsafe;          
            Fyf_control = (-Kcontrol*manifold_S*(b3*b3+1) + Fyr*(b*(1+q*b3)/Iz+(b3+q*b3*b3)/m/Vx) - r*(b3dot*(q-2*b3)+b3+q*b3*b3+2*b3*b3*b3dot*(b3-q)/(b3*b3+1)) - B*(-b3dot+2*b3*b3dot*(b3-q)/(b3*b3+1)) - b4*(-q*b3dot+2*b3*b3dot*(1+q*b3)/(b3*b3+1)) + b4dot*(q*b3+1)) / (a*(1+q*b3)/Iz-(b3+q*b3*b3)/m/Vx);
            
            enableSSC = 1;
        } 
        // upper right corner case
        if((r > 1.6*mu*g/Vx) && (B < b1+b0*r) && (B > B_rc)){
        	rsafe = 1.6*r_C;
            
        	manifold_S = qrc*(r - rsafe);
        	Fyf_control =(-Kcontrol*Iz*manifold_S/qrc+b*Fyr-1.6*mu*g*Iz*Vx_dot*(1)/(Vx*Vx))/a;
            
            enableSSC = 1;
        }
            
        if((r <= b3*B - b4) && (r < b5*B-b6) && (B > -b1+b0*r)) {
            rsafe = (1/(1+b3*b3))*(b3*b3*r + b3*B - b4); //sign of b4 is changed from other case
            Bsafe = (1/(1+b3*b3))*(b3*r + B + b3*b4);    //sign of b4 is changed from other case
           
            manifold_S = r - rsafe - q*B + q*Bsafe;            
            Fyf_control = (-Kcontrol*manifold_S*(b3*b3+1) + Fyr*(b*(1+q*b3)/Iz+(b3+q*b3*b3)/m/Vx) - r*(b3dot*(q-2*b3)+b3+q*b3*b3+2*b3*b3*b3dot*(b3-q)/(b3*b3+1)) - B*(-b3dot+2*b3*b3dot*(b3-q)/(b3*b3+1)) + b4*(-q*b3dot+2*b3*b3dot*(1+q*b3)/(b3*b3+1)) - b4dot*(q*b3+1)) / (a*(1+q*b3)/Iz-(b3+q*b3*b3)/m/Vx); //sign of b4 is changed from other case
            
            enableSSC = 1;
        }
        // lower left corner case
        if((r < -1.6*mu*g/Vx) && (B > -b1+b0*r) && (B < -B_rc)){
        	rsafe = -1.6*r_C;
            
        	manifold_S = qrc*(r - rsafe);
        	Fyf_control =(-Kcontrol*Iz*manifold_S/qrc+b*Fyr-1.6*mu*g*Iz*Vx_dot*(-1)/(Vx*Vx))/a;
            
            enableSSC = 1;
        }
	}
////////////////////////////////////////////////////////////////////////
    // Solve for safe point (original safe area)
/*
    if (r <= 0){
        if (B >= (bsafe_const + bsafe_slope*d + msafe_const*r)) {
            msafe = msafe_const;
            bsafe = bsafe_const + bsafe_slope*d;
            rsafe = (1/(1+msafe*msafe))*(r + msafe*B - bsafe*msafe);
            Bsafe = (1/(1+msafe*msafe))*(msafe*r + msafe*msafe*B + bsafe);
            rsafe_dot = (1/(1+msafe*msafe))*(rdot + msafe*Bdot-msafe*bsafe_slope*d_dot);
            Bsafe_dot = (1/(1+msafe*msafe))*(msafe*rdot + msafe*msafe*Bdot+1*bsafe_slope*d_dot);
            
            manifold_S = r - rsafe - q*B + q*Bsafe;
            
            b2_temp = (q+msafe)/(1+msafe*msafe);
            Fyf_control = (-b2_temp*r-Kcontrol*manifold_S+(b*msafe*b2_temp/Iz+b2_temp/m/Vx)*Fyr)/(a*msafe*b2_temp/Iz-b2_temp/m/Vx);
            
            enableSSC = 1;
        }   
    }
    else {
        if (B <= (-bsafe_const + bsafe_slope*d + msafe_const*r)) {
            msafe = msafe_const;
            bsafe = -bsafe_const + bsafe_slope*d;
            rsafe = (1/(1+msafe*msafe))*(r + msafe*B - bsafe*msafe);
            Bsafe = (1/(1+msafe*msafe))*(msafe*r + msafe*msafe*B + bsafe);
            rsafe_dot = (1/(1+msafe*msafe))*(rdot + msafe*Bdot-msafe*bsafe_slope*d_dot);
            Bsafe_dot = (1/(1+msafe*msafe))*(msafe*rdot + msafe*msafe*Bdot+1*bsafe_slope*d_dot);
            
            manifold_S = r - rsafe - q*B + q*Bsafe;
            
            b2_temp = (q+msafe)/(1+msafe*msafe);
            Fyf_control = (-b2_temp*r-Kcontrol*manifold_S+(b*msafe*b2_temp/Iz+b2_temp/m/Vx)*Fyr)/(a*msafe*b2_temp/Iz-b2_temp/m/Vx);
           
            enableSSC = 1;
        }
    }
    
*/
////////////////////////////////////////////////////////////////////////
/*
    // Solve for safe point (parallelogram safe area)
    // If states are above the yaw rate limit, find safe point
    if(r > mu*g/10) {
        rsafe = mu*g/10;
        Bsafe = B;
        rsafe_dot = 0;
        Bsafe_dot = Bdot;

        manifold_S = r - rsafe - q*B + q*Bsafe;
        Fyf_control = (-Kcontrol*Iz*manifold_S + b*Fyr)/a;

        enableSSC = 1;
    }
    if(r < -mu*g/10) {
        rsafe = -mu*g/10;
        Bsafe = B;
        rsafe_dot = 0;
        Bsafe_dot = Bdot;

        manifold_S = r - rsafe - q*B + q*Bsafe;
        Fyf_control = (-Kcontrol*Iz*manifold_S + b*Fyr)/a;

        enableSSC = 1;
    }
    // If states are above the sideslip limit, find safe point
    if (B >= (bsafe_const + bsafe_slope*d + msafe_const*r)) {
        msafe = msafe_const;
        bsafe = bsafe_const + bsafe_slope*d;
        rsafe = (1/(1+msafe*msafe))*(r + msafe*B - bsafe*msafe);
        Bsafe = (1/(1+msafe*msafe))*(msafe*r + msafe*msafe*B + bsafe);
        rsafe_dot = (1/(1+msafe*msafe))*(rdot + msafe*Bdot-msafe*bsafe_slope*d_dot);
        Bsafe_dot = (1/(1+msafe*msafe))*(msafe*rdot + msafe*msafe*Bdot+1*bsafe_slope*d_dot);

        manifold_S = r - rsafe - q*B + q*Bsafe;

        b2_temp = (q+msafe)/(1+msafe*msafe);
        Fyf_control = (-b2_temp*r-Kcontrol*manifold_S+(b*msafe*b2_temp/Iz+b2_temp/m/Vx)*Fyr)/(a*msafe*b2_temp/Iz-b2_temp/m/Vx);

        enableSSC = 1;
    }   
    if (B <= (-bsafe_const + bsafe_slope*d + msafe_const*r)) {
        msafe = msafe_const;
        bsafe = -bsafe_const + bsafe_slope*d;
        rsafe = (1/(1+msafe*msafe))*(r + msafe*B - bsafe*msafe);
        Bsafe = (1/(1+msafe*msafe))*(msafe*r + msafe*msafe*B + bsafe);
        rsafe_dot = (1/(1+msafe*msafe))*(rdot + msafe*Bdot-msafe*bsafe_slope*d_dot);
        Bsafe_dot = (1/(1+msafe*msafe))*(msafe*rdot + msafe*msafe*Bdot+1*bsafe_slope*d_dot);

        manifold_S = r - rsafe - q*B + q*Bsafe;

        b2_temp = (q+msafe)/(1+msafe*msafe);
        Fyf_control = (-b2_temp*r-Kcontrol*manifold_S+(b*msafe*b2_temp/Iz+b2_temp/m/Vx)*Fyr)/(a*msafe*b2_temp/Iz-b2_temp/m/Vx);

        enableSSC = 1;
    }
 */
 ////////////////////////////////////////////////////////////////////////
 /*   
    // Solve for safe point (parallelogram safe area with changing speeds)
    // If states are above the yaw rate limit, find safe point
    if (Vx > 1){   
        if(r > mu*g/Vx) {
            rsafe = mu*g/Vx;
            Bsafe = B;
            rsafe_dot = 0;
            Bsafe_dot = Bdot;

            manifold_S = r - rsafe - q*B + q*Bsafe;
            Fyf_control = (-Kcontrol*10*Iz*manifold_S + b*Fyr - mu*g*Iz*Vx_dot*(1)/(Vx*Vx))/a;

            enableSSC = 1;
        }
        if(r < -mu*g/Vx) {
            rsafe = -mu*g/Vx;
            Bsafe = B;
            rsafe_dot = 0;
            Bsafe_dot = Bdot;

            manifold_S = r - rsafe - q*B + q*Bsafe;
            Fyf_control = (-Kcontrol*10*Iz*manifold_S + b*Fyr - mu*g*Iz*Vx_dot*(-1)/(Vx*Vx))/a;

            enableSSC = 1;
        }
   
        b0 = b/Vx;
        b0_dot = -b*Vx_dot/(Vx*Vx);
        b1 = bsafe_const;
   
        // If states are above the sideslip limit, find safe point
        if (B >= (b1 + b0*r)) {
            rsafe = (1/(1+b0*b0))*(r+b0*B - b1*b0);
            Bsafe = (1/(1+b0*b0))*(b0*r+b0*b0*B + b1);
            rsafe_dot = 0; //change later
            Bsafe_dot = 0; //change later

            manifold_S = r - rsafe - q*B + q*Bsafe;

            Fyf_control = (Fyr*((b/Iz)*(b0*b0 + q*b0)+(1/m/Vx)*(q+b0))-Kcontrol*manifold_S*(1+b0*b0) - B*(2*q*b0*b0_dot-b0_dot+2*b0*b0*b0_dot*(1-q*b0)/(1+b0*b0)) - r*(q*b0_dot+q+b0+2*b0*b0_dot*(1-q*b0)/(1+b0*b0)) + b1*(-b0_dot+2*b0*b0_dot*(b0+q)/(1+b0*b0)))/((a/Iz)*(b0*b0+ q*b0)-(1/m/Vx)*(q+b0));

            enableSSC = 1;
        }   
        if (B <= (-b1 + b0*r)) {
            rsafe = (1/(1+b0*b0))*(r+b0*B + b1*b0);     //sign of b1 is changed from other case
            Bsafe = (1/(1+b0*b0))*(b0*r+b0*b0*B - b1);  //sign of b1 is changed from other case
            rsafe_dot = 0; //change later
            Bsafe_dot = 0; //change later

            manifold_S = r - rsafe - q*B + q*Bsafe;

            Fyf_control = (Fyr*((b/Iz)*(b0*b0 + q*b0)+(1/m/Vx)*(q+b0))-Kcontrol*manifold_S*(1+b0*b0) - B*(2*q*b0*b0_dot-b0_dot+2*b0*b0*b0_dot*(1-q*b0)/(1+b0*b0)) - r*(q*b0_dot+q+b0+2*b0*b0_dot*(1-q*b0)/(1+b0*b0)) - b1*(-b0_dot+2*b0*b0_dot*(b0+q)/(1+b0*b0)))/((a/Iz)*(b0*b0+ q*b0)-(1/m/Vx)*(q+b0)); //sign of b1 is changed from other case

            enableSSC = 1;
        }
	}
 */
////////////////////////////////////////////////////////////////////////  
/*
    // Solve for safe point (isocline safe area with changing speeds) -- working code used in thesis data
    if (Vx > 2){
        
        // define quantities needed for yaw rate (triangle) boundaries
        alpha_slf = atan2(3*mu*Fzf, CalphaF);                           // max front slip angle
        alpha_slr = atan2(3*mu*Fzr, CalphaR);                           // max rear slip angle
        dmax = atan((a+b)*mu*g/Vx/Vx - tan(alpha_slr)) + alpha_slf;    // max stable delta
        B_C = b*mu*g/Vx/Vx - tan(alpha_slr);                            // upper left corner of safe area                      
        r_C = mu*g/Vx;                                                  // upper left corner of safe area 
        B_D = (b/(a+b))*(tan(alpha_slf+dmax) - tan(alpha_slr)) + tan(alpha_slr); // upper right corner of safe area
        r_D = (Vx/(a+b))*(tan(alpha_slf+dmax) - tan(alpha_slr));       // upper right corner of safe area
        b3 = (r_D-r_C)/(B_D-B_C);                                       // slope of yaw rate boundary
        b4 = r_C - B_C*(r_D-r_C)/(B_D-B_C);                             // intercept of yaw rate boundary
        
        dmaxdot = (-2*Vx*(a+b)*mu*g*Vx_dot)/(Vx*Vx*Vx*Vx + ((a+b)*mu*g-Vx*Vx*tan(alpha_slr))*((a+b)*mu*g-Vx*Vx*tan(alpha_slr)));
        B_Cdot = -2*b*mu*g*Vx_dot/(Vx*Vx*Vx);
        r_Cdot = -mu*g*Vx_dot/(Vx*Vx);
        B_Ddot = (b*dmaxdot)/(a+b)/(cos(alpha_slf+dmax))/(cos(alpha_slf+dmax));
        r_Ddot = Vx_dot*(tan(alpha_slf+dmax)-tan(alpha_slr))/(a+b) + Vx*dmaxdot/(a+b)/(cos(alpha_slf+dmax))/(cos(alpha_slf+dmax));
        b3dot = (r_Ddot-r_Cdot)/(B_D-B_C)- (r_D-r_C)*(B_Ddot-B_Cdot)/(B_D-B_C)/(B_D-B_C);
        b4dot = r_Cdot + (-r_Ddot*B_C-r_D*B_Cdot+r_Cdot*B_C+r_C*B_Cdot)/(B_D-B_C) + (r_D*B_C-r_C*B_C)*(B_Ddot-B_Cdot)/(B_D-B_C)/(B_D-B_C);
        
        
        
        // Define quantities needed for sideslip boundaries
        b0 = b/Vx;                      // slope of sideslip boundary
        b0_dot = -b*Vx_dot/(Vx*Vx);     // derivative of slope of sideslip boundary    
        b1 = tan(alpha_slr);               // intercept of sideslip boundary
   
        // If states are above the sideslip limit, find safe point
        if (B >= (b1 + b0*r)) {
            rsafe = (1/(1+b0*b0))*(r+b0*B - b1*b0);
            Bsafe = (1/(1+b0*b0))*(b0*r+b0*b0*B + b1);
            rsafe_dot = 0; //change later
            Bsafe_dot = 0; //change later

            manifold_S = r - rsafe - q*B + q*Bsafe;

            Fyf_control = (Fyr*((b/Iz)*(b0*b0 + q*b0)+(1/m/Vx)*(q+b0))-Kcontrol*manifold_S*(1+b0*b0) - B*(2*q*b0*b0_dot-b0_dot+2*b0*b0*b0_dot*(1-q*b0)/(1+b0*b0)) - r*(q*b0_dot+q+b0+2*b0*b0_dot*(1-q*b0)/(1+b0*b0)) + b1*(-b0_dot+2*b0*b0_dot*(b0+q)/(1+b0*b0)))/((a/Iz)*(b0*b0+ q*b0)-(1/m/Vx)*(q+b0));

            enableSSC = 1;
        }   
        if (B <= (-b1 + b0*r)) {
            rsafe = (1/(1+b0*b0))*(r+b0*B + b1*b0);     //sign of b1 is changed from other case
            Bsafe = (1/(1+b0*b0))*(b0*r+b0*b0*B - b1);  //sign of b1 is changed from other case
            rsafe_dot = 0; //change later
            Bsafe_dot = 0; //change later

            manifold_S = r - rsafe - q*B + q*Bsafe;

            Fyf_control = (Fyr*((b/Iz)*(b0*b0 + q*b0)+(1/m/Vx)*(q+b0))-Kcontrol*manifold_S*(1+b0*b0) - B*(2*q*b0*b0_dot-b0_dot+2*b0*b0*b0_dot*(1-q*b0)/(1+b0*b0)) - r*(q*b0_dot+q+b0+2*b0*b0_dot*(1-q*b0)/(1+b0*b0)) - b1*(-b0_dot+2*b0*b0_dot*(b0+q)/(1+b0*b0)))/((a/Iz)*(b0*b0+ q*b0)-(1/m/Vx)*(q+b0)); //sign of b1 is changed from other case

            enableSSC = 1;
        }
        if(r >= b3*B + b4) {
            rsafe = (1/(1+b3*b3))*(b3*b3*r + b3*B + b4);
            Bsafe = (1/(1+b3*b3))*(b3*r + B - b3*b4);
            rsafe_dot = 0; //change later
            Bsafe_dot = 0; //change later
 
            manifold_S = r - rsafe - q*B + q*Bsafe;
            
            Fyf_control = (-Kcontrol*manifold_S*(b3*b3+1) + Fyr*(b*(1+q*b3)/Iz+(b3+q*b3*b3)/m/Vx) - r*(b3dot*(q-2*b3)+b3+q*b3*b3+2*b3*b3*b3dot*(b3-q)/(b3*b3+1)) - B*(-b3dot+2*b3*b3dot*(b3-q)/(b3*b3+1)) - b4*(-q*b3dot+2*b3*b3dot*(1+q*b3)/(b3*b3+1)) + b4dot*(q*b3+1)) / (a*(1+q*b3)/Iz-(b3+q*b3*b3)/m/Vx);
            
            // assuming Vxdot = 0
            // Fyf_control = (-Kcontrol*manifold_S*(b3*b3+1)-r*(b3+q*b3*b3)+Fyr*(b*(1+q*b3)/Iz + (b3+q*b3*b3)/(m*Vx))) / (a*(1+q*b3)/Iz - (b3+q*b3*b3)/(m*Vx));
            
            // upper left corner case
            //if(B <= (-b1 + b0*r)){
            //    rsafe = r_C;
            //    Bsafe = B_C;
            //    manifold_S = r - rsafe - q*B + q*Bsafe;
            //    Fyf_control = (-Kcontrol*manifold_S - q*r - q*B_Cdot+r_Cdot + Fyr*(b/Iz+q/m/Vx)) / (a/Iz-q/m/Vx);
            //}
            
            enableSSC = 1;
        }
        if(r <= b3*B - b4) {
            rsafe = (1/(1+b3*b3))*(b3*b3*r + b3*B - b4); //sign of b4 is changed from other case
            Bsafe = (1/(1+b3*b3))*(b3*r + B + b3*b4);    //sign of b4 is changed from other case
            rsafe_dot = 0; //change later
            Bsafe_dot = 0; //change later
           
            manifold_S = r - rsafe - q*B + q*Bsafe;
            
            Fyf_control = (-Kcontrol*manifold_S*(b3*b3+1) + Fyr*(b*(1+q*b3)/Iz+(b3+q*b3*b3)/m/Vx) - r*(b3dot*(q-2*b3)+b3+q*b3*b3+2*b3*b3*b3dot*(b3-q)/(b3*b3+1)) - B*(-b3dot+2*b3*b3dot*(b3-q)/(b3*b3+1)) + b4*(-q*b3dot+2*b3*b3dot*(1+q*b3)/(b3*b3+1)) - b4dot*(q*b3+1)) / (a*(1+q*b3)/Iz-(b3+q*b3*b3)/m/Vx); //sign of b4 is changed from other case
            
            // assuming Vxdot = 0
            // Fyf_control = (-Kcontrol*manifold_S*(b3*b3+1)-r*(b3+q*b3*b3)+Fyr*(b*(1+q*b3)/Iz + (b3+q*b3*b3)/(m*Vx))) / (a*(1+q*b3)/Iz - (b3+q*b3*b3)/(m*Vx));
            
            // lower right corner case
            //if(B >= (b1 + b0*r)){
            //    rsafe = -r_C;
            //    Bsafe = -B_C;
            //    manifold_S = r - rsafe - q*B + q*Bsafe;
            //    Fyf_control = (-Kcontrol*manifold_S - q*r + q*B_Cdot-r_Cdot + Fyr*(b/Iz+q/m/Vx)) / (a/Iz-q/m/Vx);
            //}
            
            enableSSC = 1;
        }
	}
*/
    
    // Set outputs
    y[0] = rsafe;
    y[1] = Bsafe;
    y[2] = rsafe_dot;
    y[3] = Bsafe_dot;
    y[4] = enableSSC;
    y[5] = Fyr;
    y[6] = Fyf_control;
    y[7] = manifold_S;
}

/* ============================================================================================== *
 * update discrete states
 * ============================================================================================== */
#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
  }
#endif /* MDL_UPDATE */

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