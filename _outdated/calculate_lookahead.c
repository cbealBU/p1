/*calculate_lookahead.c
 *Rami Y. Hindiyeh
 *DDL
 *
 *Computes variable lookahead for controller based upon degree of rear tire
 *saturation
*/
#define S_FUNCTION_NAME  calculate_lookahead
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>
#include <stdio.h>

#define NUMOFPARAMS (9)
#define NUMOFINPUTS (4)
#define NUMOFOUTPUTS (3)

#define MINSPEED (1)
#define GRAVACC  (9.81)

/*Parameter List
 *Ts
 *param.fl.C
 *param.fr.C
 *param.rl.C
 *param.rr.C
 *param.a
 *param.b
 *param.m
 *
 **/

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
    const real_T Ts = mxGetPr(ssGetSFcnParam(S,0))[0];
    
    ssSetSampleTime(S, 0, Ts);
    ssSetOffsetTime(S, 0, 0.0);
}

#undef MDL_INITIALIZE_CONDITIONS
#undef MDL_DERIVATIVES


static void mdlOutputs(SimStruct *S, int_T tid)
{
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    //Extract vehicle parameters from input parameters
    const real_T a = mxGetPr(ssGetSFcnParam(S,5))[0]; 
    const real_T b = mxGetPr(ssGetSFcnParam(S,6))[0];
    const real_T m = mxGetPr(ssGetSFcnParam(S,7))[0];
    const real_T Cfl = mxGetPr(ssGetSFcnParam(S,1))[0];
    const real_T Cfr = mxGetPr(ssGetSFcnParam(S,2))[0];
    const real_T Crl = mxGetPr(ssGetSFcnParam(S,3))[0];
    const real_T Crr = mxGetPr(ssGetSFcnParam(S,4))[0];
    const real_T k = mxGetPr(ssGetSFcnParam(S,8))[0]; //Potential field gain
    const real_T Cf =  Cfl + Cfr; //Lumped front tire stiffness
    const real_T Cr = Crl + Crr; //Lumped rear tire stiffness
    const real_T L = a + b;
    const real_T Fzf_nom = (b/L)*m*GRAVACC; //Nominal front axle normal load
    const real_T Fzr_nom = (a/L)*m*GRAVACC; //Nominal rear axle normal load
    
    const real_T mu_p = 1; //For now, set peak friction coeff to 1
    const real_T mu_s = 0.8; //For now, set sliding friction coeff to 0.8
  
    const real_T alpha_slf = atan2(3*mu_p*Fzf_nom,Cf);
    const real_T alpha_slr = atan2(3*mu_p*Fzr_nom,Cr);
    
    //Input measurements
    real_T Vx, VyCG, r,delta;
    
    //Calculated quantities
    real_T alpha_f, alpha_r, tan_alpha_f, tan_alpha_r; //Front and rear wheel slip angles
    real_T Fy_f, Fy_r; //Front and rear lateral forces
    real_T l_f, l_r; //Front and rear scaling factors
    real_T xla; //Computed lookahead value
    
    
    //Parse input vector into specific input variables;
    Vx = *uPtrs[0];
    VyCG = *uPtrs[1];
    r = *uPtrs[2];
    delta = *uPtrs[3];
    
    //Compute slip angle at front and rear tires:
    
    if (Vx > MINSPEED) {
        alpha_f = atan2(VyCG + a*r,Vx) - delta;
        alpha_r = atan2(VyCG - b*r,Vx);
        }
    else  {
        alpha_f = 0;
        alpha_r = 0;
    }
    
    //Now, compute the lumped tire force estimates using the slip angles
    if (fabs(alpha_f) < alpha_slf) {
        tan_alpha_f = tan(alpha_f);
        Fy_f = -Cf*tan_alpha_f + pow(Cf,2)*(2 - mu_s/mu_p)*fabs(tan_alpha_f)*tan_alpha_f/(3*mu_p*Fzf_nom) - pow(Cf,3)*pow(tan_alpha_f,3)*(1-2*mu_s/(3*mu_p))/(9*pow(mu_p,2)*pow(Fzf_nom,2));
        
    }
    else {
        if (alpha_f > 0) {
            Fy_f = -mu_s*Fzf_nom;
        }
        else if (alpha_f < 0) {
            Fy_f = mu_s*Fzf_nom;
        }
    }
    
    if (fabs(alpha_r) < alpha_slr) {
        tan_alpha_r = tan(alpha_r);
        Fy_r = -Cr*tan_alpha_r + pow(Cr,2)*(2 - mu_s/mu_p)*fabs(tan_alpha_r)*tan_alpha_r/(3*mu_p*Fzr_nom) - pow(Cr,3)*pow(tan_alpha_r,3)*(1-2*mu_s/(3*mu_p))/(9*pow(mu_p,2)*pow(Fzr_nom,2));
    }
    else {
        if (alpha_r > 0) {
            Fy_r = -mu_s*Fzr_nom;
//             printf("Rear Saturation Positive\r\n");
        }
        else if (alpha_r < 0) {
            Fy_r = mu_s*Fzr_nom;
//             printf("Rear Saturation Negative\r\n");
        }
    }
    
    if (alpha_f == 0) {
        l_f = 1;
    }
    else {
        l_f = Fy_f/(-Cf*alpha_f);
    }
    
    if (alpha_r == 0) {
        l_r = 1;
    }
    else {
        l_r = Fy_r/(-Cr*alpha_r);
        
        if (l_r < 0) {
            printf("alpha_slr = %f, alpha_r = %f\r\n", alpha_slr, alpha_r);
            printf("Fy_r = %f, Fy_rlin = %f\r\n", Fy_r, -Cr*alpha_r);
        }
    }
      
    xla = (l_f*Cf + l_r*Cr)/(2*k);    
    
    y[0] = xla;
    y[1] = l_f;
    y[2] = l_r;
        
}



static void mdlTerminate(SimStruct *S)
{

}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
