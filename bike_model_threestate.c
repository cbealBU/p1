#define S_FUNCTION_NAME  bike_model_threestate
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>
#include <stdio.h>

#define NUMOFINPUTS 	8
#define NUMOFOUTPUTS	9
#define NUMOFSTATES	3
#define NUMOFPARAMS 16

#define PI     	3.14159 
#define GRAVACC (9.81)
#define D2R			(1) /*(PI/180)*/	/* degree to radian */
#define R2D			(1) /*(180/PI)*/	/* radian to degree */

#define SPEEDLIMIT (0.2)

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

/*====================*
 * S-function methods *
 *====================*/
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUMOFPARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    ssSetNumContStates(S, NUMOFSTATES);
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
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *x0 = ssGetContStates(S);
    int_T i;

    /* Initial Conditions */
//     for (i = 0; i < NUMOFSTATES; i++) { 
//         x0[i] = 0;
//     }
    x0[0] = 0;
    x0[1] = 0;
    x0[2] = 0.21;
    
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T *x = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    real_T *y = ssGetOutputPortRealSignal(S,0);
    
    real_T CurrentTime = ssGetT(S);
    
    real_T delta;
    real_T deltal = *uPtrs[0];
    real_T deltar = *uPtrs[1];
    real_T taulr = *uPtrs[2];
    real_T taurr = *uPtrs[3];
    real_T mu_f_disturbance = *uPtrs[4];
    real_T mu_r_disturbance = *uPtrs[5];
    real_T UseTwoWheel = *uPtrs[6];
    real_T UseDiffDrive = *uPtrs[7];
    
    real_T Uy = x[0];
    real_T r  = x[1];
    real_T Ux = x[2];
    
    real_T alphalf, alpharf, alphalr, alpharr;
    real_T alpha_f, alpha_r;
    
    real_T Fxlf, Fxrf, Fxlr, Fxrr, Fxlr_command, Fxrr_command;
    real_T Fylf, Fyrf, Fylr, Fyrr;
    real_T Fyf, Fyr, Fxf, Fxr, Fxr_command;
    real_T M_diff;
    
    const real_T Clf = mxGetPr(ssGetSFcnParam(S, 0))[0];
    const real_T Crf = mxGetPr(ssGetSFcnParam(S, 1))[0];
    const real_T Clr = mxGetPr(ssGetSFcnParam(S, 2))[0];
    const real_T Crr = mxGetPr(ssGetSFcnParam(S, 3))[0];
    const real_T Cf = Clf + Crf;
    const real_T Cr = Clr + Crr;
    
    const real_T M = mxGetPr(ssGetSFcnParam(S, 4))[0];
    const real_T Iz = mxGetPr(ssGetSFcnParam(S, 5))[0];
    
    const real_T a = mxGetPr(ssGetSFcnParam(S, 6))[0];
    const real_T b  = mxGetPr(ssGetSFcnParam(S, 7))[0];
    const real_T L = a + b;
    const real_T c =  mxGetPr(ssGetSFcnParam(S, 8))[0];
    
    const real_T tp = mxGetPr(ssGetSFcnParam(S, 9))[0];
    const real_T tm = mxGetPr(ssGetSFcnParam(S,10))[0];
    const real_T mu_p_f = mxGetPr(ssGetSFcnParam(S,11))[0];
    const real_T mu_s_f = mxGetPr(ssGetSFcnParam(S,12))[0];
    const real_T mu_p_r = mxGetPr(ssGetSFcnParam(S, 13))[0];
    const real_T mu_s_r = mxGetPr(ssGetSFcnParam(S,14))[0];
    const real_T r_w = mxGetPr(ssGetSFcnParam(S,15))[0];
    
    const real_T Fzf = (b/L)*M*GRAVACC; //Nominal front axle normal load
    const real_T Fzr = (a/L)*M*GRAVACC; //Nominal rear axle normal load
    
    const real_T Fzlf = Fzf/2;
    const real_T Fzrf = Fzf/2;
    const real_T Fzlr = Fzr/2;
    const real_T Fzrr = Fzr/2;
    
    delta = (deltal + deltar)/2;
    
    Fxlf = 0;
    Fxrf = 0;
    Fxf = Fxlf + Fxrf;
    
    if (UseTwoWheel == 0) {
        Fxlr_command = taulr/r_w;

        if (fabs(Fxlr_command) <= mu_r_disturbance*mu_s_r*Fzlr) {
            Fxlr = Fxlr_command;
        }
        else {
                if (Fxlr_command > 0) {
                    Fxlr = mu_r_disturbance*mu_s_r*Fzlr;
                }
                else {
                    Fxlr = -mu_r_disturbance*mu_s_r*Fzlr;
                }          
        }


        Fxrr_command = taurr/r_w;

        if (fabs(Fxrr_command) <= mu_r_disturbance*mu_s_r*Fzrr) {
            Fxrr = Fxrr_command;
        }
        else {
            if (Fxrr_command > 0) {
                Fxrr = mu_r_disturbance*mu_s_r*Fzrr;
            }
            else {
                Fxrr = -mu_r_disturbance*mu_s_r*Fzrr;
            }
        }



        if (Ux > SPEEDLIMIT) {
            alphalf = (atan2(Uy + a*r, Ux - c*r) - deltal);
            alpharf = (atan2(Uy + a*r, Ux + c*r) - deltar);
            alphalr = (atan2(Uy - b*r, Ux - c*r));
            alpharr = (atan2(Uy - b*r, Ux + c*r));

            Fylf = CalcNonLinearTireForce(Fzlf, Fxlf, mu_f_disturbance*mu_p_f, mu_f_disturbance*mu_s_f, Clf, alphalf);
            Fyrf = CalcNonLinearTireForce(Fzrf, Fxrf, mu_f_disturbance*mu_p_f, mu_f_disturbance*mu_s_f, Crf, alpharf);           
            Fylr = CalcNonLinearTireForce(Fzlr, Fxlr, mu_r_disturbance*mu_p_r, mu_r_disturbance*mu_s_r, Clr, alphalr);
            Fyrr = CalcNonLinearTireForce(Fzrr, Fxrr, mu_r_disturbance*mu_p_r, mu_r_disturbance*mu_s_r, Crr, alpharr);

            y[0] = atan2(x[0], x[2]);
            y[1] = x[1];
            y[2] = x[2];
    // 		y[2] = (tm+tp)*Clf*atan2(x[0],Ux) + (tm+tp)*Clf*a/Ux*x[1] - (tm+tp)*Clf*deltal;
    // 		y[3] = (tm+tp)*Crf*atan2(x[0],Ux) + (tm+tp)*Crf*a/Ux*x[1] - (tm+tp)*Crf*deltar;
            //For now, assume constant aligning moment arm, though we know this is not true for
            //nonlinear tires
            y[3] = -(tm+tp)*Fylf;
            y[4] = -(tm+tp)*Fyrf;
            y[5] = Fylf + Fyrf;
            y[6] = Fylr + Fyrr;
            y[7] = Fxlr + Fxrr;
            y[8] = 0;
        }
        else
        {
    /*		x[0] = x[1] = 0;		*/

            y[0]=y[1]=y[2]=y[3]=y[4]=y[5]=y[6]=y[7]=y[8] = 0;
        }
    }
    else {
        Fxr_command = (taulr + taurr)/r_w;
        
        if (UseDiffDrive == 1) {
            Fxlr_command = taulr/r_w;
            Fxrr_command = taurr/r_w;
            
            if (fabs(Fxlr_command) <= mu_r_disturbance*mu_s_r*Fzlr) {
                Fxlr = Fxlr_command;
            }
            else {
                if (Fxlr_command > 0) {
                    Fxlr = mu_r_disturbance*mu_s_r*Fzlr;
                }
                else {
                    Fxlr = -mu_r_disturbance*mu_s_r*Fzlr;
                }
            }
            if (fabs(Fxrr_command) <= mu_r_disturbance*mu_s_r*Fzrr) {
                Fxrr = Fxrr_command;
            }
            else {
                if (Fxrr_command > 0) {
                    Fxrr = mu_r_disturbance*mu_s_r*Fzrr;
                }
                else {
                    Fxrr = -mu_r_disturbance*mu_s_r*Fzrr;
                }
            }
            M_diff = c*(Fxrr-Fxlr);
            if (M_diff != 0) {
                printf("M_diff = %f\r\n", M_diff);
            }
        }

        if (fabs(Fxr_command) <= mu_r_disturbance*mu_s_r*Fzr) {
            Fxr = Fxr_command;
        }
        else {
            if (Fxr_command > 0) {
                Fxr = mu_r_disturbance*mu_s_r*Fzr;
            }
            else {
                Fxr = -mu_r_disturbance*mu_s_r*Fzr;
            }
        }

        if (Ux > SPEEDLIMIT) {
            alpha_f = (atan2(Uy + a*r, Ux) - delta);
            alpha_r = (atan2(Uy - b*r, Ux));

            Fyf = CalcNonLinearTireForce(Fzf, Fxf, mu_f_disturbance*mu_p_f, mu_f_disturbance*mu_s_f, Cf, alpha_f);
            Fyr = CalcNonLinearTireForce(Fzr, Fxr, mu_r_disturbance*mu_p_r, mu_r_disturbance*mu_s_r, Cr, alpha_r);
            

            y[0] = atan2(x[0], x[2]);
            y[1] = x[1];
            y[2] = x[2];
    // 		y[2] = (tm+tp)*Clf*atan2(x[0],Ux) + (tm+tp)*Clf*a/Ux*x[1] - (tm+tp)*Clf*deltal;
    // 		y[3] = (tm+tp)*Crf*atan2(x[0],Ux) + (tm+tp)*Crf*a/Ux*x[1] - (tm+tp)*Crf*deltar;
            //For now, assume constant aligning moment arm, though we know this is not true for
            //nonlinear tires
            y[3] = -(tm+tp)*0.5*Fyf;
            y[4] = -(tm+tp)*0.5*Fyf;
            y[5] = Fyf;
            y[6] = Fyr;
            y[7] = Fxr;
            y[8] = M_diff;
        }
        else
        {
    /*		x[0] = x[1] = 0;		*/

            y[0]=y[1]=y[2]=y[3]=y[4]=y[5]=y[6]=y[7]=y[8]=0;
        }
    
    }
        
}

#define MDL_DERIVATIVES  /* Change to #undef to remove function */
static void mdlDerivatives(SimStruct *S)
{
    real_T *dx = ssGetdX(S);
    real_T *x = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    
    real_T CurrentTime = ssGetT(S);
/*	 real_T Ux, Uy, r, delta;

    Ux = *uPtrs[0];

    delta = *uPtrs[1] * D2R ;
*/
    real_T delta;
    real_T deltal = *uPtrs[0];
    real_T deltar = *uPtrs[1];
    real_T taulr = *uPtrs[2];
    real_T taurr = *uPtrs[3];
    real_T mu_f_disturbance = *uPtrs[4];
    real_T mu_r_disturbance = *uPtrs[5];
    real_T UseTwoWheel = *uPtrs[6];
    real_T UseDiffDrive = *uPtrs[7];

    real_T Uy = x[0];
    real_T r  = x[1];
    real_T Ux = x[2];

    real_T alphalf, alpharf, alphalr, alpharr;
    real_T alpha_f, alpha_r;
    
    real_T Fxlf, Fxrf, Fxlr, Fxrr, Fxlr_command, Fxrr_command;
    real_T Fylf, Fyrf, Fylr, Fyrr;
    real_T Fyf, Fyr, Fxf, Fxr, Fxr_command;
    real_T M_diff;
    
    const real_T Clf = mxGetPr(ssGetSFcnParam(S, 0))[0];
    const real_T Crf = mxGetPr(ssGetSFcnParam(S, 1))[0];
    const real_T Clr = mxGetPr(ssGetSFcnParam(S, 2))[0];
    const real_T Crr = mxGetPr(ssGetSFcnParam(S, 3))[0];
    const real_T Cf = Clf + Crf;
    const real_T Cr = Clr + Crr;
    
    const real_T M = mxGetPr(ssGetSFcnParam(S, 4))[0];
    const real_T Iz = mxGetPr(ssGetSFcnParam(S, 5))[0];
    
    const real_T a = mxGetPr(ssGetSFcnParam(S, 6))[0];
    const real_T b  = mxGetPr(ssGetSFcnParam(S, 7))[0];
    const real_T L = a+b;
    const real_T c =  mxGetPr(ssGetSFcnParam(S, 8))[0];
    
    const real_T tp = mxGetPr(ssGetSFcnParam(S, 9))[0];
    const real_T tm = mxGetPr(ssGetSFcnParam(S,10))[0];
    const real_T mu_p_f = mxGetPr(ssGetSFcnParam(S,11))[0];
    const real_T mu_s_f = mxGetPr(ssGetSFcnParam(S,12))[0];
    const real_T mu_p_r = mxGetPr(ssGetSFcnParam(S, 13))[0];
    const real_T mu_s_r = mxGetPr(ssGetSFcnParam(S,14))[0];
    const real_T r_w = mxGetPr(ssGetSFcnParam(S,15))[0];
	
    const real_T Fzf = (b/L)*M*GRAVACC; //Nominal front axle normal load
    const real_T Fzr = (a/L)*M*GRAVACC; //Nominal rear axle normal load
    
    const real_T Fzlf = Fzf/2;
    const real_T Fzrf = Fzf/2;
    const real_T Fzlr = Fzr/2;
    const real_T Fzrr = Fzr/2;
    
    delta = (deltal + deltar)/2;
    
    Fxlf = 0;
    Fxrf = 0;
    Fxf = Fxlf + Fxrf;
    
    if (UseTwoWheel == 0) {
        Fxlr_command = taulr/r_w;

        if (fabs(Fxlr_command) <= mu_r_disturbance*mu_s_r*Fzlr) {
            Fxlr = Fxlr_command;
        }
        else {
            if (Fxlr_command > 0) {
                Fxlr = mu_r_disturbance*mu_s_r*Fzlr;
            }
            else {
                Fxlr = -mu_r_disturbance*mu_s_r*Fzlr;
            }          
        }

        Fxrr_command = taurr/r_w;

        if (fabs(Fxrr_command) <= mu_r_disturbance*mu_s_r*Fzrr) {
            Fxrr = Fxrr_command;
        }
        else {
            if (Fxrr_command > 0) {
                Fxrr = mu_r_disturbance*mu_s_r*Fzrr;
            }
            else {
                Fxrr = -mu_r_disturbance*mu_s_r*Fzrr;
            }          
        }

        if (Ux > SPEEDLIMIT)
        {
            alphalf = (atan2(Uy + a*r, Ux - c*r) - deltal);
            alpharf = (atan2(Uy + a*r, Ux + c*r) - deltar);
            alphalr = (atan2(Uy - b*r, Ux - c*r));
            alpharr = (atan2(Uy - b*r, Ux + c*r));

            Fylf = CalcNonLinearTireForce(Fzlf, Fxlf, mu_f_disturbance*mu_p_f, mu_f_disturbance*mu_s_f, Clf, alphalf);
            Fyrf = CalcNonLinearTireForce(Fzrf, Fxrf, mu_f_disturbance*mu_p_f, mu_f_disturbance*mu_s_f, Crf, alpharf);
            Fylr = CalcNonLinearTireForce(Fzlr, Fxlr, mu_r_disturbance*mu_p_r, mu_r_disturbance*mu_s_r, Clr, alphalr);
            Fyrr = CalcNonLinearTireForce(Fzrr, Fxrr, mu_r_disturbance*mu_p_r, mu_r_disturbance*mu_s_r, Crr, alpharr);

    		dx[0] = (Fylf*cos(deltal)+Fyrf*cos(deltar)+Fylr+Fyrr)/M - r*Ux;
    		dx[1] = (a*(Fylf*cos(deltal)+Fyrf*cos(deltar))-b*(Fylr+Fyrr)+c*(Fxrr - Fxlr+Fylf*sin(deltal)-Fyrf*sin(deltar)))/Iz;
            dx[2] = (Fxlf + Fxrf + Fxlr + Fxrr - Fylf*sin(deltal) - Fyrf*sin(deltar))/M + r*Uy;
            


//             dx[0] = (Fylf+Fyrf+Fylr+Fyrr)/M - r*Ux;
//             dx[1] = (a*(Fylf+Fyrf)-b*(Fylr+Fyrr))/Iz;
//             dx[2] = (Fxlf + Fxrf + Fxlr + Fxrr)/M + r*Uy;

    //         printf("Fylf =  %f\r\n", Fylf);

        }
        else
        {
            dx[0] = dx[1] = dx[2] = 0;
        }
    }
    else {
        Fxr_command = (taulr + taurr)/r_w;

        if (fabs(Fxr_command) <= mu_r_disturbance*mu_s_r*Fzr) {
            Fxr = Fxr_command;
        }
        else {
            if (Fxr_command > 0) {
                Fxr = mu_r_disturbance*mu_s_r*Fzr;
            }
            else {
                Fxr = -mu_r_disturbance*mu_s_r*Fzr;
            }        
        }
        if (UseDiffDrive == 1) {
            Fxlr_command = taulr/r_w;
            Fxrr_command = taurr/r_w;
            
            if (fabs(Fxlr_command) <= mu_r_disturbance*mu_s_r*Fzlr) {
                Fxlr = Fxlr_command;
            }
            else {
                if (Fxlr_command > 0) {
                    Fxlr = mu_r_disturbance*mu_s_r*Fzlr;
                }
                else {
                    Fxlr = -mu_r_disturbance*mu_s_r*Fzlr;
                }
            }
            if (fabs(Fxrr_command) <= mu_r_disturbance*mu_s_r*Fzrr) {
                Fxrr = Fxrr_command;
            }
            else {
                if (Fxrr_command > 0) {
                    Fxrr = mu_r_disturbance*mu_s_r*Fzrr;
                }
                else {
                    Fxrr = -mu_r_disturbance*mu_s_r*Fzrr;
                }
            }
            M_diff = c*(Fxrr - Fxlr);
        }


        if (Ux > SPEEDLIMIT) {
            alpha_f = (atan2(Uy + a*r, Ux) - delta);
            alpha_r = (atan2(Uy - b*r, Ux));

            Fyf = CalcNonLinearTireForce(Fzf, Fxf, mu_f_disturbance*mu_p_f, mu_f_disturbance*mu_s_f, Cf, alpha_f);
            Fyr = CalcNonLinearTireForce(Fzr, Fxr, mu_r_disturbance*mu_p_r, mu_r_disturbance*mu_s_r, Cr, alpha_r);
                    
            if (UseDiffDrive == 1){
                dx[0] = (Fyf+Fyr)/M - r*Ux;
                dx[1] = (a*Fyf-b*Fyr+M_diff)/Iz;
                dx[2] = (Fxf + Fxr - Fyf*sin(delta))/M + r*Uy;
            }
            else {
/*
                dx[0] = (Fyf+Fyr)/M - r*Ux;
                dx[1] = (a*Fyf-b*Fyr)/Iz;
                dx[2] = (Fxf + Fxr - Fyf*sin(delta))/M + r*Uy;
*/
                
                dx[0] = (Fyf*cos(delta)+Fyr)/M - r*Ux;
                dx[1] = (a*Fyf*cos(delta)-b*Fyr)/Iz;
                dx[2] = (Fxf + Fxr - Fyf*sin(delta))/M + r*Uy;
            }
            

        }
        else
        {
            dx[0] = dx[1] = dx[2] = 0;
        }
    
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
