/*
 * File: bic_model.c
 * Authors: Jihan Ryu, Paul Yih
 *
 * $Revision: 484 $ $Date: 2007-07-12 15:45:55 -0700 (Thu, 12 Jul 2007) $
 *
 * Abstract:  bicycle model of a car
 *	The input is
 *		*uPtrs[0] = speed in m/s
 *		*uPtrs[1] = steering angle in RADIANS
 * The output is
 *		y[0] = slip angle at CG in RADIANS
 *		y[1] = yaw rate (rad/s)
 *     
 * See matlabroot/simulink/src/sfuntmpl_doc.c for more details
 */

#define S_FUNCTION_NAME  bic_model
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>

#define NUMOFINPUTS 	7
#define NUMOFOUTPUTS	4
#define NUMOFSTATES	2
#define NUMOFPARAMS 11

#define PI     	3.14159 
#define D2R			(1) /*(PI/180)*/	/* degree to radian */
#define R2D			(1) /*(180/PI)*/	/* radian to degree */

#define SPEEDLIMIT (0.2)

/* Comment out this line to use nonlinear model. */
#define atan2(x,y) ((x)/(y))

/*================*
 * Car Parameters *
 *================*/
/*static real_T M	= 1698.0;
static real_T Iz = 2000.0;

static real_T Cfl = 40000.0;
static real_T Cfr = 40000.0;
static real_T Crl = 50000.0;
static real_T Crr = 50000.0;

static real_T a	= 1.33;
static real_T b = 1.17;

static real_T Cfl = 60000.0;
static real_T Cfr = 60000.0;
static real_T Crl = 60000.0;
static real_T Crr = 60000.0;
*/

//static real_T tp=0.0230;

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
    for (i = 0; i < NUMOFSTATES; i++) { 
        x0[i] = 0;
    }
    
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T *x = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    real_T *y = ssGetOutputPortRealSignal(S,0);
    real_T Ux = *uPtrs[0];
    real_T deltal = *uPtrs[1];
    real_T deltar = *uPtrs[2];
    real_T deltarl = *uPtrs[3];
    real_T deltarr = *uPtrs[4];
    
    const real_T Cfl = mxGetPr(ssGetSFcnParam(S, 0))[0];
    const real_T Cfr = mxGetPr(ssGetSFcnParam(S, 1))[0];
    const real_T Crl = mxGetPr(ssGetSFcnParam(S, 2))[0];
    const real_T Crr = mxGetPr(ssGetSFcnParam(S, 3))[0];
    const real_T M = mxGetPr(ssGetSFcnParam(S, 4))[0];
    const real_T Iz = mxGetPr(ssGetSFcnParam(S, 5))[0];
    const real_T a = mxGetPr(ssGetSFcnParam(S, 6))[0];
    const real_T b  = mxGetPr(ssGetSFcnParam(S, 7))[0];
    const real_T c =  mxGetPr(ssGetSFcnParam(S, 8))[0];
    const real_T tp = mxGetPr(ssGetSFcnParam(S, 9))[0];
    const real_T tm = mxGetPr(ssGetSFcnParam(S,10))[0];
       
/*
c31=(tm+tp)*Cfl*a/Ux;
c32=(tm+tp)*Cfl;
c41=(tm+tp)*Cfr*a/Ux;
c42=(tm+tp)*Cfr;

d31=-(tm+tp)*Cfl;
d42=-(tm+tp)*Cfr;
*/

if (Ux > SPEEDLIMIT) {
		y[0] = atan2(x[0], Ux);
		y[1] = x[1];
		y[2] = (tm+tp)*Cfl*atan2(x[0],Ux) + (tm+tp)*Cfl*a/Ux*x[1] - (tm+tp)*Cfl*deltal;
		y[3] = (tm+tp)*Cfr*atan2(x[0],Ux) + (tm+tp)*Cfr*a/Ux*x[1] - (tm+tp)*Cfr*deltar;
	}
	else
	{
/*		x[0] = x[1] = 0;		*/

		y[0]=y[1]=y[2]=y[3]=0;
	}
}

#define MDL_DERIVATIVES  /* Change to #undef to remove function */
static void mdlDerivatives(SimStruct *S)
{
    real_T *dx = ssGetdX(S);
    real_T *x = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
/*	 real_T Ux, Uy, r, delta;

    Ux = *uPtrs[0];

    delta = *uPtrs[1] * D2R ;
*/
    real_T Ux = *uPtrs[0];
    real_T deltal = *uPtrs[1];
    real_T deltar = *uPtrs[2];
    real_T deltarl = *uPtrs[3];
    real_T deltarr = *uPtrs[4];
    real_T modss = *uPtrs[5];
    real_T modyr = *uPtrs[6];

    real_T Uy = x[0];
    real_T r  = x[1];

    real_T forcefl;
	real_T forcefr;
	real_T forcerl;
	real_T forcerr;
    
    const real_T Cfl = mxGetPr(ssGetSFcnParam(S, 0))[0];
    const real_T Cfr = mxGetPr(ssGetSFcnParam(S, 1))[0];
    const real_T Crl = mxGetPr(ssGetSFcnParam(S, 2))[0];
    const real_T Crr = mxGetPr(ssGetSFcnParam(S, 3))[0];
    const real_T M = mxGetPr(ssGetSFcnParam(S, 4))[0];
    const real_T Iz = mxGetPr(ssGetSFcnParam(S, 5))[0];
    const real_T a = mxGetPr(ssGetSFcnParam(S, 6))[0];
    const real_T b  = mxGetPr(ssGetSFcnParam(S, 7))[0];
    const real_T c =  mxGetPr(ssGetSFcnParam(S, 8))[0];
    const real_T tp = mxGetPr(ssGetSFcnParam(S, 9))[0];
    const real_T tm = mxGetPr(ssGetSFcnParam(S,10))[0];
	
	if (Ux > SPEEDLIMIT)
	{
		forcefl=-Cfl*(atan2(Uy + a*r, Ux - c*r) - deltal);
		forcefr=-Cfr*(atan2(Uy + a*r, Ux + c*r) - deltar);
		forcerl=-Crl*(atan2(Uy - b*r, Ux - c*r) - deltarl);
		forcerr=-Crr*(atan2(Uy - b*r, Ux + c*r) - deltarr);

		dx[0] = modss+(forcefl+forcefr+forcerl+forcerr)/M - r*Ux;
		dx[1] = modyr+(a*(forcefl+forcefr)-b*(forcerl+forcerr))/Iz;
	}
    else
	{
        dx[0] = dx[1] = 0;
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
