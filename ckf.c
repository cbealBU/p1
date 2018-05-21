/*
 * File: ckf.c  (Crazy Kalman Filter)
 *
 * Author: Christopher Gadda
 * (C)2005 Dynamic Design Laboratory -- Stanford University
 *
 * $Revision: 150 $ $Date: 2005-08-11 18:03:27 -0700 (Thu, 11 Aug 2005) $
 *
 * This file implements an s-function for use with Simulink.  The filter accepts
 * current, voltage and angular rate information for a motor and uses a
 * Kalman filter to estimate the motor constant and resistance.  It assumes
 * that the inputs are specified in SI units, and the estimates it generates
 * will also be in SI units.
 *
 * The way this filter works is by assuming that the resistance and motor
 * constant are the states of a discrete-time linear, time-varying system.
 * The dynamics of this system are assumed to be constant (A=I) and the
 * measurement matrix (C) is time-varying, depending on the instaneous current
 * and angular velocity. 
 *
 * This file is based (very loosely) on $MATLAB/simulink/src/sfuntmpl_basic.c
 */

#define S_FUNCTION_NAME  ckf
#define S_FUNCTION_LEVEL 2

#include <math.h>
#include "simstruc.h"

#define NUMOFPARAMS 2
#define NUMOFSTATES	5
#define NUMOFINPUTS 3
#define NUMOFOUTPUTS 3

static int INPUTWIDTHS[]={1,1,1};
static int OUTPUTWIDTHS[]={1,1,3};


static char msg[80];

/*====================*
 * S-function methods *
 *====================*/
static void mdlInitializeSizes(SimStruct *S)
{
	int i;
    
    ssSetNumSFcnParams(S, NUMOFPARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        ssSetErrorStatus(S,"Incorrect number of parameters.");
        return;
    }
    
    ssSetNumDiscStates(S, NUMOFSTATES);
    ssSetNumContStates(S, 0);

    if (!ssSetNumInputPorts(S, NUMOFINPUTS))
    	return;
    
    for(i=0;i<NUMOFINPUTS;i++)
    {
        ssSetInputPortWidth(S, i, INPUTWIDTHS[i]);
        ssSetInputPortDirectFeedThrough(S, i, 0);
    }
    
    if (!ssSetNumOutputPorts(S, NUMOFOUTPUTS))
    	return;
    for(i=0;i<NUMOFOUTPUTS;i++)
    {
        ssSetOutputPortWidth(S, i, OUTPUTWIDTHS[i]);
    }
    
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
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS
#ifdef MDL_INITIALIZE_CONDITIONS
/* Function: mdlInitializeConditions ========================================= */
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *x=ssGetRealDiscStates(S);

    x[0]=x[1]=0;  /* Initial estimates of zero.  This could be fixed up! */
    x[2]=x[4]=1;  /* Initialize P to the identity matrix. */
    x[3]=0;
}
#endif /* MDL_INITIALIZE_CONDITIONS */


/* Function: mdlOutputs ======================================================= */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T *Rest=ssGetOutputPortRealSignal(S,0);
    real_T *kest=ssGetOutputPortRealSignal(S,1);
    real_T *P=ssGetOutputPortRealSignal(S,2);
    const real_T *x=ssGetRealDiscStates(S);
    int i;
    
    *Rest=x[0];
    *kest=x[1];

    for(i=0;i<OUTPUTWIDTHS[2];i++)
        P[i]=x[i+2];
}

    

#define MDL_UPDATE 
#ifdef MDL_UPDATE
/* Function: mdlUpdate ======================================================
 * Abstract:
 *    This function is called once for every major integration time step.
 *    Discrete states are typically updated here, but this function is useful
 *    for performing any tasks that should only take place once per
 *    integration step.
 */

#define DOT(x,y) ((x)[0]*(y)[0]+(x)[1]*(y)[1])
#define SYM_MAT_MULT_221(x,y,z) (z)[0]=DOT(x,y);(z)[1]=DOT(x+1,y)
#define SYM_MAT_MULT_212(x,y,z) (z)[0]=(x)[0]*(y)[0];(z)[1]=(x)[1]*(y)[0];(z)[2]=(x)[1]*(y)[1]

//static real_T wmat[] = { 0.0001, 0.0, 0.0001 };
//#define W (wmat)

static void mdlUpdate(SimStruct *S, int_T tid)
{
    const real_T volts=**ssGetInputPortRealSignalPtrs(S,0);
    const real_T amps=**ssGetInputPortRealSignalPtrs(S,1);
    const real_T omega=**ssGetInputPortRealSignalPtrs(S,2);
    const real_T V=*mxGetPr(ssGetSFcnParam(S,0));
    const real_T *W=mxGetPr(ssGetSFcnParam(S,1));
    real_T *x=ssGetRealDiscStates(S);

    real_T *P;
    real_T L[2],C[2];
	real_T inno,tmp,PC[2],tmp2[3];
	
    int i;

//	printf("V=%g and W=[%g, %g, %g].         ",(real_T)V,W[0],W[1],W[2]);
	
	C[0]=amps;
    C[1]=omega;

    P=&(x[2]);
    
	if((fabs(C[0])>.01)&&(fabs(C[1])>.01))
	{
		/* First create L, the feedback gains. */
		SYM_MAT_MULT_221(P,C,PC);
		tmp=(DOT(C,PC)+V);
		L[0]=PC[0]/tmp;
		L[1]=PC[1]/tmp;
		
		/* Now compute the updates to our estimates */
		inno=volts-DOT(C,x);
		x[0]+=L[0]*inno;
		x[1]+=L[1]*inno;
		
		/* Now update the covariance matrix. */
		SYM_MAT_MULT_212(L,PC,tmp2);
		for(i=0;i<3;i++)
			P[i]+=W[i]-tmp2[i];
	}
}        
#endif /* MDL_UPDATE */

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
