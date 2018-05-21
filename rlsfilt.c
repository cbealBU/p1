/*
 * File: rlsfilt.c
 *
 * Author: Christopher Gadda
 * (C)2004 Dynamic Design Laboratory -- Stanford University
 *
 * $Revision: 150 $ $Date: 2005-08-11 18:03:27 -0700 (Thu, 11 Aug 2005) $
 *
 * This file implements an s-function for use with Simulink.  The filter accepts
 * current, voltage and angular rate information for a motor and uses a
 * Kalman filter to estimate the motor constant and resistance.  It assumes
 * that the inputs are specified in SI units, and the estimates it generates
 * will also be in SI units.
 * 	
 * This filter uses a recursive least-squares parameter estimation technique.
 * The data is exponentially weighted so that older data has less of an effect
 * on the estimate than newer data.  The exponential "forgetting factor" is
 * passed into this function as a parameter.
 *
 * This file is based (very loosely) on $MATLAB/simulink/src/sfuntmpl_basic.c
 */

#define S_FUNCTION_NAME  rlsfilt
#define S_FUNCTION_LEVEL 2

#include <math.h>
#include "simstruc.h"

#define NUMOFPARAMS 1
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

    
//    real_T       *y = ssGetOutputPortSignal(S,0);



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

static void mdlUpdate(SimStruct *S, int_T tid)
{
    const real_T volts=**ssGetInputPortRealSignalPtrs(S,0);
    const real_T amps=**ssGetInputPortRealSignalPtrs(S,1);
    const real_T omega=**ssGetInputPortRealSignalPtrs(S,2);
    const real_T lambda=*mxGetPr(ssGetSFcnParam(S,0));
    real_T *x=ssGetRealDiscStates(S);

    real_T *P,*w;
    real_T k[2],Py[2];
    real_T yPy,yw,alpha;
    real_T y[2];
    real_T poolkey[3];
    
    int i;
    	
	y[0]=amps;
    y[1]=omega;
    w=x;
    P=&(x[2]);
    
	if((fabs(y[0])>.01)&&(fabs(y[1])>.01))
	{
		SYM_MAT_MULT_221(P,y,Py);
		yPy=DOT(y,Py);
		k[0]=Py[0]/(yPy+lambda);
		k[1]=Py[1]/(yPy+lambda);

		alpha=volts-DOT(y,w);
	   
		w[0]+=k[0]*alpha;
		w[1]+=k[1]*alpha;
		
		SYM_MAT_MULT_212(Py,k,poolkey);
		for(i=0;i<3;i++)
			P[i]=P[i]/lambda-poolkey[i]/lambda;
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
