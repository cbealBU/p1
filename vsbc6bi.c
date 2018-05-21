/* 
 *  vsbc6bi.c:  	Data Processing Function for BI Technologies Steering Position Sensor		
 *			        py	4/6/02
 *                  py  7/24/02 rev
 *                  py  12/14/02 rev
 *                  py  1/19/03 rev
 *                  py  1/21/03 rev
 *                  py  2/21/03 rev
 *                  py  7/6/04 rev
 *                  py  7/9/04 rev
 *
 *				     Based on sfuntmpl.c: C template for a level 2 s-function.
 */

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  vsbc6bi
#include  <stdlib.h>  
#include "simstruc.h"

/*=========================================================================*
 * Number of S-function Parameters and macros to access from the SimStruct *
 *=========================================================================*/

#define NUM_PARAMS                  	(1)
#define SAMPLE_TIME_PARAM           	(ssGetSFcnParam(S,0)) 

/*==================================================*
 * Macros to access the S-function parameter values *
 *==================================================*/

#define SAMPLE_TIME              	    ((uint_T) mxGetPr(SAMPLE_TIME_PARAM)[0])

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 * The sizes information is used by Simulink to determine the S-function
 * block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    ssSetNumInputPorts(S, 1);
    ssSetInputPortWidth(S, 0, 3);  
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    ssSetNumOutputPorts(S, 1);
    ssSetOutputPortWidth(S, 0, 1);  

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, 0);
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 * This function is used to specify the sample time(s) for your
 * S-function. You must register the same number of sample times as
 * specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

/* Function: mdlOutputs =======================================================
 * Abstract:
 * In this function, you compute the outputs of your S-function
 * block. Generally outputs are placed in the output vector, ssGetY(S).
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    InputRealPtrsType VOLTS = ssGetInputPortRealSignalPtrs(S,0);
    real_T *POSITION = ssGetOutputPortRealSignal(S,0);
    real_T P1, P2, P3, Vcc2, Vc1lo, Vc1hi, Vc2, k, L1, L2, L3, L4, L5, L6, L7, L8;

    // cold new sensor
    Vcc2 = 4.986;                           // full scale voltage P2
    Vc1hi = 2.400;                          // half scale voltage P1 (P2 hi)
    Vc1lo = 2.402;                          // half scale voltage P1 (P2 lo)

    Vc2 = Vcc2/2;                           // half full scale voltage P2
    k = 90/Vc2;                             // position to voltage ratio P2

   // cold new sensor
    Vcc2 = 4.96;                           // full scale voltage P2
    Vc1hi = 2.36;                          // half scale voltage P1 (P2 hi)
    Vc1lo = 2.38;                          // half scale voltage P1 (P2 lo)

    Vc2 = Vcc2/2;                           // half full scale voltage P2
    k = 90/Vc2;                             // position to voltage ratio P2
    
    P1 = *VOLTS[0];                         // sensor input signals
    P2 = *VOLTS[1];
    P3 = *VOLTS[2];

    L1 = 0.7;                               // P3 signal threshold levels
    L2 = 1.1;
    L3 = 1.599;
    L3 = 1.622;
    L4 = 2.028;
    L4 = 2.040;
    L5 = 2.430;
    L5 = 2.420;
    L6 = 2.935;
    L6 = 2.935;
    L7 = 3.45;
    L8 = 4.0;

    if (P3 < L1)                            // -900 to -630 degrees
    {
        if (P1 < Vc1lo && P2 < Vc2) 
        {
            POSITION[0] = -720 - k*P2;
        }
        else if (P1 > Vc1lo)
        {
            POSITION[0] = -720 + k*P2;
        }
        else
        {
            POSITION[0] = -810 - k*(P2 - Vc2);
        }
    }

    else if (P3 >= L1 && P3 < L2)           // -630 to -450 degrees
    {
        if (P1 > Vc1hi) 
        {
            POSITION[0] = -630 + k*(P2 - Vc2);
        }
        else
        {
            POSITION[0] = -450 - k*(P2 - Vc2);
        }
    }    
    
    else if (P3 >= L2 && P3 < L3)           // -450 to -270 degrees
    {
        if (P1 < Vc1lo) 
        {
            POSITION[0] = -360 - k*P2;
        }
        else
        {
            POSITION[0] = -360 + k*P2;
        }
    }

    else if (P3 >= L3 && P3 < L4)           // -270 to -90 degrees
    {
        if (P1 > Vc1hi) 
        {
            POSITION[0] = -270 + k*(P2 - Vc2);
        }
        else
        {
            POSITION[0] = -90 - k*(P2 - Vc2);
        }
    }    
    
    else if (P3 >= L4 && P3 < L5)           // -90 to 90 degrees
    {
        if (P1 < Vc1lo) 
        {
            POSITION[0] = 0 - k*P2;
        }
        else
        {
            POSITION[0] = 0 + k*P2;
        }
    }
    
    else if (P3 >= L5 && P3 < L6)           // 90 to 270 degrees
    {
        if (P1 > Vc1hi) 
        {
            POSITION[0] = 90 + k*(P2 - Vc2);
        }
        else
        {
            POSITION[0] = 270 - k*(P2 - Vc2);
        }
    } 
    
    else if (P3 >= L6 && P3 < L7)           // 270 to 450 degrees
    {
        if (P1 < Vc1lo) 
        {
            POSITION[0] = 360 - k*P2;
        }
        else
        {
            POSITION[0] = 360 + k*P2;
        }
    }
    
    else if (P3 >= L7 && P3 < L8)           // 450 to 630 degrees
    {
        if (P1 > Vc1hi) 
        {
            POSITION[0] = 450 + k*(P2 - Vc2);
        }
        else
        {
            POSITION[0] = 630 - k*(P2 - Vc2);
        }
    } 
    
    else                                    // 630 to 810 degrees
    {
        if (P1 < Vc1lo) 
        {
            POSITION[0] = 720 - k*P2;
        }
        else if (P1 > Vc1lo && P2 < Vc2)
        {
            POSITION[0] = 720 + k*P2;
        }
        else
        {
            POSITION[0] = 810 + k*(P2 - Vc2);
        }
    }

    if (POSITION[0] < -900)                 // out of range negative
    {
        POSITION[0] = -900;
    }

    if (POSITION[0] > 900)                  // out of range positive
    {
        POSITION[0] = 900;
    }        
        
    POSITION[0] = -(POSITION[0] + 104);       // sensor position offset

}


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

