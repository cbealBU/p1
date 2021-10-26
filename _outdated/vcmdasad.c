/* 
 *  vcmdasad.c:  A/D Device Driver for 16 Channels on VersaLogic VCM-DAS-1|2 Board
 *              jcg 18-06-99
 *
 *              Based on sfuntmpl.c: C template for a level 2 S-function.
 *
 *****************************************************************************
 *
 * Version: $Revision: 114 $
 * Last modified on: $Date: 2005-07-10 19:24:41 -0700 (Sun, 10 Jul 2005) $ 
 * by: $Author: cgadda $
 *
 */

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  vcmdasad

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */

#include  <stdlib.h>
#include "simstruc.h"

/*=========================================================================*
 * Number of S-function Parameters and macros to access from the SimStruct *
 *=========================================================================*/

#define NUM_PARAMS                  (3)
#define NUM_CHANNELS_PARAM          (ssGetSFcnParam(S,0))
#define INPUT_RANGE_PARAM           (ssGetSFcnParam(S,1))
#define SAMPLE_TIME_PARAM       (ssGetSFcnParam(S,2))

/*==================================================*
 * Macros to access the S-function parameter values *
 *==================================================*/

#define NUM_CHANNELS                ((uint_T) mxGetPr(NUM_CHANNELS_PARAM)[0])
#define INPUT_RANGE(ch)             ((int_T)  mxGetPr(INPUT_RANGE_PARAM)[ch])
#define AD_SAMPLE_TIME          ((uint_T) mxGetPr(SAMPLE_TIME_PARAM)[0])

/*=======================================*
 * Addresses for A/D Converter Registers *
 *=======================================*/

#define BASE_ADDRESS            (0x0340)
#define SELECT_ADDRESS          (BASE_ADDRESS+1)
#define ADCSTAT_ADDRESS         (BASE_ADDRESS+0)
#define ADCL_ADDRESS            (BASE_ADDRESS+4)

/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"Error encountered due to ...");
 *       return;
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable. For example the following will cause
 * unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[65536];         {ILLEGAL: to fix use "static char msg[65536];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 *
 * See matlabroot/simulink/src/sfunctmpl.doc for more details.
 */

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    /* See sfuntmpl.doc for more details on the macros below */

    ssSetNumSFcnParams(S, NUM_PARAMS);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    ssSetNumInputPorts(S, 0);

    ssSetNumOutputPorts(S, 1);
    ssSetOutputPortWidth(S, 0, NUM_CHANNELS);

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
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, AD_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);

}



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



#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START)
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
  }
#endif /*  MDL_START */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block. Generally outputs are placed in the output vector, ssGetY(S).
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T *y = ssGetOutputPortRealSignal(S,0);
    uint_T  i,j;
    int_T   raw;

    for (i = 0; i < NUM_CHANNELS; i++) {

    switch(INPUT_RANGE(i)) {                       /* Aquire and Convert Data */
        case 1:                                        /* 0-5V Range */
        case 2:                                        /* 0-10V Range */
        case 3:
            outpw(SELECT_ADDRESS,i+256);                  /* +-5V Range */
            while((inp(ADCSTAT_ADDRESS)&0x40)== 0) {
            }
            raw = inpw(ADCL_ADDRESS);
        if (raw > 32767) {                              /* Convert Two's Complement */
                raw = (raw-65536);
                y[i] = 0.0001525878907*raw;
            }
            else  {
                y[i] = 0.0001525878907*raw;
            }
            break;
        case 4:                                        /* +-10V Range */
            outpw(SELECT_ADDRESS,i+256);
            while((inp(ADCSTAT_ADDRESS)&0x40)== 0) {
            }
            raw = inpw(ADCL_ADDRESS);
        if (raw > 32767) {                              /* Convert Two's Complement */
                raw = (raw-65536);
                y[i] = 0.0003051757813*raw;
            }
            else  {
                y[i] = 0.0003051757813*raw;
            }
            break;
    }
    }
}


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



#define MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
/*    outp(0x0300,0xC0);  */
}


/*======================================================*
 * See sfuntmpl.doc for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
