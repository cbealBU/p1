/* 
 *		vsbc6ad.c:  A/D Device Driver for 8 Channels on VersaLogic VSBC-6 Board
 *		jcg 18-06-99
 *
 *		Based on sfuntmpl.c: C template for a level 2 S-function.
 */

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  vsbc6ad
#include  <stdlib.h>  
#include "simstruc.h"

/*=========================================================================*
 * Number of S-function Parameters and macros to access from the SimStruct *
 *=========================================================================*/

#define NUM_PARAMS                  	(3)
#define NUM_CHANNELS_PARAM          	(ssGetSFcnParam(S,0))
#define INPUT_RANGE_PARAM           	(ssGetSFcnParam(S,1))
#define SAMPLE_TIME_PARAM	      	    (ssGetSFcnParam(S,2)) 

/*==================================================*
 * Macros to access the S-function parameter values *
 *==================================================*/

#define NUM_CHANNELS                	((uint_T) mxGetPr(NUM_CHANNELS_PARAM)[0])
#define INPUT_RANGE(ch)             	((int_T)  mxGetPr(INPUT_RANGE_PARAM)[ch])
#define AD_SAMPLE_TIME		      	    ((uint_T) mxGetPr(SAMPLE_TIME_PARAM)[0])

/*=======================================*
 * Addresses for A/D Converter Registers *
 *=======================================*/

#define ACR_ADDRESS		            	(0xE4)
#define DCAS_ADDRESS		            (0xE2)
#define ADC_ADDRESS		      	        (0xE4)

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
 * This function is used to specify the sample time(s) for your
 * S-function. You must register the same number of sample times as
 * specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, AD_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

/* Function: mdlOutputs =======================================================
 * Abstract:
 * In this function, you compute the outputs of your S-function
 * block. Generally outputs are placed in the output vector, ssGetY(S).
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T *y = ssGetOutputPortRealSignal(S,0);
    uint_T  i,j;
    int_T   raw;
   
    for (i = 0; i < NUM_CHANNELS; i++) {         
 
    switch(INPUT_RANGE(i)) {                     	/* Aquire and Convert Data */
        case 1:                                    	/* 0-5V Range */
            outp(ACR_ADDRESS,0x40+i);               /* Trigger Conversion */  
            while((inp(DCAS_ADDRESS)&0x04)== 0) { 	/* Wait Until Completed */
            } 
            raw = inpw(ADC_ADDRESS);  		        /* Input 12 bit Value & Scale */ 
            y[i] = raw*0.001220703;
            break;
        case 2:                                 	/* 0-10V Range */
            outp(ACR_ADDRESS,0x50+i);               
            while((inp(DCAS_ADDRESS)&0x04)== 0) {
            } 
            raw = inpw(ADC_ADDRESS); 
            y[i] = raw*0.002441406;
	        break;  
        case 3:
            outp(ACR_ADDRESS,0x48+i);           	/* +-5V Range */               
            while((inp(DCAS_ADDRESS)&0x04)== 0) {
            } 
            raw = inpw(ADC_ADDRESS); 
	    if (raw > 2047) {             	      	    /* Convert Two's Complement */
                raw = (raw-65536);
                y[i] = 0.002441406*raw; 
            }
            else  {
                y[i] = 0.002441406*raw; 
            }
            break;        
        case 4:          				            /* +-10V Range */ 
            outp(ACR_ADDRESS,0x58+i);               
            while((inp(DCAS_ADDRESS)&0x04)== 0) {
            } 
            raw = inpw(ADC_ADDRESS); 
	    if (raw > 2047) {                          	/* Convert Two's Complement */
                raw = (raw-65536);
                y[i] = 0.004882813*raw; 
            }
            else  {
                y[i] = 0.004882813*raw;
            } 
            break;  
    }
    } 
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


