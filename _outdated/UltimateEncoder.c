/* Ultimate Encoder Board Driver */
/* UltimateEncoder.c for xPC Target*/
/* JPS and CDG Sept 2003
*/

#define 	S_FUNCTION_LEVEL 	2
#undef 		S_FUNCTION_NAME
#define 	S_FUNCTION_NAME UltimateEncoder

#include 	<stddef.h>
#include 	<stdlib.h>
#include 	<string.h>

#include 	"tmwtypes.h"
#include 	"simstruc.h" 

#ifdef MATLAB_MEX_FILE
#include 	"mex.h"
#else
#include 	<windows.h>
#endif

#define 	NUMBER_OF_ARGS        	(4)
#define 	SAMP_TIME_ARG          	ssGetSFcnParam(S,0)     
#define 	NUM_CHANNELS_ARG        ssGetSFcnParam(S,1)     //First N channels?
#define     BASE_ARG                ssGetSFcnParam(S,2)     //Base address in decimal
#define     UNWRAP_ARG              ssGetSFcnParam(S,3)     //Vector with entry for each channel
                                                            //where 1 causes the code to unwrap at 65535
#define     BASE                    (uint_T)mxGetPr(BASE_ARG)[0]
#define     NUM_CHANNELS            (uint_T)mxGetPr(NUM_CHANNELS_ARG)[0]
#define     LED_ON_NUM               1/(2*mxGetPr(SAMP_TIME_ARG)[0])

#define 	SAMP_TIME_IND           (0)

#define 	NO_I_WORKS             	(0)
#define     U(element)          (*uPtrs[element])  /* Pointer to Input Port0 */

#define 	NO_R_WORKS              (0)

static char_T msg[256];

static void mdlInitializeSizes(SimStruct *S)
{
	int_T num_channels, i;
		
  	ssSetNumSFcnParams(S, NUMBER_OF_ARGS);  
  	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) 
	{
    	sprintf(msg,"Wrong number of input arguments passed.\n%d arguments are expected\n",NUMBER_OF_ARGS);
        ssSetErrorStatus(S,msg);
        return;
    }
	
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, NUM_CHANNELS*2+2);      //Previous Value for each channel, 
                                                  //plus wraps for each and LED Samples since flash and LED on
	
	ssSetNumOutputPorts(S, NUM_CHANNELS);
	for (i=0;i<NUM_CHANNELS;i++) 
		ssSetOutputPortWidth(S, i, 1);
	
    ssSetNumInputPorts(S, 1);
    ssSetInputPortWidth(S, 0, NUM_CHANNELS);
	
    ssSetNumSampleTimes(S, 1);
	
    ssSetNumRWork(S, NO_R_WORKS);
    ssSetNumIWork(S, NO_I_WORKS);
    ssSetNumPWork(S, 0);
	
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
	
	
	ssSetSFcnParamNotTunable(S,0);
	ssSetSFcnParamNotTunable(S,1);
	
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE | SS_OPTION_PLACE_ASAP);
	
}

static void mdlInitializeSampleTimes(SimStruct *S)
{    
   	ssSetSampleTime(S, 0, mxGetPr(SAMP_TIME_ARG)[SAMP_TIME_IND]);

    if (mxGetN((SAMP_TIME_ARG))==1) 
		ssSetOffsetTime(S, 0, 0.0);
	else
		ssSetOffsetTime(S, 0, mxGetPr(SAMP_TIME_ARG)[1]);
}

static void mdlInitializeConditions(SimStruct *S)
{
    real_T *state = ssGetRealDiscStates(S);
	int_T i;
	
	/* initializing states */
    for (i = 0; i < NUM_CHANNELS; i++)
        state[i] = 0;
}


#define MDL_START 
static void mdlStart(SimStruct *S)
{
	
#ifndef MATLAB_MEX_FILE
	
	int_T channel, tmp, i;
	int_T low, high;
	real_T          *state = ssGetRealDiscStates(S);

	if((inp(BASE+3)&0xff)!=0x90)
		outp(BASE+3, 0x90);       //configure ports on 82c55 if not already configd
	
	//Initially set to not reset on index pulse, LED Off
	outp(BASE+2, 0x00);       //set b0 to HI to reset on index pulse
	outp(BASE+1,0xff);        //set sel low, oe low for safety
	
	
	//for each channel initialize state to current count
	//B0 is select, B1 is OE1, etc
	/*for (i=0;i<NUM_CHANNELS;i++) 
	{
        outp(BASE+1,0xff-1-(2<<i));        //set sel low, oe low, all other oe high
		high=inp(BASE)&0xff;          //read high byte
		outp(BASE+1,0xff-(2<<i));        //set sel hi, oe low, all other oe high
		low=inp(BASE)&0xff;        //read low byte
		outp(BASE+1,0xff);        //set all oe hi, sel hi
		state[i]=((high)*256+low)*0;
	}
	*/
#endif
	
}	

static void mdlOutputs(SimStruct *S, int_T tid)
{
	
#ifndef MATLAB_MEX_FILE
	
	int_T 			i, temp;
	int_T			channel;
	real_T  		*y;
	int_T          low,high;
	real_T          count;
	real_T          prevtemp;
    real_T          *state = ssGetRealDiscStates(S);
	int_T           ledtemp=0;
	int_T           ResetTemp=0;
	int_T           debugtemp=0;
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
	
	//Prepare to set reset bit for each channel later
	//Input of 1 sets it to reset on index
	for (i=0;i<NUM_CHANNELS;i++) 
	{
        if(U(i)>.5)
			ResetTemp=ResetTemp+(1<<i);
        
        //Read count data
        y=ssGetOutputPortSignal(S,i);
        outp(BASE+1,0xff-1-(2<<i));        //set sel low, oe low, all other oe high
        high=inp(BASE)&0xff;          //read high byte
        outp(BASE+1,0xff-(2<<i));        //set sel hi, oe low, all other oe high
        low=inp(BASE)&0xff;        //read low byte
        outp(BASE+1,0xff);        //set all oe hi, sel hi
        count=(high)*256+low;
        
        if((uint_T)mxGetPr(UNWRAP_ARG)[i])
		{
			prevtemp=state[i];
			if((count<prevtemp)&((count-prevtemp)<(-32768)))
				state[NUM_CHANNELS+i]=state[NUM_CHANNELS+i]+1;;

			if((count>prevtemp)&((count-prevtemp)>(32768)))
				state[NUM_CHANNELS+i]=state[NUM_CHANNELS+i]-1;
		}
		
        y[0]=count+65536*state[NUM_CHANNELS+i];
        //debugtemp=0xff-1-(2<<i);
        //y[0]=debugtemp;
        debugtemp=-1;
        state[i]=count;
        
	}   
	
	//Reset and LED
	state[2*NUM_CHANNELS]=state[2*NUM_CHANNELS]+1;      //increment time led has been in current state
	if(state[2*NUM_CHANNELS]>LED_ON_NUM-1)             //if on/off for too long
    {
	    state[2*NUM_CHANNELS+1]=(state[2*NUM_CHANNELS+1]==0);
        state[2*NUM_CHANNELS]=0;                            //reset on time
	}
	ledtemp=128*state[2*NUM_CHANNELS+1]; 
	outp(BASE+2, ledtemp+ResetTemp);       //set c to HI to reset on index pulse
	
	
	
#endif
	
}

static void mdlTerminate(SimStruct *S)
{
}

#ifdef	MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif

