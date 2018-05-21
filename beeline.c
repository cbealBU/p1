// beeline.c - RS-232 driver and text parser for Novatel Beeline system.
//
// This code appears to be based on an example RS-232 driver supplied by
// The Mathworks, called rs232_setup.c
//
// It's unclear who in the DDL originally wrote this code.
//
// $Revision: 133 $ $Date: 2005-07-19 21:29:18 -0700 (Tue, 19 Jul 2005) $


/* Outputs are
	0	UTC (attitude), 
	1	yaw, 
	2	roll, 
	3	status (attitude),
	4	UTC (velocity), 
	5	latency,
	6	horizontal speed, 
	7	velocity direction, 
	8	vertical speed, 
	9	status (velocity)
*/
/* Status codes are as follows:
  Attitude:
	0 - no attitude
	1 - good 2D floating attitude solution
	2 - good 2D integer attitude solution
	3 - floating ambiguity attitude solution with line bias known
	4 - fixed ambiguity attitude solution with line bias known

    While those descriptions may not be all that clear, that's all there is
	about them in the user's manual.  In summary, 4 is good, anything else is
	bad.
	
  Velocity:
    0 - velocity computed from differentially corrected carrier phase data
	1 - velocity computed from differentially corrected Doppler data
	2 - old velocity from differentially corrected phase or Doppler
	3 - velocity from single point computations
	4 - old velocity from single point computations
	5 - invalid velocity
*/

#undef S_FUNCTION_NAME
#define S_FUNCTION_NAME beeline

#include <stddef.h>
#include <stdlib.h>

#include "tmwtypes.h"
#include "simstruc.h" 

#ifdef MATLAB_MEX_FILE
#include "mex.h"
#else
#include <windows.h>  // Strange as it may seem, you really do need this!
#include <string.h>
#include "rs232_xpcimport.h"
#endif

// We need to define some COM port symbols
#define COM_PORT				0  // 0 - COM1, 1 - COM2, etc.
#define BAUD_RATE				57600
#define DATA_BITS				8
#define STOP_BITS				1
#define THE_PARITY  			0x100  // {0x100, 0x200, or 0x400}
#define PROTOCOL				0
#define SEND_BUFFER				1024
#define RECEIVE_BUFFER			1024
#define TX_SHIFT_EMPTY          0x40

// These are used in place of discrete states to store the previous value for a
// particular output, in between updates from the Beeline.
#define NO_R_WORKS                      (10)

#define NO_I_WORKS                      (0)
#define	NUMBER_OF_ARGS          		(1)

// This external variable keeps track of which com port are open.  I'm not sure
// if it's required or not.
extern int rs232ports[]; 

void sendString(const char *string)
{
#ifndef MATLAB_MEX_FILE
	rl32eSendBlock(COM_PORT, (void *)string, strlen(string));
#endif
}

void sendStringAndWait(const char *string)
{
#ifndef MATLAB_MEX_FILE
	rl32eSendBlock(COM_PORT, (void *)string, strlen(string));
	while (!(rl32eLineStatus(COM_PORT) & TX_SHIFT_EMPTY));
#endif
}


static void mdlInitializeSizes(SimStruct *S)
{
#ifndef MATLAB_MEX_FILE
// This line is VERY IMPORTANT.  It's a little bit of spaghetti code from our
// good friends at The Mathworks, but if you remove it, this driver will crash
// xPC as soon as you try to download your model.
#include "rs232_xpcimport.c"
#endif

#ifdef MATLAB_MEX_FILE
        ssSetNumSFcnParams(S, NUMBER_OF_ARGS);  /* Number of expected parameters */
        if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
        {
            ssSetErrorStatus(S,"Wrong number of parameters passed.\nThere is only one parameter: sample time in seconds.\n");
            return;
        }
#endif

                
        /* Set-up size information */
        ssSetNumContStates(S, 0);
        ssSetNumDiscStates(S, 0);
        ssSetNumOutputs(S, 10);
        ssSetNumInputs(S, 0);
        ssSetDirectFeedThrough(S, 0); /* Direct dependency on inputs */
        ssSetNumSampleTimes(S,0);
        ssSetNumInputArgs(S, NUMBER_OF_ARGS);
        ssSetNumIWork(S, NO_I_WORKS); 
        ssSetNumRWork(S, NO_R_WORKS);
        ssSetNumPWork(S, 0);   /* number of pointer work vector elements*/
        ssSetNumModes(S, 0);   /* number of mode work vector elements   */
        ssSetNumNonsampledZCs(S, 0);   /* number of nonsampled zero crossings   */
        ssSetOptions(S, 0);   /* general options (SS_OPTION_xx)        */
        
}
 
/* Function to initialize sample times */
static void mdlInitializeSampleTimes(SimStruct *S)
{
	ssSetSampleTime(S, 0, mxGetPr(ssGetSFcnParam(S,0))[0]);
	ssSetOffsetTime(S, 0, 0.0);
}
 

static void mdlInitializeConditions(real_T *x0, SimStruct *S)
{
	int i;
	
#ifndef MATLAB_MEX_FILE
// First we initialize the COM port...
	rl32eInitCOMPort(	COM_PORT,
						DefaultCOMIOBase[COM_PORT],
						DefaultCOMIRQ[COM_PORT],
						BAUD_RATE,
						THE_PARITY,
						STOP_BITS, //(int_T)mxGetPr(STOPBIT_ARG)[0],
						DATA_BITS, //9-((int_T)mxGetPr(DATABIT_ARG)[0]),
						RECEIVE_BUFFER, //(int_T)mxGetPr(RECBUF_ARG)[0],
						SEND_BUFFER,  //(int_T)mxGetPr(SENDBUF_ARG)[0],
						PROTOCOL); //((int_T)mxGetPr(PROTOCOL_ARG)[0])-1);
	rs232ports[COM_PORT]=1;

// Then we initialize the Beeline device itself...
// Turn on RTS handshaking
    sendStringAndWait("com1_rts toggle,high,2,0 \r\n");
	
// Turn off any logs that were on (should be none)
	sendStringAndWait("unlogall \r\n");          

	sendStringAndWait("unlog RCCA \r\n");

// "Pitch" (roll) constrained to +-5degrees at startup, now set back to +-90 centered around 0        
// Unconstraining this here creates the problem where you can't run this code
// to see if the beeline has locked up until after it has locked up.  Now we
// wait until the attitude status == 4 before unconstraining the roll angle.
//        strcpy( tmp, "attmode constrain_pitch 0 90 \r\n");
//        rl32eSendBlock(COM_PORT, (void *) tmp, strlen(tmp));
//        while (!(rl32eLineStatus(COM_PORT) & TX_SHIFT_EMPTY));       

// Log ASCII attitude data at 5Hz   
	sendStringAndWait("log com1 ATTA ontime 0.2 \r\n");

// Log ASCII velocity data at 5Hz   
	sendStringAndWait("log com1 VLHA ontime 0.2 \r\n");
#endif

// Now we need to initialize our RWorks...
// We just set them all to zero.
	for(i=0;i<NO_R_WORKS;i++)
        ssSetRWorkValue(S,i,0);
}

/* Function to compute outputs */
static void mdlOutputs(real_T *y, const real_T *x, const real_T *u, SimStruct *S, int_T tid)
{

#ifndef MATLAB_MEX_FILE
    int i,n, j, k, received, fieldnum, index, data, status, output;
	char *formatsend;
	char c;
	char tmp[256], logtype[8]; 
	char in[256];
	double out;
                
	k = 0;
	if (!rs232ports[COM_PORT]) {
			printf("        RS232 I/O-send Error: choosen COM-port not initialized\n");
			return;
	}

	fieldnum = 1;
	index = 0;
	data = 0;

	status = inp(0X03FE)&0x10;

	k=rl32eReceiveBufferCount(COM_PORT);
     
	if ((status == 0) && (k != 0))
	{
     /* Read each data field and save those of interest */ 

		in[0] = '\0';

		for (i=0;i<k;i++)
		{
			c = rl32eReceiveChar(COM_PORT);
			if (c == '$')
			{
				i++;
				break;
			}
		}

		k -= i;

		for (i=0;i<k;i++)
		{
			c = rl32eReceiveChar(COM_PORT);
			in[index++] = c;
			in[index] = '\0';
			if (c == 10)
				break;
		} 
		
		if (in[0]=='A') {         
			y[4] = ssGetRWorkValue(S,4);
			y[5] = ssGetRWorkValue(S,5);
			y[6] = ssGetRWorkValue(S,6);
			y[7] = ssGetRWorkValue(S,7);
			y[8] = ssGetRWorkValue(S,8);
			y[9] = ssGetRWorkValue(S,9);
		}
		else if (in[0]=='V') {        
			y[0] = ssGetRWorkValue(S,0);
			y[1] = ssGetRWorkValue(S,1);
			y[2] = ssGetRWorkValue(S,2);
			y[3] = ssGetRWorkValue(S,3);
		}
		else {
			y[0] = ssGetRWorkValue(S,0);
			y[1] = ssGetRWorkValue(S,1);
			y[2] = ssGetRWorkValue(S,2);
			y[3] = ssGetRWorkValue(S,3);
			y[4] = ssGetRWorkValue(S,4);
			y[5] = ssGetRWorkValue(S,5);
			y[6] = ssGetRWorkValue(S,6);
			y[7] = ssGetRWorkValue(S,7);
			y[8] = ssGetRWorkValue(S,8);
			y[9] = ssGetRWorkValue(S,9);
		}
		
		i = 0;
		j = 0;
		
		while (i<index) {
			if ((in[i]!=',')&&(in[i]!='*'))  {
				tmp[j] = in[i];
				tmp[j+1] = '\0';
			}
			else {
                
				if (in[0]=='A') {
					
					switch (fieldnum) {
						case 3:
							sscanf(tmp,"%lf", &out);
							y[0] = (real_T) out;
							ssSetRWorkValue(S,0,y[0]);
							break;
						case 7:
							sscanf(tmp,"%lf",&out);
							y[1]= (real_T) out;
							ssSetRWorkValue(S,1,y[1]);
							break;
						case 8:
							sscanf(tmp,"%lf", &out);
							y[2]= (real_T) out;
							ssSetRWorkValue(S,2,y[2]);
							break;
						case 13:
							sscanf(tmp,"%d", &n);
							y[3] = (real_T) n;
							ssSetRWorkValue(S,3,y[3]);
							// Here's where we check to see if it's time
                            // to unconstrain the roll angle.
                            if (y[3]==4)
								sendString("attmode constrain_pitch 0 90 \r\n");
							break;
						default:
							break;
					}  
					
					fieldnum++;
					j=-1;
					
				} // if('A')
				else if (in[0]=='V')
				{
					switch (fieldnum)
					{
						case 3:
							sscanf(tmp,"%lf",&out);
							y[4]= (real_T) out;
							ssSetRWorkValue(S,4,y[4]);
							break;
						case 4:
							sscanf(tmp,"%lf",&out);
							y[5]= (real_T) out;
							ssSetRWorkValue(S,5,y[5]);
							break;
						case 6:
							sscanf(tmp,"%lf", &out);
							y[6]= (real_T) out;
							ssSetRWorkValue(S,6,y[6]);
							break;
						case 7:
							sscanf(tmp,"%lf", &out);
							y[7] = (real_T) out;
							ssSetRWorkValue(S,7,y[7]);
							break;
						case 8:
							sscanf(tmp,"%lf", &out);
							y[8] = (real_T) out;
							ssSetRWorkValue(S,8,y[8]);
							break;
						case 9:
							sscanf(tmp,"%d", &n);
							y[9] = (real_T) n;
							ssSetRWorkValue(S,9,y[9]);
							break;
						default:
							break;
					} 
					
					fieldnum++;
					j=-1;
					
				} // else if ('V')  
				else
				{
					y[3]= (real_T) 0;
					ssSetRWorkValue(S,3,y[3]);
					y[9]= (real_T) 0; // This seems wrong to me... CDG 6/15/05
					ssSetRWorkValue(S,9,y[9]);
				}
			} 
			
			i++;   
			j++;
		}
	}
	else
	{
		y[0] = ssGetRWorkValue(S,0);
		y[1] = ssGetRWorkValue(S,1);
		y[2] = ssGetRWorkValue(S,2);
		y[3] = ssGetRWorkValue(S,3);
		y[4] = ssGetRWorkValue(S,4);
		y[5] = ssGetRWorkValue(S,5);
		y[6] = ssGetRWorkValue(S,6);
		y[7] = ssGetRWorkValue(S,7);
		y[8] = ssGetRWorkValue(S,8);
		y[9] = ssGetRWorkValue(S,9);
    }
#endif

}

/* Function to compute model update */
static void mdlUpdate(real_T *x, const real_T *u, SimStruct *S, int_T tid)
{
}
 
/* Function to compute derivatives */
static void mdlDerivatives(real_T *dx, const real_T *x, const real_T *u, SimStruct *S, int_T tid)
{
}
 
/* Function to perform housekeeping at execution termination */
static void mdlTerminate(SimStruct *S)
{        
#ifndef MATLAB_MEX_FILE
    char tmp[16];

                sendStringAndWait("unlogall\r\n");
                rl32eCloseCOMPort(COM_PORT);
                rs232ports[COM_PORT]=0;
#endif
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
