#define S_FUNCTION_NAME rs232_stream_rcv
#define S_FUNCTION_LEVEL 2

#include <stddef.h>
#include <stdlib.h>

#include "tmwtypes.h"
#include "simstruc.h"

#ifdef MATLAB_MEX_FILE
#include "mex.h"
#else
#include <windows.h>
#include <string.h>
#include "rs232_xpcimport.h"
#endif

/* Input Arguments */
#define NUMBER_OF_ARGS          (1)
#define SAMPLE_TIME             ssGetSFcnParam(S,0)

#define NUM_OUTPUTS           2
#define OUT_PORT_0_NAME       y0
#define OUT_PORT_1_NAME       y1
#define OUTPUT_0_WIDTH        100
#define OUTPUT_1_WIDTH        1
#define OUTPUT_0_DTYPE        SS_UINT8
#define OUTPUT_1_DTYPE        SS_UINT8
#define OUTPUT_0_COMPLEX      COMPLEX_NO
#define OUTPUT_1_COMPLEX      COMPLEX_NO

#define PORT_ARG                 1          //Com port number -1
#define BAUDRATE_ARG             9600     //Must reset GPS receiver each time you change this
#define DATABIT_ARG              8          //# of data bits in a packet
#define STOPBIT_ARG              1
#define PARITY_ARG               0x100
#define PROTOCOL_ARG             0       //Protocol ?? I am guessing this is handshaking
#define SENDBUF_ARG              8192
#define RECBUF_ARG               8192
#define TX_SHIFT_EMPTY           0x40

#define NO_I_WORKS               (0)
#define NO_R_WORKS               (6)

static char_T msg[SENDBUF_ARG];

extern int rs232ports[];
extern int rs232recbufs[];

static char msgBuff[SENDBUF_ARG];           //Global Variables to buffer serial port data
static int  index;
static int  starStatus;
static double OutputLatch[NUM_OUTPUTS];


static void mdlInitializeSizes(SimStruct *S)
{
    
#ifndef MATLAB_MEX_FILE
#include "rs232_xpcimport.c"
#endif
    
#ifdef MATLAB_MEX_FILE
    ssSetNumSFcnParams(S, NUMBER_OF_ARGS);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        sprintf(msg,"Wrong number of input arguments passed.\n1 argument is expected\n");
        ssSetErrorStatus(S,msg);
        return;
    }
#endif
    
    /* Set-up size information */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    ssSetNumInputPorts(S, 0);
    //ssSetNumInputs(S, 0);
    
    /* set the output port properties */
    ssSetNumOutputPorts(S, 2);
    //ssSetNumOutputs(S, NUM_OUTPUTS); // this is the port width
    ssSetOutputPortWidth(S, 0, OUTPUT_0_WIDTH);
    ssSetOutputPortWidth(S, 1, OUTPUT_1_WIDTH);
    ssSetOutputPortDataType(S, 0, SS_UINT8);
    ssSetOutputPortDataType(S, 1, SS_UINT8);
    ssSetOutputPortComplexSignal(S, 0, OUTPUT_0_COMPLEX);
    ssSetOutputPortComplexSignal(S, 1, OUTPUT_1_COMPLEX);
    
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, (uint_T)mxGetPr(SAMPLE_TIME)[0]);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS
#ifdef MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S)
{
#ifndef MATLAB_MEX_FILE
    int k, i; // variables to account for and loop over the amount of data in the buffer
    char c; // character to pull data out of the buffer
    
//    baudrates are {115200,57600,38400,19200,9600,4800,2400,1200,300,110};
//    parities are  {0x100,0x200,0x400};
    
    rl32eInitCOMPort(  PORT_ARG,
            DefaultCOMIOBase[PORT_ARG],
            DefaultCOMIRQ[PORT_ARG],
            9600,   //This is the default baud rate on reset
            PARITY_ARG,
            STOPBIT_ARG,
            DATABIT_ARG,
            RECBUF_ARG,
            SENDBUF_ARG,
            PROTOCOL_ARG);
    
//Reset the com port to the new desired speed.
//     rl32eInitCOMPort(   PORT_ARG,
//                 DefaultCOMIOBase[PORT_ARG],
//                 DefaultCOMIRQ[PORT_ARG],
//                 BAUDRATE_ARG,
//                 PARITY_ARG,
//                 STOPBIT_ARG,
//                 DATABIT_ARG,
//                 RECBUF_ARG,
//                 SENDBUF_ARG,
//                 PROTOCOL_ARG);
    rs232ports[PORT_ARG]=1;
    
    /* Check to make sure the port is open */
    if (!rs232ports[PORT_ARG]) {
        printf("RS232 I/O-send Error: choosen COM-port not initialized\n");
        return;
    }
    
    // Empty the buffer by reading and tossing each character available
    k = rl32eReceiveBufferCount(PORT_ARG);
    for(i = 0; i<k; i++){
        c=rl32eReceiveChar(PORT_ARG);
        
    }
    
#endif
    
}
#endif


static void mdlOutputs(SimStruct *S, int_T tid)
{
    // real_T *y, const real_T *x, const real_T *u,
#ifndef MATLAB_MEX_FILE
    
// !!!! NOTE !!! This function accesses global variables as they appear on line 55
// It needs to do this because the serial string spans multiple sample times
//
//      static char msgBuff[SENDBUF_ARG];           //Global Variables to buffer serial port data
//      static int  index;
//      static int  starStatus;
    
    int i,j,k,m;              //General purpose counters
    
    unsigned char *y0 = (unsigned char*) ssGetOutputPortSignal(S, 0);
    unsigned char *y1 = (unsigned char*) ssGetOutputPortSignal(S, 1);
    
    //static double VelSolType;  //Storage of velocity solution type (used to default latency to 0.0
    //if in Doppler Mode)
    
    /* Check to make sure the port is open */
    if (!rs232ports[PORT_ARG]) {
        printf("RS232 I/O-send Error: choosen COM-port not initialized\n");
        return;
    }
    
    k = rl32eReceiveBufferCount(PORT_ARG);  // Find out how many chars are in the buffer
    
    if (k > OUTPUT_0_WIDTH) {
        sprintf(msg,"COM Port buffer has more data than receive block buffer can handle\n");
        ssSetErrorStatus(S,msg);
        return;
    }
    y1[0] = (char)k; // set the second output port to the number of received characters
    
    // Craig's code
    for(i=0; i<k; i++)
        y0[i] = (unsigned char)rl32eReceiveChar(PORT_ARG); // Read all of the characters out of the buffer (and into the block output)
    y0[k] = (char)0;
    
#endif
}

/* Function to perform housekeeping at execution termination */
static void mdlTerminate(SimStruct *S)
{
#ifndef MATLAB_MEX_FILE
    
    /* close the COM port when done */
    rl32eCloseCOMPort(PORT_ARG);
    rs232ports[PORT_ARG]=0;
#endif
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
