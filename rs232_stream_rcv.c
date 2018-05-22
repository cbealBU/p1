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
//#define OUT_PORT_0_NAME       y0
//#define OUT_PORT_1_NAME       y1
#define OUTPUT_0_WIDTH        33
#define OUTPUT_1_WIDTH        1
#define OUTPUT_0_COMPLEX      COMPLEX_NO
#define OUTPUT_1_COMPLEX      COMPLEX_NO

#define PORT_ARG                 1          //Com port number minus one (zero indexed)
#define BAUDRATE_ARG             115200     //Must reset GPS receiver each time you change this
#define DATABIT_ARG              8          //# of data bits in a packet
#define STOPBIT_ARG              1
#define PARITY_ARG               0x100
#define PROTOCOL_ARG             0       //Protocol ?? I am guessing this is handshaking
#define SENDBUF_ARG              8192
#define RECBUF_ARG               8192
#define TX_SHIFT_EMPTY           0x40

//#define NO_I_WORKS               (0)
//#define NO_R_WORKS               (6)

static char_T errMsg[SENDBUF_ARG];

extern int rs232ports[];
extern int rs232recbufs[];

static unsigned char msgBuff[SENDBUF_ARG];           //Global Variables to buffer serial port data
static int  index;
static double OutputLatch[NUM_OUTPUTS];

// checksum function
unsigned char checksum (unsigned char *ptr, size_t sz) {
    unsigned char chk = 0;
    while (sz-- != 0)
        chk -= *ptr++;
    return chk;
}

// function for converting all of the data to doubles
double parseCharToDouble(unsigned char *startByte)
{
    //unsigned char *tmp;
    //tmp = *startByte;
    return (double)*startByte;
}

// function for converting all of the data to doubles
double parseShortToDouble(unsigned char *startByte)
{
    unsigned char tmp[2];
    unsigned short *output;
    tmp[1] = startByte[0];
    tmp[0] = startByte[1];
    output = &tmp[0];
    return (double)*output;
}

// function for converting all of the data to doubles
double parseLongToDouble(unsigned char *startByte)
{
    unsigned char tmp[8];
    unsigned long long *output;
    tmp[7] = startByte[0];
    tmp[6] = startByte[1];
    tmp[5] = startByte[2];
    tmp[4] = startByte[3];
    tmp[3] = startByte[4];
    tmp[2] = startByte[5];
    tmp[1] = startByte[6];
    tmp[0] = startByte[7];
    output = &tmp[0];
    return (double)*output;
}

// function for converting all of the data to doubles
double parseFloatToDouble(unsigned char *startByte)
{
    unsigned char tmp[4];
    float *output;
    tmp[3] = startByte[0];
    tmp[2] = startByte[1];
    tmp[1] = startByte[2];
    tmp[0] = startByte[3];
    output = &tmp[0];
    return (double)*output;
}

// function for converting all of the data to doubles
double parseDoubleToDouble(unsigned char *startByte)
{
    unsigned char tmp[8];
    double *output;
    tmp[7] = startByte[0];
    tmp[6] = startByte[1];
    tmp[5] = startByte[2];
    tmp[4] = startByte[3];
    tmp[3] = startByte[4];
    tmp[2] = startByte[5];
    tmp[1] = startByte[6];
    tmp[0] = startByte[7];
    output = &tmp[0];
    return *output;
}

static void mdlInitializeSizes(SimStruct *S)
{
    
#ifndef MATLAB_MEX_FILE
#include "rs232_xpcimport.c"
#endif
    
#ifdef MATLAB_MEX_FILE
    ssSetNumSFcnParams(S, NUMBER_OF_ARGS);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        sprintf(errMsg,"Wrong number of input arguments passed.\n1 argument is expected\n");
        ssSetErrorStatus(S,errMsg);
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
    //ssSetOutputPortDataType(S, 0, SS_UINT8);
    //ssSetOutputPortDataType(S, 1, SS_UINT8);
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
            BAUDRATE_ARG,   //This is the default baud rate on reset
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
    // set the message index value to zero
    index = 0;
    
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
    
    double *y0 = (double*)ssGetOutputPortSignal(S, 0); // arrays corresponding to the output ports
    double *y1 = (double*)ssGetOutputPortSignal(S, 1);
    
    /* Check to make sure the port is open */
    if (!rs232ports[PORT_ARG]) {
        printf("RS232 I/O-send Error: choosen COM-port not initialized\n");
        return;
    }
    
    k = rl32eReceiveBufferCount(PORT_ARG);  // Find out how many chars are in the buffer
    //if(k > 0)
    //    printf("Debug point C: %d bytes available\r\n",k);
    
    // TODO: move this to an appropriate point later on
    for(i=0; i<k; i++) // loop through all of the bytes available
    {
        if(0 == index) // if not already in the midst of pulling bytes in the middle of a message
        {
            msgBuff[0] = (unsigned char)rl32eReceiveChar(PORT_ARG); // pull one character into the first output buffer byte (will be overwritten if not the start of a message)
            //printf("%x \r\n",msgBuff[0]);
            if(msgBuff[0] == '$') // check to see that the start of the output buffer is a $ character, signifying the start of a message
            {
                index++; // if a $ character is in place, note a successful first character received
                //printf("Debug point A\r\n");
            }
        }
        else if(index < 128) // if not already at the end of the message
        {
            msgBuff[index] = (unsigned char)rl32eReceiveChar(PORT_ARG); // read one byte out of the buffer (and into the block output)
            index++; // increment the byte count for the output
            //printf("Debug point D: index is %d\r\n",index);
            
        }
        else
        {
            // if 128 bytes have been received, we have the whole binary message, so leave the rest of the bytes in the buffer
            //printf("Debug point B\r\n");
            break;
        }
    }
    
    // if a full message was received, process it
    if(128 <= index)
    {
        index = 0; // reset the index flag back to zero so the next message can be read
        y1[0] = (double)checksum(msgBuff+8,118); // calculate and output the error in the checksum (should always be zero)
        
        // debug: print out the header information
        printf("%x %x %x %x %x %x %x... %x %x\r\n",msgBuff[0],msgBuff[1],msgBuff[2],msgBuff[3],msgBuff[4],msgBuff[5],msgBuff[6],msgBuff[126],msgBuff[127]);
        
        // now parse the message
        y0[0] = parseLongToDouble(msgBuff+0); // skipping output of header information
        y0[1] = parseDoubleToDouble(msgBuff+8);
        y0[2] = parseShortToDouble(msgBuff+16);
        y0[3] = parseShortToDouble(msgBuff+18);
        y0[4] = parseShortToDouble(msgBuff+20);
        y0[5] = parseCharToDouble(msgBuff+22);
        y0[6] = parseCharToDouble(msgBuff+23);
        y0[7] = parseDoubleToDouble(msgBuff+24);
        y0[8] = parseDoubleToDouble(msgBuff+32);
        y0[9] = parseFloatToDouble(msgBuff+40);
        y0[10] = parseFloatToDouble(msgBuff+44);
        y0[11] = parseFloatToDouble(msgBuff+48);
        y0[12] = parseFloatToDouble(msgBuff+52);
        y0[13] = parseFloatToDouble(msgBuff+56);
        y0[14] = parseFloatToDouble(msgBuff+60);
        y0[15] = parseFloatToDouble(msgBuff+64);
        y0[16] = parseShortToDouble(msgBuff+68);
        y0[17] = parseShortToDouble(msgBuff+70);
        y0[18] = parseFloatToDouble(msgBuff+72);
        y0[19] = parseFloatToDouble(msgBuff+76);
        y0[20] = parseFloatToDouble(msgBuff+80);
        y0[21] = parseFloatToDouble(msgBuff+84);
        y0[22] = parseFloatToDouble(msgBuff+88);
        y0[23] = parseFloatToDouble(msgBuff+92);
        y0[24] = parseFloatToDouble(msgBuff+96);
        y0[25] = parseFloatToDouble(msgBuff+100);
        y0[26] = parseFloatToDouble(msgBuff+104);
        y0[27] = parseFloatToDouble(msgBuff+108);
        y0[28] = parseFloatToDouble(msgBuff+112);
        y0[29] = parseFloatToDouble(msgBuff+116);
        y0[30] = parseFloatToDouble(msgBuff+120);
        y0[31] = parseShortToDouble(msgBuff+124);
        y0[32] = parseShortToDouble(msgBuff+126);
        
    }
    else
    {
        for(i = 0; i < OUTPUT_0_WIDTH; i++)
            y0[i] = 0.0; // zero the output since there is no new message
        y1[i] = -99; // flag for no data
        //printf("Debug point E\r\n");
    }
//        printf("Output 0: %02x %f\r\n",msgBuff[0],y0[0]);
//        printf("Output 32: %02x%02x %f\r\n",msgBuff[126],msgBuff[127],y0[32]);
    
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
