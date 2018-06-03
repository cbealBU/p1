#define S_FUNCTION_NAME vs330_bin3_rcv
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
#define OUTPUT_0_WIDTH        24
#define OUTPUT_1_WIDTH        1
#define OUTPUT_0_COMPLEX      COMPLEX_NO
#define OUTPUT_1_COMPLEX      COMPLEX_NO

#define PORT_ARG                 1          // com port number minus one (zero indexed)
#define BAUDRATE_ARG             115200     // must reset GPS receiver each time you change this
#define DATABIT_ARG              8          // # of data bits in a packet
#define STOPBIT_ARG              1
#define PARITY_ARG               0x100
#define PROTOCOL_ARG             0       // protocol?? I am guessing this is handshaking
#define SENDBUF_ARG              8192
#define RECBUF_ARG               8192
#define TX_SHIFT_EMPTY           0x40

static char_T errMsg[SENDBUF_ARG];          // message string used for setup error messages

extern int rs232ports[];
extern int rs232recbufs[];

static unsigned char msgBuff[SENDBUF_ARG];           // global Variables to buffer serial port data
static int index;

// checksum function - modified from found code:
// the algorithm is clever and subtracts the bytes from the checksum 
// provided at the end of the array so that the final value is zero
unsigned short checksum (unsigned char *ptr, size_t sz) {
    unsigned short chk = *(unsigned short*)(ptr+sz); // initialize the sum with the reference checksum
    while (sz-- != 0) // keep decreasing the value of sz, the offset to the end of the array
        chk -= (unsigned short)*(ptr+sz); // subtract the value of each byte (promoting to ushort since the checksum is 2 bytes)
    return chk;
}

// function for converting all of the data to doubles
double parseCharToDouble(unsigned char *startByte)
{
    return (double)*startByte;
}

// function for converting all of the data to doubles
double parseShortToDouble(unsigned char *startByte)
{
    unsigned short *output = (unsigned short *)startByte;
    return (double)*output;
}

// function for converting all of the data to doubles
double parseLongToDouble(unsigned char *startByte)
{
    unsigned long long *output = (unsigned long long*)startByte;
    return (double)*output;
}

// function for converting all of the data to doubles
double parseFloatToDouble(unsigned char *startByte)
{
    float *output = (float*)startByte;
    return (double)*output;
}

// function for converting all of the data to doubles
double parseDoubleToDouble(unsigned char *startByte)
{
    double *output = (double*)startByte;
    return (double)*output;
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
    
    /* set the output port properties */
    ssSetNumOutputPorts(S, 2);
    ssSetOutputPortWidth(S, 0, OUTPUT_0_WIDTH);
    ssSetOutputPortWidth(S, 1, OUTPUT_1_WIDTH);
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
    
    rs232ports[PORT_ARG]=1; // set the port status flag to initialized
        
    // Empty the buffer by reading and tossing each character available
    k = rl32eReceiveBufferCount(PORT_ARG);
    for(i = 0; i < k; i++)
        c=rl32eReceiveChar(PORT_ARG);

    // set the message index value to zero
    index = 0;
    
#endif
    
}
#endif


static void mdlOutputs(SimStruct *S, int_T tid)
{
    // real_T *y, const real_T *x, const real_T *u,
#ifndef MATLAB_MEX_FILE
    
// !!!! NOTE !!! This function accesses global variables as they appear above
// It needs to do this because the serial string spans multiple sample times
//
//      static unsigned char msgBuff[SENDBUF_ARG];           //Global Variables to buffer serial port data
//      static int index;
    
    int i,k;              //General purpose counting variables
    
    double *y0 = (double*)ssGetOutputPortSignal(S, 0); // arrays corresponding to the output ports
    double *y1 = (double*)ssGetOutputPortSignal(S, 1);
    
    /* Check to make sure the port is open */
    if (!rs232ports[PORT_ARG]) {
        printf("RS232 I/O-send Error: choosen COM-port not initialized\n");
        return;
    }
    
    k = rl32eReceiveBufferCount(PORT_ARG);  // Find out how many chars are in the buffer
    
    for(i=0; i<k; i++) // loop through all of the bytes available
    {
        if(0 == index) // if not already in the midst of pulling bytes in the middle of a message
        {
            msgBuff[0] = (unsigned char)rl32eReceiveChar(PORT_ARG); // pull one character into the first output buffer byte (will be overwritten if not the start of a message)
            if(msgBuff[0] == '$') // check to see that the start of the output buffer is a $ character, signifying the start of a message
                index++; // if a $ character is in place, note a successful first character received
        }
        else if(index < 128) // if not already at the end of the message
        {
            msgBuff[index] = (unsigned char)rl32eReceiveChar(PORT_ARG); // read one byte out of the buffer (and into the block output)
            index++; // increment the byte count for the output
        }
        else
            break; // if 128 bytes have been received, we have the whole binary message, so leave the rest of the bytes in the buffer
    }
    
    // if a full message was received, process it
    if(128 <= index)
    {
        index = 0; // reset the index flag back to zero so the next message can be read
        y1[0] = (double)checksum(msgBuff+8,116); // calculate and output the error in the checksum (should always be zero)
        if(0 != y1[0])
        {
            for(i = 0; i < OUTPUT_0_WIDTH; i++)
                y0[i] = 0.0; // zero the output since there is no new message
            y1[0] = -99; // flag for no data
            return;
        }
        
        // now parse the message: commented outputs are not needed for
        // usage on P1 and are removed for clarity in the block diagram
        //y0[0] = parseLongToDouble(msgBuff+0);     // header information
        y0[0] = parseDoubleToDouble(msgBuff+8);     // GPS time of week
        y0[1] = parseShortToDouble(msgBuff+16);     // GPS week number
        //y0[3] = parseShortToDouble(msgBuff+18);   // SATS tracked
        y0[2] = parseShortToDouble(msgBuff+20);     // SATS used
        y0[3] = parseCharToDouble(msgBuff+22);      // NAV mode
        //y0[6] = parseCharToDouble(msgBuff+23);    // Spare
        y0[4] = parseDoubleToDouble(msgBuff+24);    // Latitude
        y0[5] = parseDoubleToDouble(msgBuff+32);    // Longitude
        y0[6] = parseFloatToDouble(msgBuff+40);     // Altitude
        y0[7] = parseFloatToDouble(msgBuff+44);    // Horizontal speed
        y0[8] = parseFloatToDouble(msgBuff+48);    // Vertical velocity
        y0[9] = parseFloatToDouble(msgBuff+52);    // Course over ground
        y0[10] = parseFloatToDouble(msgBuff+56);    // Heading
        y0[11] = parseFloatToDouble(msgBuff+60);    // Pitch (roll for P1)
        //y0[15] = parseFloatToDouble(msgBuff+64);    // Spare
        y0[12] = parseShortToDouble(msgBuff+68);    // Age of differential
        y0[13] = parseShortToDouble(msgBuff+70);    // Attitude status
        y0[14] = parseFloatToDouble(msgBuff+72);    // Yaw stddev
        y0[15] = parseFloatToDouble(msgBuff+76);    // Pitch (roll) stddev
        y0[16] = parseFloatToDouble(msgBuff+80);    // Horizontal RMS
        y0[17] = parseFloatToDouble(msgBuff+84);    // Vertical RMS
        //y0[22] = parseFloatToDouble(msgBuff+88);    // Horizontal DOP
        //y0[23] = parseFloatToDouble(msgBuff+92);    // Vertical DOP
        //y0[24] = parseFloatToDouble(msgBuff+96);    // Time DOP
        y0[18] = parseFloatToDouble(msgBuff+100);   // Covariance North-North
        y0[19] = parseFloatToDouble(msgBuff+104);   // Covariance North-East
        y0[20] = parseFloatToDouble(msgBuff+108);   // Covariance North-Up
        y0[21] = parseFloatToDouble(msgBuff+112);   // Covariance East-East
        y0[22] = parseFloatToDouble(msgBuff+116);   // Covariance East-Up
        y0[23] = parseFloatToDouble(msgBuff+120);   // Covariance Up-Up
        //y0[31] = parseShortToDouble(msgBuff+124);   // 16-bit bytewise checksum 
        //y0[32] = parseShortToDouble(msgBuff+126);   // Carriage return, line feed 
    }
    else // if index < 128 (a message is not being parsed this time step)
    {
    }
    
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
