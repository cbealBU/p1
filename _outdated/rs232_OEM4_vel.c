/* $Revision: 107 $ $Date: 2005-07-10 12:22:29 -0700 (Sun, 10 Jul 2005) $ */
/* rs232_setup.c - xPC Target, non-inlined S-function driver for setup RS232 on motherboards  */
/* Copyright (c) 1994-1999 by The MathWorks, Inc. All Rights Reserved. */

/*
    Driver written by CRC by on ~4/9/2001 from a collection of drivers by Gerdes
    and Mathworks.  You don't like it, modifications start at $1 a minute.

    Modified on 8/22/01 by CRC to include the GPS time stamp from the header

    Modified on 8/29/01 by CRC to fix floating point precision
    problem with gps time stamp log

    Modified on 3/5/02 by CRC to log vertical velocity data for
    preliminary road grade assesment

    This is actually a driver to interface with
    the OEM4 GPS module directly and not
    just a general purpose RS232 driver.

    Reads and parses in BESTVEL log for the following variables:
    y[0] = GPS time stamp
    y[1] = latency in [sec]
    y[2] = horozontal speed in [m/s]
    y[3] = Track over ground (direction w.r.t true north) [deg]
    y[4] = vertical speed in [m/s]
    y[5] = Solution Status as per page 103 in the Novatel book 2 (3 good the rest bad)

    You MUST wait at least 10 seconds after powering up the receiver before
    you run this driver, (You need to wait for the antennae to lock anyway.)
    or else the serial port configuration sometimes does not work.

    All floats to percision given by receiver.

    Driver latency is about 13ms in addition to GPS card latency

    The only parameter passed to this block is the sample rate.  The other
    parameters you may want to change are #defined below.
*/

#undef S_FUNCTION_NAME
#define S_FUNCTION_NAME rs232_OEM4_vel

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

#define PORT_ARG                 1          //Com port number -1
#define BAUDRATE_ARG             115200     //Must reset GPS receiver each time you change this
#define DATABIT_ARG              8          //# of data bits in a packet
#define STOPBIT_ARG              1
#define PARITY_ARG               0x100
#define PROTOCOL_ARG             0       //Protocol ?? I am guessing this is handshaking
#define SENDBUF_ARG              8192
#define RECBUF_ARG               8192

#define NO_I_WORKS              (0)
#define NO_R_WORKS              (6)

static char_T msg[512];

extern int rs232ports[];
extern int rs232recbufs[];

static char msgBuff[8192];           //Global Variables to buffer serial port data
static int  index;
static int  starStatus;

static void mdlInitializeSizes(SimStruct *S)
{
        int_T num_channels, mux;

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


        /* Set-up size information (Simulink, bleah!) */
        ssSetNumContStates(S, 0);
        ssSetNumDiscStates(S, 0);
        ssSetNumOutputs(S, 6);
        ssSetNumInputs(S, 0);
        ssSetDirectFeedThrough(S, 0);           /* Direct dependency on inputs */
        ssSetNumSampleTimes(S,0);
        ssSetNumInputArgs(S, NUMBER_OF_ARGS);
        ssSetNumIWork(S, NO_I_WORKS);           /* number of integer work variables */
        ssSetNumRWork(S, NO_R_WORKS);           /* number of real work variables */
        ssSetNumPWork(         S, 0);           /* number of pointer work vector elements*/
        ssSetNumModes(         S, 0);           /* number of mode work vector elements   */
        ssSetNumNonsampledZCs( S, 0);           /* number of nonsampled zero crossings   */
        ssSetOptions(          S, 0);           /* general options (SS_OPTION_xx)        */

}

/* Function to initialize sample times */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, (uint_T)mxGetPr(SAMPLE_TIME)[0]);
    ssSetOffsetTime(S, 0, 0.0);
}


static void mdlInitializeConditions(real_T *x0, SimStruct *S)
{
#ifndef MATLAB_MEX_FILE

//    baudrates are {115200,57600,38400,19200,9600,4800,2400,1200,300,110};
//    parities are  {0x100,0x200,0x400};
    int k, kt, i, j;
    char tmp[256], c, ct;

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

    rs232ports[PORT_ARG]=1;

/* Initialization strings for GPS receiver */
    sprintf( tmp, "com,com1,%i\n\r",BAUDRATE_ARG);  //Set up the com port
    rl32eSendBlock(PORT_ARG, (void *) tmp, strlen(tmp));
    while (!(rl32eLineStatus(PORT_ARG) & TX_SHIFT_EMPTY));

/* This resend is just a delay tactic.  For some reason
   this is the only delay that seems to work reliably */
    sprintf( tmp, "com,com1,%i\n\r",BAUDRATE_ARG);  //Set up the com port
    rl32eSendBlock(PORT_ARG, (void *) tmp, strlen(tmp));
    while (!(rl32eLineStatus(PORT_ARG) & TX_SHIFT_EMPTY));



    for(i=0;i<1000000;i++); //This is a necessary and tested delay
//Reset the com port to the new desired speed.
    rl32eInitCOMPort(   PORT_ARG,
                DefaultCOMIOBase[PORT_ARG],
                DefaultCOMIRQ[PORT_ARG],
                BAUDRATE_ARG,
                PARITY_ARG,
                STOPBIT_ARG,
                DATABIT_ARG,
                RECBUF_ARG,
                SENDBUF_ARG,
                PROTOCOL_ARG);
    rs232ports[PORT_ARG]=1;

    strcpy( tmp, "unlogall \n\r");                          //Empty all data logs
    rl32eSendBlock(PORT_ARG, (void *) tmp, strlen(tmp));
        while (!(rl32eLineStatus(PORT_ARG) & TX_SHIFT_EMPTY));

    for(i = 0; i<10000000; i++);
    k = rl32eReceiveBufferCount(PORT_ARG);
    for(i = 0; i<k; i++){
        c=rl32eReceiveChar(PORT_ARG);
    }

    strcpy( tmp, "log com1 BESTVELA ontime 0.1 \n\r");      //Velocity @ 10hz
    rl32eSendBlock(PORT_ARG, (void *) tmp, strlen(tmp));        //The A after bestvel means ASCII
        while (!(rl32eLineStatus(PORT_ARG) & TX_SHIFT_EMPTY));

    index = 0;
    starStatus = 0;

    ssSetRWorkValue(S,0,0);
    ssSetRWorkValue(S,1,0);
    ssSetRWorkValue(S,2,0);
    ssSetRWorkValue(S,3,0);
    ssSetRWorkValue(S,4,0);
    ssSetRWorkValue(S,5,0);


#endif
}

/* Function to compute outputs */
static void mdlOutputs(real_T *y, const real_T *x, const real_T *u, SimStruct *S, int_T tid)
{
#ifndef MATLAB_MEX_FILE

// !!!! NOTE !!! This function accesses global variables as they appear on line 55
// It needs to do this because the serial string spans multiple sample times
//
//      static char msgBuff[8192];           //Global Variables to buffer serial port data
//      static int  index;
//      static int  starStatus;

    int i,j,k,              //General purpose counters
        parseMe,            //flag which signifies time to parse variable
        fieldnum = 0,       //This is the field number index for parsing
        kk = 0;             //debug variables

    char c, in,             //Temporary char variables for parsing GPS string
         tmp[512];

    double out;              //Temporary float variable for parsing GPS string

/* Check to make sure the port is open */
    if (!rs232ports[PORT_ARG]) {
        printf("RS232 I/O-send Error: choosen COM1-port not initialized\n");
        return;
    }

    parseMe = 0;
    k = rl32eReceiveBufferCount(PORT_ARG);  //If there is anything, it is all there
    kk = k;

    for(i=0; i<k; i++){
        c=rl32eReceiveChar(PORT_ARG);       //Read in the first char off the buffer

        if(starStatus == 0){                //If not in a string, poll for pound sign
            if(c == '#'){
                starStatus = 1;
            }
        }
        else{                           //If in the string
            if(c != '*'){
                msgBuff[index] = c;
                index++;
            }
            else{
                starStatus = 0;
                 parseMe = 1;
                msgBuff[index] = ',';    //We poll for commas later so we need to add one at the end
                index++;
            }
        }
    }

/* If we have a full string, parse it */
    if(parseMe == 1){                             //If so, then we just caught a star
        j = 0;
        fieldnum = 0;
        for(i = 0; i< index; i++) {               // k is the length of the output string
            in = msgBuff[i];
            if (in !=',' && in !=';'){
                tmp[j] = in;
                j++;
            }
            else {
               tmp[j] = '\0';                     //Tack on a terminator
               switch (fieldnum) {
                    case 6:                         //GpsTime [sec into the week]
                        sscanf(tmp,"%lf",&out);
                        y[0]=  (real_T) out;
                        break;

                    case 10:                         //Solution Status [pg 103, book2]
                        switch(tmp[0]){
                            case 'S':
                                if(tmp[1] == 'O') y[5] = 3;
                                else y[5] = 5.1;
                                break;
                            case 'I':
                                y[5] = 1.1;
                                break;
                            case 'N':
                                if(tmp[1] == 'O') y[5] = 2.1;
                                else y[5] = 11.1;
                                break;
                            case 'C':
                                if(tmp[2] == 'V') y[5] = 4.1;
                                else y[5] = 6.1;
                                break;
                            case 'V':
                                if(tmp[2] == 'H') y[5] = 7.1;
                                else y[5] = 8.1;
                                break;
                            case 'R':
                                y[5] = 9.1;
                                break;
                            case 'D':
                                y[5] = 0.1;
                                break;
                            default:
                                y[5] = 0;
                                break;
                        }
                        break;
                    case 12:                         //Latency in [Sec]
                        sscanf(tmp,"%lf",&out);
                        // y[1]= (real_T) out;       //from old version
                        y[1] = 0;
                        break;
                    case 14:                         //horozontal speed [m/s]
                        sscanf(tmp,"%lf", &out);
                        y[2]= (real_T) out;
                        break;
                    case 15:                         //track over ground [deg]
                        sscanf(tmp,"%lf",&out);
                        y[3]= (real_T) out;
                        break;
                    case 16:                         //vertical speed [m/s]
                        sscanf(tmp,"%lf",&out);
                        y[4]= (real_T) out;
                        break;
                    default:
                        break;
                }
                fieldnum++;
                j = 0;
            }
        }//End For
        parseMe = 0;
        index = 0;

        ssSetRWorkValue(S,0,y[0]);
        ssSetRWorkValue(S,1,y[1]);
        ssSetRWorkValue(S,2,y[2]);
        ssSetRWorkValue(S,3,y[3]);
        ssSetRWorkValue(S,4,y[4]);
        ssSetRWorkValue(S,5,y[5]);
    } //End parseME
    else{
        y[0] = (real_T) ssGetRWorkValue(S,0);
        y[1] = (real_T) ssGetRWorkValue(S,1);
        y[2] = (real_T) ssGetRWorkValue(S,2);
        y[3] = (real_T) ssGetRWorkValue(S,3);
        y[4] = (real_T) ssGetRWorkValue(S,4);
        y[5] = (real_T) ssGetRWorkValue(S,5);
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

        strcpy( tmp, "unlogall\n\r");
        rl32eSendBlock(PORT_ARG, (void *) tmp, strlen(tmp));
            while (!(rl32eLineStatus(PORT_ARG) & TX_SHIFT_EMPTY));
        rl32eCloseCOMPort(PORT_ARG);
        rs232ports[PORT_ARG]=0;
#endif
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
