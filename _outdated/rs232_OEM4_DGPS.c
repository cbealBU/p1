/*
    Driver written by CRC by on ~4/9/2001 from a collection of drivers by Gerdes
    and Mathworks.  You don't like it, modifications start at $1 a minute.

    Modified on 8/22/01 by CRC to include the GPS time stamp from the header

    Modified on 8/29/01 by CRC to fix floating point precision
    problem with gps time stamp log

    Modified on 3/5/02 by CRC to log vertical velocity data for
    preliminary road grade assesment

    Modified on 6/08/02 by CRC to log position and velocity data in the
    same log so that we may look at navigation information.

    This is actually a driver to interface with
    the OEM4 GPS module directly and not
    just a general purpose RS232 driver.

    Reads and parses in BESTXYZ log for the following variables:
    (This returns everything in the ECEF, WGS84 coordinates
    y[0] = GPS time stamp
    y[1] = Position Solution Status
    y[2] = Position Solution Type
    y[3] = Pos x
    y[4] = Pos y
    y[5] = Pos z
    y[6] = Velocity Solution Status
    y[7] = Position Solution Type
    y[8] = Vel x
    y[9] = Vel y
    y[10] = Vel z
    y[11]= Number of satelites used for solution
    y[12] = Latency of Velocity Measurements

    You MUST wait at least 10 seconds after powering up the receiver before
    you run this driver, (You need to wait for the antennae to lock anyway.)
    or else the serial port configuration sometimes does not work.

    All floats to percision given by receiver.

    Driver latency is about 26ms in addition to GPS card latency

    The only parameter passed to this block is the sample rate.  The other
    parameters you may want to change are #defined below.
*/

#undef S_FUNCTION_NAME
#define S_FUNCTION_NAME rs232_OEM4_DGPS

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
#define NUM_OUTPUTS             (13)

#define PORT_ARG                 1          //Com port number -1
#define BAUDRATE_ARG             115200     //Must reset GPS receiver each time you change this
#define DATABIT_ARG              8          //# of data bits in a packet
#define STOPBIT_ARG              1
#define PARITY_ARG               0x100
#define PROTOCOL_ARG             0       //Protocol ?? I am guessing this is handshaking
#define SENDBUF_ARG              8192
#define RECBUF_ARG               8192

#define NO_I_WORKS              (0)
#define NO_R_WORKS              (0)

static char_T msg[SENDBUF_ARG];

extern int rs232ports[];
extern int rs232recbufs[];

static char msgBuff[SENDBUF_ARG];           //Global Variables to buffer serial port data
static int  index;
static int  starStatus;
static double OutputLatch[NUM_OUTPUTS];

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
        ssSetNumOutputs(S, NUM_OUTPUTS);
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

    strcpy( tmp, "com com2 38400 \n\r");            // 
    rl32eSendBlock(PORT_ARG, (void *) tmp, strlen(tmp));        //
        while (!(rl32eLineStatus(PORT_ARG) & TX_SHIFT_EMPTY));


    strcpy( tmp, "interfacemode com2 rtca none \n\r");            // 
    rl32eSendBlock(PORT_ARG, (void *) tmp, strlen(tmp));        //
        while (!(rl32eLineStatus(PORT_ARG) & TX_SHIFT_EMPTY));

    strcpy( tmp, "log com1 BESTXYZA ontime 0.1 \n\r");            // 10hz
    rl32eSendBlock(PORT_ARG, (void *) tmp, strlen(tmp));        //The A after bestvel means ASCII
        while (!(rl32eLineStatus(PORT_ARG) & TX_SHIFT_EMPTY));

    index = 0;
    starStatus = 0;

     for(i = 0; i<NUM_OUTPUTS; i++){
          OutputLatch[i] = 1;
     }

#endif
}

double GetSolStatus( char *s ){
     double ans;
     switch(s[0]){
          case 'S':
               if(s[1] == 'O') ans = 0.1;
               else ans = 3.1;
               break;
          case 'I':
               ans = 1.1;
               break;
          case 'N':
               if(s[1] == 'O') ans = 2.1;
               else ans = 11.1;
               break;
          case 'C':
               if(s[2] == 'V') ans = 4.1;
               else ans = 6.1;
               break;
          case 'V':
               if(s[2] == 'H') ans = 7.1;
               else ans = 8.1;
               break;
          case 'R':
               ans = 9.1;
               break;
          case 'D':
               ans = 10.1;
               break;
          default:
               ans = 999.1;       //99 is the magic debug variable
               break;
     }
     return ans;
}

double GetSolType( char *s ){
     double ans;
     switch(s[0]){
          case 'N':
               if(s[1] == 'O') 
		  ans = 0;
               else if(s[7]=='F') 	// Narrow_Float
		  ans = 3.4;
               else if(s[7]=='I') 	// Narrow_INT
		  ans = 5.0;
               break;
          case 'F':
               ans = 1;
               break;
          case 'S':
               ans = 1.6;
               break;
          case 'P':
               ans = 1.7;		//PSR Diferential
               break;
          case 'W':
               if(s[1] == 'A') ans = 1.8;	//WAAS ;)
               else ans = 4.9;			//Wide_Int
               break;
          case 'L':
               if(s[3] == 'F') ans = 3.2;	//L1_Float
               else ans = 4.8;			//L1_Int
               break;
          case 'I':
               ans = 3.3;
               break;
          case 'D':                     //DOPPLER_VELOCITY (Velocity only)
               ans = 4.7;
          default:
               ans = 999.1;       //99 is the magic debug variable
               break;
     }
     return ans;
}

/* Function to compute outputs */
static void mdlOutputs(real_T *y, const real_T *x, const real_T *u, SimStruct *S, int_T tid)
{
#ifndef MATLAB_MEX_FILE

// !!!! NOTE !!! This function accesses global variables as they appear on line 55
// It needs to do this because the serial string spans multiple sample times
//
//      static char msgBuff[SENDBUF_ARG];           //Global Variables to buffer serial port data
//      static int  index;
//      static int  starStatus;

    int i,j,k,              //General purpose counters
        parseMe,            //flag which signifies time to parse variable
        fieldnum = 0,       //This is the field number index for parsing
        kk = 0;             //debug variables

    char c, in,             //Temporary char variables for parsing GPS string
         tmp[512];

    double out;              //Temporary float variable for parsing GPS string
    
    static double VelSolType;  //Storage of velocity solution type (used to default latency to 0.0
                               //if in Doppler Mode)

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
                    case 10:                         //Solution Status [pg 102, book2]
                         y[1] = GetSolStatus(tmp);
                        break;
                    case 11:                         //Solution Type [pg 101, book2]
                         y[2] = GetSolType(tmp);
                         break;
                    case 12:                            //Pos x
                        sscanf(tmp,"%lf",&out);
                        y[3]= (real_T) out;
                        break;
                    case 13:                           //Pos y
                        sscanf(tmp,"%lf",&out);
                        y[4]= (real_T) out;
                        break;
                    case 14:                           // Pos z
                        sscanf(tmp,"%lf",&out);
                        y[5]= (real_T) out;
                        break;
                    case 18:                         //Velocity Solution Status [pg 103, book2]
                         y[6] = GetSolStatus(tmp);
                        break;
                    case 19:                         //Velocity Solution Type [pg 103, book2]
                         y[7] = GetSolType(tmp);
                         VelSolType = GetSolType(tmp);
                         break;
                    case 20:                            //Vel x
                        sscanf(tmp,"%lf",&out);
                        y[8]= (real_T) out;
                        break;
                    case 21:                            //Vel y
                        sscanf(tmp,"%lf",&out);
                        y[9]= (real_T) out;
                        break;
                    case 22:                            //Vel z
                        sscanf(tmp,"%lf",&out);
                        y[10]= (real_T) out;
                        break;
                    case 31:                            //Num Sats
                        sscanf(tmp,"%lf",&out);
                        y[11]= (int) out;
                        break;
                    case 27:                            //Velocity Latency (defaulted to 0 if in doppler mode)
                        sscanf(tmp,"%lf",&out);
                        if (VelSolType == 4.7){
                            y[12] = 0.0;
                        }
                        else {
                            y[12] = (real_T) out;
                        }
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

        for( i=0; i<NUM_OUTPUTS; i++){
          OutputLatch[i] = y[i];
        }
    } //End parseME
    else{
          for( i=0; i<NUM_OUTPUTS; i++){
               y[i] = OutputLatch[i];
          }
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
