/*
 *  vsbc_visteon_wheelspeed.c:
 *      Device driver for reading 4 channels of 8 bit wheel speed from PIC
 *      wheelspeed circuit.  Adapted from PSPIN_DELAYaul and Sam's digitial I/O program.
 *      Which was Based on sfuntmpl.c: C template for a level 2 S-function.
 *
 *      This code basically steps through 4 addresses on the PIC and reads the
 *      8 bit wheel speed counts off of the bus.  Most of the time is spent waiting
 *      for the PIC to respond.  PIC and C code not yet really optomized yet.
 *
 *      Christopher R. Carlson 7.31.00
 ****  Updated 12/09/04 by Christopher Gadda  ****
 *
 *      Originally this file was called Vsbc_visteon_wheelspeed_SX.c.  I've
 *      shortened it to just wheelspeed.c since it has nothing to do with
 *      Visteon anymore, and it's not like there are other wheelspeed sensors
 *      that aren't hooked to the VSBC.  The SX part is still relevent, since this
 *      driver talks to the wheelspeed board that's based on the Ubicom SX part,
 *      not a PIC microcontroller, but this comment should cover that fact.
 *
 *      I've also changed this code around a little bit so that we only read the
 *      the channels that we're actually interested in.  This is important
 *      because there's a significant delay associated with reading each
 *      channel.  As part of this change, I removed the NUM_CHANNELS s-function
 *      parameter, so now you need only specify the sample time.  Which channels
 *      are actually read is specified at compile time using the defines
 *      NUM_CHANNELS, CHANNEL_INDICES, and STR_CHANNEL.  (See below)
 *
 */

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  wheelspeed

#include "simstruc.h" // for SimStruct & related macros

#ifdef    MATLAB_MEX_FILE
#include  "mex.h"
#endif

// Here's where you can change which channels get read.
// There's a hefty performance hit for each additional channel that gets read,
// so don't read any that you don't need.
#define NUM_CHANNELS            (2)		// Number of wheelspeed channels
#define CHANNEL_INDICES			{2,3}	// Choose from 0,1,2,&3
#define STR_CHANNEL				(0)		// 1 if you want to read the steering
										// channel, 0 if you don't.
/*										
// These delay times have to be hand tuned.  These seem to work well on an
// 800MHz Pentium 3, using Watcom 11.  Faster processors will need larger
// values, a slower processor can get away with smaller values.
// Unfortunately, these delay loops are also compiler dependent, so we have to
// check the version of Watcom C that we're using.  
#if __WATCOMC__ == 1230
// If we're using openWatcom C 1.3, these are the right values.
#define SMALL_DELAY			5000
#define BIG_DELAY			(1*SMALL_DELAY)
#pragma message ("It appears you are using openWatcom 1.3");
#else // Older version of Watcom, probably Watcom 11
//  If we're using Watcom C 11, these are the right values:
//#define SMALL_DELAY			10000
#define BIG_DELAY			(320*SMALL_DELAY)
#pragma message ("It appears you are using Watcom 11");
#endif // end of __WATCOM__ == 1230
*/

// If using Microsoft Visual C Compiler
//#ifdef __MSVC__
#define SMALL_DELAY			5000
#define BIG_DELAY			(1*SMALL_DELAY)
//#endif

// I/O Register Addresses
// These may need to be adjusted if your VSBC is configured differently or if
// you're using a digital I/O card instead of the on-board digital I/O
#define DCAS_ADDRESS                (0xE2)
#define DIOLO_ADDRESS               (0xE6)
#define DIOHI_ADDRESS               (0xE7)

// Below this line, you shouldn't have to change anything...
//============================================================================



/*=========================================================================*
 * Number of S-function Parameters and macros to access from the SimStruct *
 *=========================================================================*/
#define NUM_PARAMS                  (1)
#define SAMPLE_TIME_PARAM           (ssGetSFcnParam(S,0))

/*==================================================*
 * Macros to access the S-function parameter values *
 *==================================================*/
#define AD_SAMPLE_TIME          ((uint_T) mxGetPr(SAMPLE_TIME_PARAM)[0])

/*****************************************
* Global Variable Declarations           *
*****************************************/
// NOTE:  These global variables should probably be replaced with RWorks,
// but since there isn't any occasion where you would want to have more than
// one wheelspeed driver block in the same model, it's ok to use a global
// variable.
static int FirstLoop;   // This is a global variable which keeps track of
	// how many times we run CRC_mdlStart.  It get initialized to one in
	// mdlStart() and then is set to zero once mdlOutputs() is run
	// the first time.


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


// This function just idles away a lot of processor cycles.  The point is to
// give the PIC time to respond to a request, since the xPC processor and
// digital I/O card run so much faster than the PIC.  It's terribly wasteful,
// but with the way the hardware works currently, there's no getting around it.
void spinDelay(int count)
{
// MS Visual C compiler seems to optimize away this loop, so we make
// the loop variable volatile to prevent it from doing so (CEB 9/25/2009)
#if XPCMSVISUALC
	volatile int i;
#else
    int i;
#endif
    
	for(i=0;i<count;i++);
}

#define STEERING_CHANNEL	4
#define UP_RESET			0x08
#define DOWN_RESET			0x20

// The reason these upcodes have A's in them instead of 2's is because the
// highest bit is actually wired to the handwheel encoder interface board,
// where it controls when the outputs of the handwheel encoder are allowed
// to change.  When it's high, the outputs can change; when low they do not
// change.  Setting it low in the downcodes ensures that whenever the driver
// for the handwheel encoder (handwheel.c) gets called, the encoder outputs
// will not be changing.  But it needs to go high sometime or the outputs
// would never change, so here's where it gets to go high.  Grody hack you
// say?  Well, I can't argue with that.

const unsigned char upcodes[]={0xA8,0xA9,0xAA,0xAB,0xAC};
const unsigned char downcodes[]={0x20,0x21,0x22,0x23,0x24};

/* Function: GetWS =======================================================
* Abstract:
*  This function toggles the enable and chip select lines high
*  and low as required to address and read the data on the SX
*  board.  See SteeringAngleWheelspeeds1.src in the ubicom directory
*  for the definitions of the data and address lines.
*
*  Here I use DIOLO for data and DIO_ADDRESS for the address and
*  enable lines
*
*/

int GetWS(int channel)
{
    int   Count, CountLSB;

    //Address WheelSpeed0, first read the low byte
    outp(DIOHI_ADDRESS,upcodes[channel]);           // set enable high and poll for acknowledge
	spinDelay(SMALL_DELAY);
    outp(DIOHI_ADDRESS,downcodes[channel]);
	spinDelay(SMALL_DELAY);
    CountLSB = inp(DIOLO_ADDRESS);

    outp(DIOHI_ADDRESS,upcodes[channel]);
	spinDelay(SMALL_DELAY);
    outp(DIOHI_ADDRESS,downcodes[channel]);
	spinDelay(SMALL_DELAY);
    Count = inp(DIOLO_ADDRESS)*128 + CountLSB;

    return(Count);
}

void HWRS(void)
{
    outp(DIOHI_ADDRESS,UP_RESET);           // Bring MCLR low
	spinDelay(SMALL_DELAY);
    outp(DIOHI_ADDRESS,DOWN_RESET);
    spinDelay(BIG_DELAY);   // Bring MCLR high again and wait for to stabelize
}

int GetSTR(void){
    int   Str, StrLSB, StrMSB;

    //Address WheelSpeed0, first read the low byte
    outp(DIOHI_ADDRESS,upcodes[STEERING_CHANNEL]);           // set enable high and poll for acknowledge
	spinDelay(SMALL_DELAY);
    outp(DIOHI_ADDRESS,downcodes[STEERING_CHANNEL]);
	spinDelay(SMALL_DELAY);
    StrLSB = inp(DIOLO_ADDRESS);

    outp(DIOHI_ADDRESS,upcodes[STEERING_CHANNEL]);
	spinDelay(SMALL_DELAY);
    outp(DIOHI_ADDRESS,downcodes[STEERING_CHANNEL]);
	spinDelay(SMALL_DELAY);
    StrMSB = inp(DIOLO_ADDRESS);

    Str = (StrMSB & 0x03) * 128 + StrLSB;       //Strip off winkel 7 and 8 and scale
    if( StrMSB & 0x04 ){                        // Left turn is positive
        Str = -1 * Str;
    }

    Str = Str * 2.5;        // Quoted 2.5 degreese per tic

    return(Str);
}

// What is the point of this function?  The world may never know...
void CRC_mdlStart()
{
	FirstLoop = 0;  //Global flag cleared, this fxn runs once only

    outp(DCAS_ADDRESS,0x02);   // initialize D0-D7 as input, D8-D15 as output

    // Burn some cycles to make sure the DIO stabilizes
	spinDelay(SMALL_DELAY);

    // Read all channels on startup to clear their (full) buffers
	HWRS();           // Hardware reset
	spinDelay(BIG_DELAY);

    // Clear the buffers
    GetWS(0);	// These functions return a value, but it's bogus data
    GetWS(1);	// so we'll just throw it away.
    GetWS(2);
    GetWS(3);
    GetSTR();
}

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
    FirstLoop = 1;  // Set the flag to initialize the harware on the first
					// time step.  Why don't we just initialize the hardware
					// right here?  That's a good question.  Too bad it wasn't
					// ever documented anywhere.
}
#endif /*  MDL_START */

/* Function: mdlOutputs =======================================================
 *    In this function, you compute the outputs of your S-function
 *    block. Generally outputs are placed in the output vector, ssGetY(S).
 */

// We need this because you can't de-reference an array initializer in C
const unsigned int channelIndices[NUM_CHANNELS]=CHANNEL_INDICES;

static void mdlOutputs(SimStruct *S, int_T tid)
{
#ifndef MATLAB_MEX_FILE
	int i;
	real_T *Count = ssGetOutputPortRealSignal(S,0);
    
    if(FirstLoop == 1)
	{
		CRC_mdlStart();

		for(i=0;i<(NUM_CHANNELS+STR_CHANNEL);i++)
			Count[i] = 0;
	}
	else
	{
		for(i=0;i<NUM_CHANNELS;i++)
			Count[i] = GetWS(channelIndices[i]);

		if(STR_CHANNEL)
			Count[i] = GetSTR();
	}
#endif // MATLAB_MEX_FILE
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
// Why is this next line commented out?  Hard to say, since no one bothered
// to document it.  It might be that this caused xPC to crash; I've heard a
// rumor of such problems with mdlTerminate code that tries to access hardware.
/*    outp(0xE4,0xC0);  */
}

// Here we undef all the standard s-function callbacks that we're not using.
#undef MDL_INITIALIZE_CONDITIONS 
#undef MDL_UPDATE
#undef MDL_DERIVATIVES


/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
