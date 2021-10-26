
/*
 * File: asciiStreamtoString
 * Abstract:
 *       A 'C' function to turn streams of ASCII characters into strings
 *
 */


/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function.
 */

#define S_FUNCTION_NAME  asciiStreamtoString
#define S_FUNCTION_LEVEL 2

#define NUM_INPUTS            2
#define IN_PORT_0_NAME        u0
#define IN_PORT_1_NAME        u1
#define INPUT_0_WIDTH         100
#define INPUT_1_WIDTH         1
#define INPUT_DIMS_0_COL      1
#define INPUT_DIMS_1_COL      1
#define INPUT_0_DTYPE         SS_UINT8
#define INPUT_1_DTYPE         SS_UINT8
#define INPUT_0_COMPLEX       COMPLEX_NO
#define INPUT_1_COMPLEX       COMPLEX_NO
#define INPUT_0_FEEDTHROUGH   1
#define INPUT_1_FEEDTHROUGH   1

#define NUM_OUTPUTS           1
#define OUT_PORT_0_NAME       y0
#define OUTPUT_0_WIDTH        100
#define OUTPUT_DIMS_0_COL     1
#define OUTPUT_0_DTYPE        SS_UINT8
#define OUTPUT_1_DTYPE        SS_UINT8
#define OUTPUT_0_COMPLEX      COMPLEX_NO
#define OUTPUT_1_COMPLEX      COMPLEX_NO

#define NUM_PARAMS               0

#define SAMPLE_TIME_0         INHERITED_SAMPLE_TIME
#define NUM_DISC_STATES       0
#define DISC_STATES_IC        [0]
#define NUM_CONT_STATES       0
#define CONT_STATES_IC        [0]

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 *
 * The following headers are included by matlabroot/simulink/include/simstruc.h
 * when compiling as a MEX file:
 *
 *   matlabroot/extern/include/tmwtypes.h    - General types, e.g. real_T
 *   matlabroot/extern/include/mex.h         - MATLAB MEX file API routines
 *   matlabroot/extern/include/matrix.h      - MATLAB MEX file API routines
 *
 * The following headers are included by matlabroot/simulink/include/simstruc.h
 * when compiling your S-function with RTW:
 *
 *   matlabroot/extern/include/tmwtypes.h    - General types, e.g. real_T
 *   matlabroot/rtw/c/libsrc/rt_matrx.h      - Macros for MATLAB API routines
 *
 */
#include "simstruc.h"


/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"error encountered due to ...");
 *       return;
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable in your procedure. For example the following
 * will cause unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[256];         {ILLEGAL: to fix use "static char msg[256];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 */

/*====================*
 * S-function methods *
 *====================*/

/*
 * Model Initialization in Simulink
 */
#define MDL_INITIAL_SIZES
static void mdlInitializeSizes(SimStruct *S)
{
    
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }
    
    ssSetNumContStates(S, NUM_CONT_STATES);
    ssSetNumDiscStates(S, NUM_DISC_STATES);
    
    /* set the input port properties */
    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;
    //if(!ssSetInputPortDimensionInfo( S, 0, DYNAMIC_DIMENSION)) return;
    ssSetInputPortWidth(S, 0, INPUT_0_WIDTH);
    ssSetInputPortDataType(S, 0, SS_UINT8);
    ssSetInputPortComplexSignal(S, 0, INPUT_0_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 0, INPUT_0_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/
    
    ssSetInputPortWidth(S, 1, INPUT_1_WIDTH);
    ssSetInputPortDataType(S, 1, SS_UINT8);
    ssSetInputPortComplexSignal(S, 1, INPUT_1_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 1, INPUT_1_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/
    
    
    /* set the output port properties */
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;
    //if(!ssSetOutputPortDimensionInfo( S, 0, DYNAMIC_DIMENSION)) return;
    ssSetOutputPortWidth(S, 0, OUTPUT_0_WIDTH);
    ssSetOutputPortDataType(S, 0, SS_UINT8);
    ssSetOutputPortComplexSignal(S, 0, OUTPUT_0_COMPLEX);
    
    /* establish a pointer to a work vector (yet to be created) */
    ssSetNumPWork(S, 1);
    ssSetNumIWork(S, 1);
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Initialize the sample times array.
 */
#define MDL_INITIALIZE_SAMPLE_TIMES
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLE_TIME_0);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
} /* end mdlInitializeSampleTimes */


#define MDL_START
static void mdlStart(SimStruct *S)
{
    unsigned char *allocatedMem = malloc(100*sizeof(unsigned char));
    // initialize to zero stored characters
    ssSetIWorkValue(S,0,0);
    // initialize to an empty character buffer
    memset(allocatedMem,'\0',100);
    ssSetPWorkValue(S,0,allocatedMem);

    
}

/* Function: mdlOutputs =======================================================
 *
 */
#define MDL_OUTPUTS
static void mdlOutputs(SimStruct *S, int_T tid)
{
    int i; // loop variable
    unsigned char *location = NULL;
    const char testChar = '\n';

    //void **pW = ssGetPWork(S);
    unsigned char *u0 = (unsigned char*)ssGetInputPortSignal(S, 0);
    const uint8_T *u1 = (uint8_T*)ssGetInputPortSignal(S, 1);
    unsigned char *y0 = (unsigned char*) ssGetOutputPortSignal(S, 0);
    
    int_T rcvdChars = ssGetIWorkValue(S,0); // Get stored number of characters
    unsigned char *charBuffer = (unsigned char*)ssGetPWorkValue(S,0); // Get pointer to character buffer
    unsigned char *output = (unsigned char*)ssGetOutputPortSignal(S,0); // Get pointer to output port
    //printf("Data flag is %d\n",u1[0]);
    if(0 == u1[0]){ // If new data is available on inport 1
        //printf("%d characters received on entry\n",rcvdChars);
        
        /* Determine how many characters are available from the input */
        //int_T uWidth = ssGetInputPortWidth(S,0);
        
        //printf("%d chars - input is: %s\n",rcvdChars,u0);
        /* Copy the appropriate number of characters into the char array */
        for (i = 0; i < INPUT_0_WIDTH; i++)
        {
            if((char)0 == u0[i])
            {
                break;
            }
            else
            {
                /* Copy over any non-blanks to the character buffer */
                charBuffer[rcvdChars] = u0[i];
                /* Increment the number of caught characters */
                rcvdChars++;
            }
        }

        /*printf("String is: ");
        for(int i = 0; i < rcvdChars; i++ )
            printf("%c",charBuffer[i]);
        printf("\n");*/

        /* memcpy does not work since need to test for blanks as above ^ */
        //memcpy(charBuffer+rcvdChars,u0,sizeof(unsigned char)*INPUT_0_WIDTH);
        /* Update the number of captured characters. Redundant from above. */
        //rcvdChars += INPUT_0_WIDTH;

        /* Old code, get rid of it soon */
        //charBuffer[rcvdChars] = u0[0];
        //charBuffer[rcvdChars] = '\0'; // terminate the character array as a string
        
        //printf("%d total received characters\n",rcvdChars);
        
        /* Check to see if there is a carriage return in the char array */
        for (i = 0; i < rcvdChars; i++ )
            if (charBuffer[i] == testChar)
            {
                location = &charBuffer[i];
                /* break since we only want to find the first newline */
                break;
            }
        
        //location = strchr(charBuffer,testChar);
        
    //printf("%p\n",location);
        /* If no carriage return, work is done. Keep storing values in the
         * PWORK vector for future output. */
        if (NULL == location){
            memset(output,(char)0,100); // zero the output buffer since there is no new message
            /*printf("String is: ");
            for(int i = 0; i < rcvdChars; i++ )
                printf("%c",charBuffer[i]);
            printf("\n");*/
            /*if (rcvdChars >= 90) // if there are more than 90 chars and no carriage returns, trash the garbage in the buffer
            {
                rcvdChars = 0;
                memset(charBuffer,(char)0,100);
            }*/
                
            /*printf("String is: ");
            for(int i = 0; i < rcvdChars; i++ )
                printf("%c",charBuffer[i]);
             printf("\n");*/
            //printf("%s\n",charBuffer);
        }
        /* If there is a carriage return, change the output value to be the
         * portion of the PWORK vector up to the carriage return and shift the
         * remainder of the PWORK vector back to the start. */
        else
        {
            // printf("Found a carriage return at %p, processing...\n",location);
            /*for(int i = 0; i < location-charBuffer; i++ )
               printf("%c",charBuffer[i]);
            printf("\n");*/
            //printf("%s\n",charBuffer);
            // copy the completed string to the output
            // memcpy(y0,charBuffer,rcvdChars-1); // minus one to skip copying the carriage return
            //ssSetOutputPortWidth(S, 0, rcvdChars-1);
            //output = ssGetOutputPortSignal(S,0);
            //for (int i = 0; i < rcvdChars -1; i++)
            //    *(output+i) = charBuffer[i];
            memset(output,(char)0,100); // zero the output buffer before filling
            memcpy(output,charBuffer,(location-charBuffer-1)*sizeof(unsigned char)); // minus one to skip copying the carriage return
            //memset(output+rcvdChars*sizeof(uint8_T),'\0',100-rcvdChars);
            // subtract the number of characters sent to the output
            /*for(i = 0; i < rcvdChars; i++ )
               printf("%c",output[i]);
            printf("\n");*/
            rcvdChars -= (location - charBuffer + 1); // plus one to strip the carriage return
            //rcvdChars = 0;
            //printf("%s",output);
            // shift the data back to the start of the buffer
            //memmove(charBuffer,charBuffer+30,rcvdChars); // plus 2 to skip storing the carriage return and newline
            memmove(charBuffer,location+1,rcvdChars); // plus 1 to skip storing the carriage return and newline
            memset(charBuffer+rcvdChars,(char)0,100-rcvdChars);
        }
        
        
        ssSetIWorkValue(S,0,rcvdChars);
    }
    else
    {
        //printf("Data flag is %d\n",u1[0]);
    }
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    unsigned char *allocatedMem = (unsigned char*)ssGetPWorkValue(S,0);
    free(allocatedMem);
}


/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
