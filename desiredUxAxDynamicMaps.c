/*
 * File : desiredUxAx.c
 * Modified from FFWclothoid_weightTransfer2AdvBankGrade.c
 *
 * Abstract:
 *      C-file S-function to calculate desired speed and accerelation of a vehicle.  This calculation take both bank
 * and grade into account.  Output UxDesired and AxDesired are the actual value at vehicle CG, which include grade and bank into account.
 *      Note that segment start counting from zero
 *
 * Real-Time Workshop note:
 *   This file can be used as is (noninlined) with the Real-Time Workshop
 *   C rapid prototyping targets, or it can be inlined using the Target
 *   Language Compiler technology and used with any target. See
 *     matlabroot/toolbox/simulink/blocks/tlc_c/timestwo.tlc
 *     matlabroot/toolbox/simulink/blocks/tlc_ada/timestwo.tlc
 *   the C and Ada TLC code to inline the S-function.
 *
 * See simulink/src/sfuntmpl_doc.c
 *
 * By Mick
 * Created on September 21, 10
 * $Revision: 1.0 $
 */

#define S_FUNCTION_NAME  desiredUxAxDynamicMaps
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"
#include <stdio.h>

#define NUM_PARAMS  5
//#define NUMOFINPUTS 2 Inputs are more complicated, look below at mdlInitializeSizes function
#define NUM_SCALAR_INPUTS 4
#define NUM_ARRAY_INPUTS 7
#define NUMOFOUTPUTS 2
#define NUM_R_WORK   5
#define g   9.81
#define desiredNoLookaheadSeg   4  // desired number of lookahead segment
#define scalingMu   1      // for straight section, make sure that the vehicle start braking early on, so that we have spare friction to track vehicle speed, in case we overestimate friction.

#define stopDstBeforeEnd   (10) // stop distance before the end of the map [m]

#define NoSegmentLong   (200) // number of divisions for numerical integration in that segment, when segment is long 
#define NoSegmentShort   (150) // number of divisions for numerical integration in that segment, when segment is short 

// Name the broken-down parameters
// #define MAP(S) ssGetSFcnParam(S, 4)
// #define numberSegment ((int)(mxGetPr(MAP(S))[5]))
// #define mapType ((int)mxGetPr(MAP(S))[6])
// 
// #define SEG(S) ssGetSFcnParam(S, 5)
// #define segType (mxGetPr(SEG(S))+numberSegment)
// #define mu (mxGetPr(SEG(S))+numberSegment*9)        // friction coefficient, in vector format for each segment, start from segment 0
// #define grade (mxGetPr(SEG(S))+numberSegment*11)    // [rad] grade, in vector format for each segment, start from segment 0, ISO coordinate
// #define bank (mxGetPr(SEG(S))+numberSegment*12)     // [rad] bank, in vector format for each segment, start from segment 0, ISO coordinate
// #define length (mxGetPr(SEG(S))+numberSegment*3)     // [m] segment length, in vector format for each segment, start from segment 0
// #define startCurvature (mxGetPr(SEG(S))+numberSegment*5)     // [1/m] start curvature, in vector format for each segment, start from segment 0
// #define endCurvature (mxGetPr(SEG(S))+numberSegment*6)     // [1/m] end curvature, in vector format for each segment, start from segment 0


// type of map
#define CLOSED        0
#define OPEN          1

#define maxMapSize ((int)(mxGetPr(ssGetSFcnParam(S, 4))[0])) // this one works

/*=======================*
 * Function prototypes *
 *=======================*/
static double straight(double segGrade, double segBank, double segMu, double finalVx, double segLength, double startOfIntegration, double segProg, SimStruct *S);
static double clothoidEntry(double m, double a, double b, double h, double segGrade, double segBank, double segMu, double finalVx, double segStartCurvature, double segEndCurvature, double segLength, double startOfIntegration, double segProg, SimStruct *S);
static double constantRadius(double m, double a, double b, double h, double segGrade, double segBank, double segMu, double finalVx, double segEndCurvature, double segLength, double startOfIntegration, double segProg, SimStruct *S);
static double clothoidExit(double m, double a, double b, double h, double segGrade, double segBank, double segMu, double finalVx, double segStartCurvature, double segEndCurvature, double segLength, double startOfIntegration, double segProg, SimStruct *S);

/*==================================================*
 * Macros to access the S-function parameter values *
 *==================================================*/

/*================*
 * Build checking *
 *================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S) {
    static int j;
    
    /*------------------------Setup Parameters---------------------*/
    
    ssSetNumSFcnParams(S, NUM_PARAMS);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }
    
    /*------------------------Setup Inputs------------------------*/
    if (!ssSetNumInputPorts(S, NUM_SCALAR_INPUTS+NUM_ARRAY_INPUTS)) return;
    
    // setup scalar inputs
    for (j=0; j<NUM_SCALAR_INPUTS; j++) {
        ssSetInputPortWidth(S, j, 1);
        //ssSetInputPortRequiredContiguous(S, j, 1);
        ssSetInputPortDirectFeedThrough(S, j, 1);
    }
    
    // setup vector inputs
    for (j=NUM_SCALAR_INPUTS; j<NUM_ARRAY_INPUTS+NUM_SCALAR_INPUTS; j++) {
        ssSetInputPortWidth(S, j, maxMapSize); 
        //ssSetInputPortRequiredContiguous(S, j, 1);
        ssSetInputPortDirectFeedThrough(S, j, 1);
    }    
    
    
    
    /*------------------------Setup Outputs------------------------*/
    
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, NUMOFOUTPUTS);
    
    /*------------------------Remaining Setup------------------------*/
    
    ssSetNumSampleTimes(S, 1);
    
    // Set the number of work variables (static module-variables)
    ssSetNumRWork(S, NUM_R_WORK);
    
    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE |
            SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove  function */
#if defined(MDL_INITIALIZE_CONDITIONS)
//Set last brake flag to 0
static void mdlInitializeConditions(SimStruct *S) {
    // Get static variables as pointers, so we can update them
    ssSetRWorkValue(S, 0, 0); // axDesired from last time step
    ssSetRWorkValue(S, 1, 0); // uxDesired from last time step
    ssSetRWorkValue(S, 2, 0); // Vx from last segment
    ssSetRWorkValue(S, 3, 0); // on final seg (of looping, so current seg, not last seg of map)
    ssSetRWorkValue(S, 4, 0); // last segment number
    
}
#endif /* MDL_INITIALIZE_CONDITIONS */

/* Function: mdlOutputs =======================================================
 * Abstract:
 * see above for definition
 */
static void mdlOutputs(SimStruct *S, int_T tid) {
    
    //need to fix this
    real_T *Y = ssGetOutputPortRealSignal(S, 0);
    
    //InputRealPtrsType in = ssGetInputPortRealSignalPtrs(S, 0);
    
    double UxDesired, AxDesired;
    int NoLookaheadSeg, counter, lookaheadSegType, lookaheadSegNumber;
    int i;
    
    
    int segNumber = (int)  (*ssGetInputPortRealSignalPtrs(S,0)[0]);    // current segment number [unitless]
    double segProgress =    *ssGetInputPortRealSignalPtrs(S,1)[0];         // distance to the next segment [m]
    int numberSegment = (int)    (*ssGetInputPortRealSignalPtrs(S,2)[0]); 
    int mapType = (int)    (*ssGetInputPortRealSignalPtrs(S,3)[0]);
    
    InputRealPtrsType segType = ssGetInputPortRealSignalPtrs(S,4);
    InputRealPtrsType mu = ssGetInputPortRealSignalPtrs(S,5);
    InputRealPtrsType bank = ssGetInputPortRealSignalPtrs(S,6);
    InputRealPtrsType grade = ssGetInputPortRealSignalPtrs(S,7);
    InputRealPtrsType startCurvature = ssGetInputPortRealSignalPtrs(S,8);
    InputRealPtrsType endCurvature = ssGetInputPortRealSignalPtrs(S,9);
    InputRealPtrsType length = ssGetInputPortRealSignalPtrs(S,10);
    

    
//     mapType=(int)*in[2];        // map type, open or closed
//     numberSegment=(int)*in[3];  // number of segments in map
//     segType=(int)*in[4];        // segment type
//     mu=
    
    // Extracting vehicle parameters
    const real_T m = mxGetPr(ssGetSFcnParam(S, 0))[0];  // vehicle mass [kg]
    const real_T a = mxGetPr(ssGetSFcnParam(S, 1))[0];  //  [m]          distance from front axle to CG;
    const real_T b = mxGetPr(ssGetSFcnParam(S, 2))[0];  //  [m]          distance from rear axle to CG;
    const real_T h = mxGetPr(ssGetSFcnParam(S, 3))[0];  //  [m]          distance from ground to CG;
  
    
    
//     // to remove, testing
//     if (numSegs != numberSegment) {
//         printf("Num Segs wrong %f %f\n",numSegs,numberSegment);
//     } 
//     
//     for (i=0; i<numberSegment; i++) {
//         if (segType[i] != *segTypePtr[i]) {printf("segType %d %d %d\n",segType[i],*segTypePtr[i],i);}
//         if (mu[i] != *segMuPtr[i]) {printf("mu %f %f %d\n",mu[i],*segMuPtr[i],i);}
//         if (bank[i] != *segBankPtr[i]) {printf("bank %f %f %d\n",bank[i],*segBankPtr[i],i);}
//         if (grade[i] != *segGradePtr[i]) {printf("grade %f %f %d\n",grade[i],*segGradePtr[i],i);}
//         if (startCurvature[i] != *segKstartPtr[i]) {printf("kStart %f %f %d\n",startCurvature[i],*segKstartPtr[i],i);}
//         if (endCurvature[i] != *segKendPtr[i]) {printf("kEnd %f %f %d\n",endCurvature[i],*segKendPtr[i],i);}
//         if (length[i] != *segLengthPtr[i]) {printf("length %f %f %d\n",length[i],*segLengthPtr[i],i);}
//         if (mapType != mapType2) {printf("maptype %d %d %d\n",mapType,mapType2,i);}
//     }
    
    //printf("mapType %i \n",mapType);
    //printf("grade %f \n",grade);
    
    // initialise value
    UxDesired=0;    // desired speed [m/s]
    ssSetRWorkValue(S, 0, 0);   // initialised AxDesired to 0
    ssSetRWorkValue(S, 3, 0);   // initialised if we are on the final segment
    AxDesired=ssGetRWorkValue(S, 0);    // desired acceleration [m/s^2]
    

    
    // If a new segment is reached, update the initial Vx at that point
    if (segNumber != ssGetRWorkValue(S, 4)) {
        ssSetRWorkValue(S, 2,ssGetRWorkValue(S, 1));
    }
    // Update work var 4 to compare with next segNumber on the next time step
        ssSetRWorkValue(S, 4, segNumber);
    
    if( mapType==OPEN ){    // open map
        //printf("open map \n");
        if (segNumber+desiredNoLookaheadSeg >= (numberSegment-1) ){  // already look beyond or at the end of the map, number segment also include segment 0, thus, need to -1
            NoLookaheadSeg=(numberSegment-1)-segNumber;   // reset the number of lookahead segment, so that it doesn't look beyond the end of the map
            
            // FIND INITIAL SPEED
            // find entry speed of the last segment
            lookaheadSegNumber=segNumber+NoLookaheadSeg;
            lookaheadSegType=(int)(*segType[lookaheadSegNumber]);
            
            //printf("lookaheadSegNumber %i, lookaheadSegType %i \n", lookaheadSegNumber, lookaheadSegType);
            
            if (NoLookaheadSeg==0){ // we are at the last segment, no more look ahead
                // this need to be calculated separately, as the start of integration is stopDstBeforeEnd meter before the end of the last segment
                
                switch (lookaheadSegType) {
                    // since we want to find the desired speed at corner entry, thus, segment progress = 0
                    case 0: UxDesired=straight(*grade[lookaheadSegNumber], *bank[lookaheadSegNumber], *mu[lookaheadSegNumber], UxDesired, *length[lookaheadSegNumber], stopDstBeforeEnd, segProgress, S);break;
                    case 1: UxDesired=clothoidEntry(m, a, b, h, *grade[lookaheadSegNumber], *bank[lookaheadSegNumber], *mu[lookaheadSegNumber], UxDesired, *startCurvature[lookaheadSegNumber], *endCurvature[lookaheadSegNumber], *length[lookaheadSegNumber], stopDstBeforeEnd, segProgress, S);break;
                    case 2: UxDesired=constantRadius(m, a, b, h, *grade[lookaheadSegNumber], *bank[lookaheadSegNumber], *mu[lookaheadSegNumber], UxDesired, *endCurvature[lookaheadSegNumber], *length[lookaheadSegNumber], stopDstBeforeEnd, segProgress, S);break;
                    case 3: UxDesired=clothoidExit(m, a, b, h, *grade[lookaheadSegNumber], *bank[lookaheadSegNumber], *mu[lookaheadSegNumber], UxDesired, *startCurvature[lookaheadSegNumber], *endCurvature[lookaheadSegNumber], *length[lookaheadSegNumber], stopDstBeforeEnd, segProgress, S);break;
                    
                    default: UxDesired=0;break; // didn't find any match
                }
            }
            else{   // we are not at the last segment
                // this need to be calculated separately, as the start of integration is stopDstBeforeEnd meter before the end of the last segment
                switch (lookaheadSegType) {
                    // since we want to find the desired speed at corner entry, thus, segment progress = 0
                    case 0: UxDesired=straight(*grade[lookaheadSegNumber], *bank[lookaheadSegNumber], *mu[lookaheadSegNumber], UxDesired, *length[lookaheadSegNumber], stopDstBeforeEnd, 0, S);break;
                    case 1: UxDesired=clothoidEntry(m, a, b, h, *grade[lookaheadSegNumber], *bank[lookaheadSegNumber], *mu[lookaheadSegNumber], UxDesired, *startCurvature[lookaheadSegNumber], *endCurvature[lookaheadSegNumber], *length[lookaheadSegNumber], stopDstBeforeEnd, 0, S);break;
                    case 2: UxDesired=constantRadius(m, a, b, h, *grade[lookaheadSegNumber], *bank[lookaheadSegNumber], *mu[lookaheadSegNumber], UxDesired, *endCurvature[lookaheadSegNumber], *length[lookaheadSegNumber], stopDstBeforeEnd, 0, S);break;
                    case 3: UxDesired=clothoidExit(m, a, b, h, *grade[lookaheadSegNumber], *bank[lookaheadSegNumber], *mu[lookaheadSegNumber], UxDesired, *startCurvature[lookaheadSegNumber], *endCurvature[lookaheadSegNumber], *length[lookaheadSegNumber], stopDstBeforeEnd, 0, S);break;
                    
                    default: UxDesired=0;break; // didn't find any match
                }
            }
            
            lookaheadSegNumber=lookaheadSegNumber-1;    // move on to the next segment
                    
        }
        else{ // the look ahead segment isn't at the end of the map
            NoLookaheadSeg=desiredNoLookaheadSeg;
            lookaheadSegNumber=segNumber+NoLookaheadSeg;
            // start integration from UxDesired=0
        }
        
        //printf("NoLookaheadSeg %i, lookaheadSegNumber %i, segNumber %i \n", NoLookaheadSeg,lookaheadSegNumber,segNumber);
                       
        //printf("UxDesired %f \n", UxDesired);
        
        // START THE LOOP HERE, now find entry speed of each lookahead corner
        for ( counter = lookaheadSegNumber; counter > segNumber; counter-- ){   // count down, and stop BEFORE hitting the current segment
            
            //printf("counter %i \n",counter);
            
            lookaheadSegType=(int)*segType[counter];      
            
            switch (lookaheadSegType) {
                // start of integration = 0, because we start integrate right from the end till the entry, thus segProgress also equal 0
                case 0: UxDesired=straight(*grade[counter], *bank[counter], *mu[counter], UxDesired, *length[counter], 0, 0, S);break;
                case 1: UxDesired=clothoidEntry(m, a, b, h, *grade[counter], *bank[counter], *mu[counter], UxDesired, *startCurvature[counter], *endCurvature[counter], *length[counter], 0, 0, S);break;
                case 2: UxDesired=constantRadius(m, a, b, h, *grade[counter], *bank[counter], *mu[counter], UxDesired, *endCurvature[counter], *length[counter], 0, 0, S);break;
                case 3: UxDesired=clothoidExit(m, a, b, h, *grade[counter], *bank[counter], *mu[counter], UxDesired, *startCurvature[counter], *endCurvature[counter], *length[counter], 0, 0, S);break;
                
                default: UxDesired=0;break; // didn't find any match                
            }    
            
            //printf("lookaheadSegType %i, counter %i, UxDesired %f \n", lookaheadSegType, counter, UxDesired);

        }        
    }
    else{   // closed map
        //printf("closed map \n");
        
        NoLookaheadSeg=desiredNoLookaheadSeg;
        
        //printf("grade[1] %f \n",grade[1]);
             
        // START THE LOOP HERE, now find entry speed of each lookahead corner
        for ( counter = (segNumber+NoLookaheadSeg); counter > segNumber; counter-- ){   // count down, and stop BEFORE hitting the current segment
            
            //printf("counter %i,numberSegment %i\n",counter,numberSegment);
            
            lookaheadSegNumber=counter; // note that in this case, lookaheadSegNumber is a variable
            if (lookaheadSegNumber>=(numberSegment-1)){ // if look too far to the next lap, fix this wrap around issue
                //printf("lookaheadSegNumber %i\n",lookaheadSegNumber);
                lookaheadSegNumber=(int)fmod((double)lookaheadSegNumber, (double)(numberSegment-1) );  // use modular calculation
                // since segment start from 0, not 1, then
                lookaheadSegNumber=lookaheadSegNumber-1;
                if (lookaheadSegNumber<0){  // if it has a wrap around, prevent negative number, add the wrap around back
                    lookaheadSegNumber=lookaheadSegNumber+numberSegment;
                }
            }
                                
            lookaheadSegType=(int)*segType[lookaheadSegNumber];
            
            //printf("lookaheadSegNumber %i \n", lookaheadSegNumber);
            //printf("lookaheadSegType %i \n",lookaheadSegType);
            //printf("grade[lookaheadSegNumber] %f \n",grade[lookaheadSegNumber]);
                                    
            switch (lookaheadSegType) {
                // start of integration = 0, because we start integrate right from the end till the entry, thus segProgress also equal 0
                case 0: UxDesired=straight(*grade[lookaheadSegNumber], *bank[lookaheadSegNumber], *mu[lookaheadSegNumber], UxDesired, *length[lookaheadSegNumber], 0, 0, S);break;
                case 1: UxDesired=clothoidEntry(m, a, b, h, *grade[lookaheadSegNumber], *bank[lookaheadSegNumber], *mu[lookaheadSegNumber], UxDesired, *startCurvature[lookaheadSegNumber], *endCurvature[lookaheadSegNumber], *length[lookaheadSegNumber], 0, 0, S);break;
                case 2: UxDesired=constantRadius(m, a, b, h, *grade[lookaheadSegNumber], *bank[lookaheadSegNumber], *mu[lookaheadSegNumber], UxDesired, *endCurvature[lookaheadSegNumber], *length[lookaheadSegNumber], 0, 0, S);break;
                case 3: UxDesired=clothoidExit(m, a, b, h, *grade[lookaheadSegNumber], *bank[lookaheadSegNumber], *mu[lookaheadSegNumber], UxDesired, *startCurvature[lookaheadSegNumber], *endCurvature[lookaheadSegNumber], *length[lookaheadSegNumber], 0, 0, S);break;
                
                default: UxDesired=0;break; // didn't find any match
            }
            //printf("In for loop, line 258 \n");
            //printf("Line 272, segNumber %i, current segment type %i \n", segNumber, (int)segType[segNumber]);
            //printf("UxDesired before current segment %f \n", UxDesired);
        }
        
        
    }

    //printf("line 267 \n");
    //printf("UxDesired %f \n", UxDesired);
    //printf("current segment type %i \n", (int)segType[segNumber]);
    //printf("NoLookaheadSeg %i \n", NoLookaheadSeg);
    
    if (NoLookaheadSeg>0){  // this is only valid when number of lookahead segment is more than 0, else, we already calculated this value at the begining
        // now calculate UxDesired and AxDesired at CURRENT segment
        
        //printf("line 287, calculate current segment, segNumber %i, current segType %i \n",segNumber,(int)segType[segNumber]);
        //printf("initial UxDesired %f \n", UxDesired);
        
        ssSetRWorkValue(S, 3, 1);   // set to 1 if we are on the final segment for exit clohtoid calculation
        
        switch ((int)*segType[segNumber]) {
            case 0: UxDesired=straight(*grade[segNumber], *bank[segNumber], scalingMu * *mu[segNumber], UxDesired, *length[segNumber], 0, segProgress, S);break;    // only say have 90% of the friction
            case 1: UxDesired=clothoidEntry(m, a, b, h, *grade[segNumber], *bank[segNumber], *mu[segNumber], UxDesired, *startCurvature[segNumber], *endCurvature[segNumber], *length[segNumber], 0, segProgress, S);break;
            case 2: UxDesired=constantRadius(m, a, b, h, *grade[segNumber], *bank[segNumber], *mu[segNumber], UxDesired, *endCurvature[segNumber], *length[segNumber], 0, segProgress, S);break;
            case 3: UxDesired=clothoidExit(m, a, b, h, *grade[segNumber], *bank[segNumber], *mu[segNumber], UxDesired, *startCurvature[segNumber], *endCurvature[segNumber], *length[segNumber], 0, segProgress, S);break;
            
            default: UxDesired=0;break; // didn't find any match
        }
    }
    
    // get AxDesired value
    AxDesired=ssGetRWorkValue(S, 0);    // desired acceleration [m/s^2]
    
    // set UxDesired for next time step for the exit clothoid function
    ssSetRWorkValue(S, 1, UxDesired);
    
    //printf("output UxDesired %f \n", UxDesired);
    //printf("output AxDesired %f \n", AxDesired);  

    Y[0]=UxDesired;        //Output desired speed at CG [m/s]
    Y[1]=AxDesired;        //Output desired acceleration at CG [m/s^2], include affect of grade (and bank)
    
}


/*
 * Function: straight
 * Abstract: calculate and return the entry speed of that segment
 */
static double straight(double segGrade, double segBank, double segMu, double finalVx, double segLength, double startOfIntegration, double segProg, SimStruct *S) {
    // startOfIntegration is the distance in [m] from the end where the integration start
    static double UxDesired, ax;
       
    // minimum possible ax (braking)
    ax= -sqrt( pow(segMu*g*cos(segBank)*cos(segGrade),2)-pow(g*sin(segBank),2) ) +g*sin(segGrade);  // ax for calculate speed profile, has effect of grade
    
    //printf("ax min in braking %f, segGrade %f \n", ax, segGrade);
    //printf("segBank %f \n", segBank);
    
    // prevent sqaure root of negative number
    if (finalVx*finalVx-2*ax*(segLength-startOfIntegration-segProg) < 0){
        UxDesired=0;    // add safety so that it cannot be a NaN
    }
    else{
        UxDesired=sqrt(finalVx*finalVx-2*ax*(segLength-startOfIntegration-segProg));
    }
      
/*
    //printf("finalVx %f, ax %f, segMu %f, segBank %f, segGrade %f \n", finalVx, ax, segMu, segBank, segGrade);
    //printf("ax in function straight %f \n", ax);
    //printf("UxDesired in straight %f \n", UxDesired);
*/
    
    ssSetRWorkValue(S, 0, ax );   // set AxDesired value 

    //printf("straight section UxDesired %f, ax %f, segLength-startOfIntegration-segProg %f \n", UxDesired, ax,(segLength-startOfIntegration-segProg) );
    
    return (UxDesired);
}

/*
 * Function: clothoidEntry
 * Abstract: calculate and return the entry speed of that segment
 */
static double clothoidEntry(double m, double a, double b, double h, double segGrade, double segBank, double segMu, double finalVx, double segStartCurvature, double segEndCurvature, double segLength, double startOfIntegration, double segProg, SimStruct *S) {
    // startOfIntegration is the distance in [m] from the end where the integration start
    static double UxDesired, c, step;
    static double constA, constB, constC, fTilde, ax, axLim, dUx_ds, ay;
    static int i;
        
    //printf("In clothoidEntry loop");
    //printf("finalVx %f \n", finalVx);
    
    c=sqrt(fabs(segEndCurvature-segStartCurvature)/(2*segLength));     // rate of Clothoid change
    UxDesired=finalVx; // initial value of the integration
    
    i=0;
    // decided integration step, to reduce computation time, prevent CPU overload
    if(segLength>1000){
        step=segLength/NoSegmentLong; // refine the step a little more
    }
    else {
        step=segLength/NoSegmentShort;
    }
    
    //printf("UxDesired %f, segLength %f, step %f, segProg %f, startOfIntegration %f \n", UxDesired,segLength,step,segProg,startOfIntegration);
        
    if ( (step+startOfIntegration) > (segLength-segProg)){ // very close to the end of the clothoid entry, need to do calculation,else, it will skip the following while loop
        // just assume that this is similar to the constant radius section
        
        //printf("line 363 \n");
        //printf("i*step+startOfIntegration %f, segLength-segProg %f \n", i*step+startOfIntegration, segLength-segProg);
        //printf("UxDesired %f \n", UxDesired);
        
        // no need to do anything with the UxDesired, as we could just pass through this information.
        
        // vehicle should have no ax at this point
        ax=0;
    }
    
    while ( (i*step+startOfIntegration) < (segLength-segProg) ) { // start the integration from the end of the corner until the entry of the corner
        
        // define constant values
        axLim=-sqrt( pow(segMu*g*cos(segBank)*cos(segGrade),2)-pow(g*sin(segBank),2) );   // negative from braking
        fTilde=1/(-1*axLim)*sqrt( pow( segMu*g*( cos(segBank)*cos(segGrade)+h*axLim/(g*a)-h*sin(segGrade)/a ), 2)-pow(g*sin(segBank), 2) );
        
        if (segEndCurvature>=0){// turning left, ay is positive
            ay=UxDesired*UxDesired*( segStartCurvature+2*c*c*(segLength-startOfIntegration-i*step) );
        }
        else{   // turning right
            ay=-UxDesired*UxDesired*( fabs(segStartCurvature)+2*c*c*(segLength-startOfIntegration-i*step) );
        }
        
        //printf("ay %f, segEndCurvature %f, UxDesired %f, s %f \n", ay, segEndCurvature,UxDesired,segLength-startOfIntegration-i*step);
        
        constA=( cos(segGrade)*cos(segBank)-sin(segGrade)*h/a-ay*sin(segBank)/g )*segMu*g;
        constB=h*segMu/a;
        constC=g*sin(segBank)+ay*cos(segBank);
        
        //printf("constA %f, constB %f, constC %f, fTilde %f \n", constA,constB,constC, fTilde);
        //printf("pow(2*constA*constB, 2)-4*(fTilde*fTilde-constB*constB)*(constC*constC-constA*constA) %f \n", pow(2*constA*constB, 2)-4*(fTilde*fTilde-constB*constB)*(constC*constC-constA*constA));
        
        // calculate ax
        if ( pow(2*constA*constB, 2)-4*(fTilde*fTilde-constB*constB)*(constC*constC-constA*constA)>0 ){
            if (constA>=0){
                ax=(2*constA*constB-sqrt(pow(2*constA*constB, 2)-4*(fTilde*fTilde-constB*constB)*(constC*constC-constA*constA)))/(2*(fTilde*fTilde-constB*constB));
            }
            else{
                ax=(-2*constA*constB-sqrt(pow(2*constA*constB, 2)-4*(fTilde*fTilde-constB*constB)*(constC*constC-constA*constA)))/(-2*(fTilde*fTilde-constB*constB));
            }
        }
        else { //bank is too high that that car is not going to make it
            ax=0;
        }
        
        //printf("ax before adding grade %f, ay %f \n", ax,ay);
        
        // add effect of grade, this is not apply to the tires, but the body of the vehicle
        ax=ax+g*sin(segGrade);  // negative grade, uphill, mean reduce ax
        
        // since we are doing backward integration, need to flip the sign of the acceleration
        ax=-ax;
        
        //printf("ax after adding grade and flip sign %f \n", ax);
        
        // update equation
        if(UxDesired==0){   // prevent singularity, avoid divided by zero
            dUx_ds=ax/1; // might create large value, i.e. at the begining of the vehicle motion, this will only increase vehicle speed by ax, which hopefully won't be too large
            //dUx_ds=0; // this might not be a good representative, as vehicle may not start moving
        }
        else{
            dUx_ds=ax/UxDesired;
        }
        
        // update maxUxEntryCorner
        UxDesired=UxDesired+step*dUx_ds;
        i=i+1;

        //printf("UxDesired %f \n", UxDesired);
        
        ax=-ax; // reset ax to the right sign convention
    }
        
    //printf("g %f \n", g);
    //printf("m %f \n", m);
    //printf("clothoid entry UxDesired %f \n", UxDesired);
    //printf("ax in function clothoidEntry %f \n", ax);
    
    ssSetRWorkValue(S, 0, ax );   // set AxDesired value, make sure that the sign is correct
    
    //printf("Exit clothoidEntry loop \n");
        
    return (UxDesired);
            
}


/*
 * Function: constantRadius
 * Abstract: calculate and return the entry speed of that segment
 * In this case, the curvature is constant, segment end curvature is equal to the start curvature
 */
static double constantRadius(double m, double a, double b, double h, double segGrade, double segBank, double segMu, double finalVx, double segEndCurvature, double segLength, double startOfIntegration, double segProg, SimStruct *S) {
    // startOfIntegration is the distance in [m] from the end where the integration start
    static double UxDesired, UxDesiredHat, c, step;
    static double constA, constB, constC, fTilde, ax, axLim, dUx_ds, ay, ayFront, ayRear;
    static int i;
        
    //printf("begin clothoid entry \n");
    //printf("segBank %f, segMu %f \n", segBank,segMu);
    
    if (segEndCurvature > 0){   // turning left
          // If limit by rear wheels
        ayRear=( cos(segGrade)*cos(segBank)*segMu*g -h/a*sin(segGrade)*segMu*g-g*sin(segBank) )/( cos(segBank)+segMu*sin(segBank) ); // when ax=0
        //printf("ayRear %f \n", ayRear);
        
        // If limit by front wheels
        ayFront=( cos(segGrade)*cos(segBank)*segMu*g +h/b*sin(segGrade)*segMu*g-g*sin(segBank) )/( cos(segBank)+segMu*sin(segBank) ); // when ax=0
        //printf("ayFront %f \n", ayFront);
        
        // choose whatever is lower, magnitude wise
        if (fabs(ayRear)>fabs(ayFront)){    ay=ayFront; }
        else {ay = ayRear;}
        
        // for safety
        if (ay < 0){    // for safety, something is wrong, set ay to zero
            ay=0;
        }
    }
    else{   // turning right
        
        // limit by rear wheels
        ayRear=( -cos(segGrade)*cos(segBank)*segMu*g +h/a*sin(segGrade)*segMu*g -g*sin(segBank) )/( cos(segBank)-segMu*sin(segBank) ); // when ax=0
        //printf("ayRear turning right %f \n", ayRear);
        //printf("-cos(segGrade)*cos(segBank)*segMu*g %f, h/a*sin(segGrade)*segMu*g %f, g*sin(segBank) %f \n", -cos(segGrade)*cos(segBank)*segMu*g, h/a*sin(segGrade)*segMu*g, g*sin(segBank) );
        //printf("-cos(segGrade)*cos(segBank)*segMu*g +h/a*sin(segGrade)*segMu*g -g*sin(segBank) %f \n", -cos(segGrade)*cos(segBank)*segMu*g +h/a*sin(segGrade)*segMu*g -g*sin(segBank) );
        
        // limit by front wheels
        ayFront=( -cos(segGrade)*cos(segBank)*segMu*g -h/b*sin(segGrade)*segMu*g -g*sin(segBank) )/( cos(segBank)-segMu*sin(segBank) ); // when ax=0
        //printf("ayFront turning right %f \n", ayFront);
        
        // choose whatever is lower, magnitude wise
        if (fabs(ayRear)>fabs(ayFront)){    ay=ayFront; }
        else {ay = ayRear;}
        
        if (ay > 0){    // for safety, something is wrong, set ay to zero
            ay=0;
        }
    }
    
    //printf("ay %f, segEndCurvature %f \n", ay,segEndCurvature);
    
    // Maximum speed through constant radius turn [m/s], radius = 1/curvature
    if ( ay/segEndCurvature >0 ) {
        UxDesiredHat=sqrt( ay/segEndCurvature );        // Maximum speed through turn [m/s], this is the start of integration value, radius = 1/curvature
        // positive ax means down hill
        ax=g*sin(segGrade);   // no longitudinal accerelation from tires
    }
    else{   // bank is too steep
        UxDesiredHat=0;
        // positive ax means down hill
        ax=g*sin(segGrade);   // no longitudinal accerelation from tires
    }
    
    //printf("ax in function constantRadius, before calculation %f \n", ax);
    //printf("UxDesiredHat  %f \n",UxDesiredHat);
    
    if(finalVx >= UxDesiredHat) {    // next segment is not the speed limitation, but the current segment is
        UxDesired=UxDesiredHat;
        
        // add effect of grade, this is not apply to the tires, but the body of the vehicle
        ax=g*sin(segGrade);  // negative grade, uphill, mean reduce ax

        //printf("ax %f \n", ax);
        
    }
    else{ // need to start integrate along the segment to see what is the safe entry speed
        // similar to Clothoid entry, as the vehicle will have to brake while doing constant radius to hit the desired exit speed for the next segment
        // same as Clothoid entry calculation, but with clothoid rate (c) = 0
        
        UxDesired=finalVx; // initial value of the integration
        
        i=0;
        // decided integration step, to reduce computation time, prevent CPU overload
        if(segLength>1000){
            step=segLength/NoSegmentLong; // refine the step a little more
        }
        else {
            step=segLength/NoSegmentShort;
        }
        
        while ( ( (i*step+startOfIntegration) < (segLength-segProg) ) && (UxDesired < UxDesiredHat) ) { // start the integration from the end of the corner until the entry of the corner
            // also make sure that the speed doesn't go beyond aciveable speed

            // define constant values
            axLim=-sqrt( pow(segMu*g*cos(segBank)*cos(segGrade),2)-pow(g*sin(segBank),2) );  // negative from braking
            fTilde=1/(-1*axLim)*sqrt( pow( segMu*g*( cos(segBank)*cos(segGrade)+h*axLim/(g*a)-h*sin(segGrade)/a ), 2)-pow(g*sin(segBank), 2) );
            if (segEndCurvature==0){// prevent error in the map, where curvature is zero
                ay=0;
            }
            else{
                ay=UxDesired*UxDesired*segEndCurvature; // this already include sign convention for the curve
            }
            constA=( cos(segGrade)*cos(segBank)-sin(segGrade)*h/a-ay*sin(segBank)/g )*segMu*g;
            constB=h*segMu/a;
            constC=g*sin(segBank)+ay*cos(segBank);
            
            // calculate ax
            if ( pow(2*constA*constB, 2)-4*(fTilde*fTilde-constB*constB)*(constC*constC-constA*constA)>0 ){
                if (constA>=0){
                    ax=(2*constA*constB-sqrt(pow(2*constA*constB, 2)-4*(fTilde*fTilde-constB*constB)*(constC*constC-constA*constA)))/(2*(fTilde*fTilde-constB*constB));
                }
                else{
                    ax=(-2*constA*constB-sqrt(pow(2*constA*constB, 2)-4*(fTilde*fTilde-constB*constB)*(constC*constC-constA*constA)))/(-2*(fTilde*fTilde-constB*constB));
                }
            }
            else { //bank is too high that that car is not going to make it
                ax=0;
            }
            
            //printf("axLim in the loop %f, ay in the loop %f \n", axLim,ay);
            //printf("ax in the loop before adding grade %f \n", ax);
                        
            // add effect of grade, this is not apply to the tires, but the body of the vehicle
            ax=ax+g*sin(segGrade);  // negative grade, uphill, mean reduce ax
            
            //printf("ax in the loop after adding grade %f, UxDesired %f \n", ax,UxDesired);
            
            // since we are doing backward integration, need to flip the sign of the acceleration, just for the backward integration, will flip this back
            ax=-ax;
                        
            //printf("ax %f \n", ax);
            
            // update equation
            if(UxDesired==0){   // prevent singularity, avoid divided by zero
                dUx_ds=ax/1; // might create large value, i.e. at the begining of the vehicle motion, this will only increase vehicle speed by ax, which hopefully won't be too large
                //dUx_ds=0; // this might not be a good representative, as vehicle may not start moving
            }
            else{
                dUx_ds=ax/UxDesired;
            }
            
            // update maxUxEntryCorner
            UxDesired=UxDesired+step*dUx_ds;
            i=i+1;
            
            //printf("UxDesired %f \n", UxDesired);
            
            ax=-ax; // reset ax to the right sign convention
        }
                
    }
    
    //printf("constant radius UxDesired %f \n", UxDesired);
    //printf("ax in function constantRadius %f \n", ax);
    
//     ssSetRWorkValue(S, 0, ax );   // set AxDesired value

    return (UxDesired);
}


/*
 * Function: clothoidExit
 * Abstract: calculate and return the entry speed of that segment
 */
static double clothoidExit(double m, double a, double b, double h, double segGrade, double segBank, double segMu, double finalVx, double segStartCurvature, double segEndCurvature, double segLength, double startOfIntegration, double segProg, SimStruct *S) {
    // startOfIntegration is the distance in [m] from the end where the integration start
    static double UxDesired, c, step, UxDesiredHat, AxDesiredHat;
    static double constA, constB, constC, fTilde, tTilde, ax, axLim, dUx_ds, ay, ayFront, ayRear;
    static int i;
   
    //printf("Enter clothoid exit \n");
    
    // decided integration step, to reduce computation time, prevent CPU overload
    if(segLength>1000){
        step=segLength/NoSegmentLong; // refine the step a little more
    }
    else {
        step=segLength/NoSegmentShort;
    }
    
    //printf("segLength %f, step %f \n", segLength,step);
    c=sqrt(fabs(segStartCurvature-segEndCurvature)/(2*segLength));     // rate of Clothoid change
    //printf("c %f\n", c);
    
    // find ay from constant radius in order to find the initial speed of the integration
    if (segStartCurvature > 0){   // turning left
        //printf("Turning left\n");
        
        // limit by rear wheels
        ayRear=( cos(segGrade)*cos(segBank)*segMu*g -h/a*sin(segGrade)*segMu*g-g*sin(segBank) )/( cos(segBank)+segMu*sin(segBank) ); // when ax=0
        //printf("ayRear turning left %f \n", ayRear);
        
        // limit by front wheels
        ayFront=( cos(segGrade)*cos(segBank)*segMu*g +h/b*sin(segGrade)*segMu*g-g*sin(segBank) )/( cos(segBank)+segMu*sin(segBank) ); // when ax=0
        //printf("ayFront turning right %f \n", ayFront);
        
        // choose whatever is lower, magnitude wise
        if (fabs(ayRear)>fabs(ayFront)){    ay=ayFront; }
        else {ay = ayRear;}
        
        // for safety
        if (ay < 0){    // for safety, something is wrong, set ay to zero
            ay=0;
        }
    }
    else{   // turning right
        //printf("Turning right\n");

        // limit by rear wheels
        ayRear=( -cos(segGrade)*cos(segBank)*segMu*g +h/a*sin(segGrade)*segMu*g -g*sin(segBank) )/( cos(segBank)-segMu*sin(segBank) ); // when ax=0
        //printf("ayRear turning right %f \n", ayRear);
        
        // limit by front wheels
        ayFront=( -cos(segGrade)*cos(segBank)*segMu*g -h/b*sin(segGrade)*segMu*g -g*sin(segBank) )/( cos(segBank)-segMu*sin(segBank) ); // when ax=0
        //printf("ayFront turning right %f \n", ayFront);
        
        // choose whatever is lower, magnitude wise
        if (fabs(ayRear)>fabs(ayFront)){    ay=ayFront; }
        else {ay = ayRear;}
        
        if (ay > 0){    // for safety, something is wrong, set ay to zero
            ay=0;
        }
    }
       
    //printf("ay from available grip %f, segStartCurvature %f \n", ay,segStartCurvature);
    
    // Maximum speed through constant radius turn [m/s], radius = 1/curvature
    if ( ay/segStartCurvature >0 && ssGetRWorkValue(S, 3)==1 ) {
        UxDesiredHat=ssGetRWorkValue(S, 2);    // velocity from previous time step [m/s^2]
    }
    else if ( ay/segStartCurvature >0 ) {
        UxDesiredHat=sqrt( ay/segStartCurvature );        // Maximum speed through turn [m/s], this is the start of integration value, radius = 1/curvature
    }
    else{   // bank is too steep
        UxDesiredHat=0;
    }
    
        // Calculate AxDesiredHat if segProg < i*step
        axLim=sqrt( pow(segMu*g*cos(segBank)*cos(segGrade),2)-pow(g*sin(segBank),2) );
        tTilde=1/axLim*sqrt( pow( segMu*g*( cos(segBank)*cos(segGrade)-h*axLim/(g*b)+h*sin(segGrade)/b ), 2)-pow(g*sin(segBank), 2) );
        if (segStartCurvature==0){// prevent error in the map, where curvature is zero
            ay=0;
        }
        else{
            
            //printf("segStartCurvature before calculate ay %f \n", segStartCurvature);
            
            if(segStartCurvature>0){    // turn left
                ay=UxDesiredHat*UxDesiredHat*( segEndCurvature+2*c*c*(segLength-segProg) );
                
                //printf("segStartCurvature %f, (segLength-i*step) %f \n", segStartCurvature,(segLength-i*step));
                
            }
            else{   // turn right
                ay=-UxDesiredHat*UxDesiredHat*( fabs(segEndCurvature)+2*c*c*(segLength-segProg) );
            }
        }
        
        //printf("ay calculating from UxDesired speed %f \n", ay);
        
        constA=( cos(segGrade)*cos(segBank)+sin(segGrade)*h/b-ay*sin(segBank)/g )*segMu*g;
        constB=h*segMu/b;
        constC=g*sin(segBank)+ay*cos(segBank);
        
        //printf("constA %f, constB %f, constC %f, tTilde %f \n", constA,constB,constC, tTilde);
        //printf("pow(2*constA*constB, 2)-4*(tTilde*tTilde-constB*constB)*(constC*constC-constA*constA) %f \n", pow(2*constA*constB, 2)-4*(tTilde*tTilde-constB*constB)*(constC*constC-constA*constA));
        
        // calculate ax, note that ax has to be positive in this case
        if ( pow(2*constA*constB, 2)-4*(tTilde*tTilde-constB*constB)*(constC*constC-constA*constA)>0 ){
            if (constA>=0){
                AxDesiredHat=(-2*constA*constB+sqrt(pow(2*constA*constB, 2)-4*(tTilde*tTilde-constB*constB)*(constC*constC-constA*constA)))/(2*(tTilde*tTilde-constB*constB));
            }
            else{
                AxDesiredHat=(2*constA*constB+sqrt(pow(2*constA*constB, 2)-4*(tTilde*tTilde-constB*constB)*(constC*constC-constA*constA)))/(-2*(tTilde*tTilde-constB*constB));
            }
        }
        else { //bank is too high that that car is not going to make it
            AxDesiredHat=0;
        }
        
        //printf("before adding grade AxDesiredHat %f \n", AxDesiredHat);
        
        // add effect of grade, this is not apply to the tires, but the body of the vehicle
        AxDesiredHat=AxDesiredHat+g*sin(segGrade);  // negative grade, uphill, mean reduce ax
                
        // just add some logic to prevent accerelation from going negative
        if (AxDesiredHat < 0){  // this assume that we will accerelate out from the exit, and that the speed profile doesn't dictate by the next segment
            // that is, accerelation in this case is always positive
            AxDesiredHat=0;
        }
        
    
    
    i=0;
    
    //printf("segProg %f \n", segProg);

    // start integration from constant radius
    // This will be a typical calculation if the next segment did not dictate the speed limit
    while (i*step < segProg) { // start the integration from the start of the corner until we are at where we are
        // limit by traction of front wheels
        
        // define constant values
        axLim=sqrt( pow(segMu*g*cos(segBank)*cos(segGrade),2)-pow(g*sin(segBank),2) );
        tTilde=1/axLim*sqrt( pow( segMu*g*( cos(segBank)*cos(segGrade)-h*axLim/(g*b)+h*sin(segGrade)/b ), 2)-pow(g*sin(segBank), 2) );
        if (segStartCurvature==0){// prevent error in the map, where curvature is zero
            ay=0;
        }
        else{
            
            //printf("segStartCurvature before calculate ay %f \n", segStartCurvature);
            
            if(segStartCurvature>0){    // turn left
                ay=UxDesiredHat*UxDesiredHat*( segEndCurvature+2*c*c*(segLength-i*step) );
                
                //printf("segStartCurvature %f, (segLength-i*step) %f \n", segStartCurvature,(segLength-i*step));
                
            }
            else{   // turn right
                ay=-UxDesiredHat*UxDesiredHat*( fabs(segEndCurvature)+2*c*c*(segLength-i*step) );
            }
        }
        
        //printf("ay calculating from UxDesired speed %f \n", ay);
        
        constA=( cos(segGrade)*cos(segBank)+sin(segGrade)*h/b-ay*sin(segBank)/g )*segMu*g;
        constB=h*segMu/b;
        constC=g*sin(segBank)+ay*cos(segBank);
        
        //printf("constA %f, constB %f, constC %f, tTilde %f \n", constA,constB,constC, tTilde);
        //printf("pow(2*constA*constB, 2)-4*(tTilde*tTilde-constB*constB)*(constC*constC-constA*constA) %f \n", pow(2*constA*constB, 2)-4*(tTilde*tTilde-constB*constB)*(constC*constC-constA*constA));
        
        // calculate ax, note that ax has to be positive in this case
        if ( pow(2*constA*constB, 2)-4*(tTilde*tTilde-constB*constB)*(constC*constC-constA*constA)>0 ){
            if (constA>=0){
                AxDesiredHat=(-2*constA*constB+sqrt(pow(2*constA*constB, 2)-4*(tTilde*tTilde-constB*constB)*(constC*constC-constA*constA)))/(2*(tTilde*tTilde-constB*constB));
            }
            else{
                AxDesiredHat=(2*constA*constB+sqrt(pow(2*constA*constB, 2)-4*(tTilde*tTilde-constB*constB)*(constC*constC-constA*constA)))/(-2*(tTilde*tTilde-constB*constB));
            }
        }
        else { //bank is too high that that car is not going to make it
            AxDesiredHat=0;
        }
        
        //printf("before adding grade AxDesiredHat %f \n", AxDesiredHat);
        
        // add effect of grade, this is not apply to the tires, but the body of the vehicle
        AxDesiredHat=AxDesiredHat+g*sin(segGrade);  // negative grade, uphill, mean reduce ax
                
        // just add some logic to prevent accerelation from going negative
        if (AxDesiredHat < 0){  // this assume that we will accerelate out from the exit, and that the speed profile doesn't dictate by the next segment
            // that is, accerelation in this case is always positive
            AxDesiredHat=0;
        }
        
        //printf("after adding grade AxDesiredHat %f \n", AxDesiredHat);
                
        // update equation
        if(UxDesiredHat==0){   // prevent singularity, see above note
            dUx_ds=AxDesiredHat/1; // might create large value, i.e. at the begining of the vehicle motion, this will only increase vehicle speed by ax, which hopefully won't be too large
            //dUx_ds=0; // this might not be a good representative, as vehicle may not start moving
        }
        else{
            dUx_ds=AxDesiredHat/UxDesiredHat;
        }
        
        // update UxDesiredHat
        UxDesiredHat=UxDesiredHat+step*dUx_ds;
        
        i=i+1;
    }
    
    
    //printf("finalVx %f \n", finalVx);
    
    UxDesired=finalVx; // initial value of the integration
    
    i=0;

    // find UxDesired that is dictate by the speed of the next segment
    while ( (i*step+startOfIntegration) < (segLength-segProg) ) { // start the integration from the end of the corner until we are at the entry of the corner
        // limit by traction of rear wheels
        
        // define constant values
        axLim=-sqrt( pow(segMu*g*cos(segBank)*cos(segGrade),2)-pow(g*sin(segBank),2) ); // negative from braking
        fTilde=1/(-1*axLim)*sqrt( pow( segMu*g*( cos(segBank)*cos(segGrade)+h*axLim/(g*a)-h*sin(segGrade)/a ), 2)-pow(g*sin(segBank), 2) );
        if (segStartCurvature==0){// prevent error in the map, where curvature is zero
            ay=0;
        }
        else{
            if(segStartCurvature>0){    // turn left
                ay=UxDesired*UxDesired*( segEndCurvature+2*c*c*(i*step+startOfIntegration));
            }
            else{   // turn right
                ay=-UxDesired*UxDesired*( fabs(segEndCurvature)+2*c*c*(i*step+startOfIntegration) );
            }
        }
        constA=( cos(segGrade)*cos(segBank)-sin(segGrade)*h/a-ay*sin(segBank)/g )*segMu*g;
        constB=h*segMu/a;
        constC=g*sin(segBank)+ay*cos(segBank);
        
        // calculate ax
        if ( pow(2*constA*constB, 2)-4*(fTilde*fTilde-constB*constB)*(constC*constC-constA*constA)>0 ){
            if (constA>=0){
                ax=(2*constA*constB-sqrt(pow(2*constA*constB, 2)-4*(fTilde*fTilde-constB*constB)*(constC*constC-constA*constA)))/(2*(fTilde*fTilde-constB*constB));
            }
            else{
                ax=(-2*constA*constB-sqrt(pow(2*constA*constB, 2)-4*(fTilde*fTilde-constB*constB)*(constC*constC-constA*constA)))/(-2*(fTilde*fTilde-constB*constB));
            }
        }
        else { //bank is too high that that car is not going to make it
            ax=0;
        }
        
        // add effect of grade, this is not apply to the tires, but the body of the vehicle
        ax=ax+g*sin(segGrade);  // negative grade, uphill, mean reduce ax
        
        // since we are doing backward integration, need to flip the sign of the acceleration
        ax=-ax;
        
        //printf("ax %f \n", ax);
        
        // update equation
        if(UxDesired==0){   // prevent singularity, avoid divided by zero
            dUx_ds=ax/1; // might create large value, i.e. at the begining of the vehicle motion, this will only increase vehicle speed by ax, which hopefully won't be too large
            //dUx_ds=0; // this might not be a good representative, as vehicle may not start moving
        }
        else{
            dUx_ds=ax/UxDesired;
        }
        
        // update maxUxEntryCorner
        UxDesired=UxDesired+step*dUx_ds;
        i=i+1;
        
        //printf("UxDesired %f \n", UxDesired);
        
        ax=-ax; // reset ax to the right sign convention       
         
    }
    
    //printf("UxDesired %f \n", UxDesired);
    //printf("compare value between UxDesired %f and UxDesiredHat %f \n", UxDesired, UxDesiredHat);
        
    if (UxDesired>UxDesiredHat){    // everything is dictate by the clothoid exit, not the speed of the next segment
        UxDesired=UxDesiredHat;
        ssSetRWorkValue(S, 0, AxDesiredHat);   // set AxDesired value
        if (fabs(AxDesiredHat) < .02 && ssGetRWorkValue(S, 3)==1 )
        {
//             printf("this is the problem \n");
        }

        //printf("Dictate by clothoid exit\n");
        //printf("Final output AxDesiredHat %f \n", AxDesiredHat);
    }   
    else{   // next segment speed dictate clothoid exit
        ssSetRWorkValue(S, 0, ax );   // set AxDesired value
                
        //printf("Dictate by next segment\n");
        //printf("Final output ax %f \n", ax);
    }
    
    //printf("g %f \n", g);
    //printf("m %f \n", m);
    //printf("clthoid exit UxDesired %f \n", UxDesired);
    //printf("ax in function clothoidExit %f \n", ax);
        
    return (UxDesired);
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S) {
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
