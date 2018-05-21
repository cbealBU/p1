/*
 * File: match_scacs2.c
 * Last Updated: 6/16/2010
 * Author: David Hoffert
 *
 *	[e,dPsi,crv,finalSeg,dst2end,
 *   curSegNum,curSegLngth,curCrvI,curCrvF,curSegProg,curMaxV,curMuBase,curGrade,curBank,
 *   nxtSegNum,nxtSegLngth,nxtCrvI,nxtCrvF,nxtMuBase,nxtGrade,nxtBank,
 *   twoSegNum,twoSegLngth,twoCrvI,twoCrvF,twoMuBase,twoGrade,twoBank,
 *   thrSegNum,thrSegLngth,thrCrvI,thrCrvF,thrMuBase,thrGrade,thrBank,
 *   numIter,wasMax,resErr,enable,guessDist,zErr]
 *      =match_scacs(posE,posN,posU,yaw,vx_cg,enable)
 *
 * Abstract:
 *	This s-function calculates the lateral and heading error from a
 *  a prescribed map given current GPS data.  Since it does so by
 *  determining the closest point that is on the map, it also reports the
 *  current curvature and whether the vehicle is approaching the end of the
 *  path (for an open path). In order to assist in racing planning, it
 *  reports information about the current, next, two-ahead and three-ahead
 *  segments of the map: segment number, segment length, initial segment
 *  curvature, final segment curvature, baseline friction coefficient,
 *  grade, and bank; for the current segment, it also reports distance
 *  along the segment and maximum allowable velocity for that segment. If
 *  the map is open (does not connect back on itself) and the vehicle is on
 *  the final, second-to-last, or third-to-last segment, the outputs
 *  describing future segments are set to -1 appropriately. For debugging
 *  and error detection, six additional outputs are provided: numIter, the
 *  number of iterations that occurred in the Newton-Raphson solver;
 *  wasMax, whether that represents the maximum number allowed; resErr, how
 *  much the computed value changed in the final iteration; enable, whether
 *  the map-matching is currently enabled; guessDist, the distance from the
 *  start of the path currently calculated; and zErr, how much the current
 *  "up" GPS reading differs from the expected value stored in the
 *  discretized map table.  This s-function matches maps of the "scacs"
 *  format (straight-clothoid-circular arc-clothoid-straight).
 *
 *  The s-function takes six inputs and ten parameters.  posE, posN, posU,
 *  and yaw come from the GPS and are used directly in map-matching.  vx_cg
 *  also comes from the GPS and is used for smarter updating of the initial
 *  guess of current position.  Enable determines whether the map-matching
 *  runs at all (for both safe initialization and saving computation time
 *  when the controller does not use map-matching.) errLim is the change in
 *  guessed distance during iterations of the Newton-Raphson solver that is
 *  small enough to declare the problem solved.  iterLim is the maximum
 *  number of iterations to perform in the Newton-Raphson, regardless of
 *  the resulting residual error (reported in all cases anyway.)  alpha,
 *  between 0 and 1 but usually between 0.7 and 0.9, limits the updated
 *  guess of the solver slightly, to improve stability.  numTerms is how
 *  many terms to use in the power series representation of clothoids. useZ
 *  is whether to consider the "up" GPS data in calculations in outputs
 *  (one may not want to since not all maps contain z-information.)  beta,
 *  between 0 and 1, is an estimate of the fraction of velocity that will
 *  be going along the path--so roughly the sideslip angle--used to better
 *  update the initial guess of current position.  map, curve, seg, and
 *  discrete are matrices containing the map information itself.
 */

// Define the s-function this code implements
#define S_FUNCTION_NAME  match_scacs2
#define S_FUNCTION_LEVEL 2

// Include necessary libraries
#include "simstruc.h"
#include <math.h>

// Define initialization constants
#define FEEDTHRU    1
#define NO_FEEDTHRU 0
#define BLOCK_SMPL  1

// Set the numbers of things in this s-function
#define NUM_PARAMS   10
#define NUM_D_STATES 0
#define NUM_C_STATES 0
#define NUM_INPUTS   2
#define NUM_OUTPUTS  6
static int INPUT_WIDTHS[]={5, 1};
static int OUTPUT_WIDTHS[]={5, 9, 7, 7, 7, 6};
#define NUM_R_WORK   1
#define NUM_P_WORK   0
#define NUM_I_WORK   1
#define NUM_MODES    0
#define NUM_NSZC     0

// Set the offset times
static double OFFSET_TIMES[]={0.0};

// Name the parameters
#define ERR_LIM_PARAM 0
#define ERR_LIM(S) ssGetSFcnParam(S,ERR_LIM_PARAM)
#define ITER_LIM_PARAM 1
#define ITER_LIM(S) ssGetSFcnParam(S,ITER_LIM_PARAM)
#define ALPHA_PARAM 2
#define ALPHA(S) ssGetSFcnParam(S,ALPHA_PARAM)
#define NUM_TERMS_PARAM 3
#define NUM_TERMS(S) ssGetSFcnParam(S,NUM_TERMS_PARAM)
#define USE_Z_PARAM 4
#define USE_Z(S) ssGetSFcnParam(S,USE_Z_PARAM)
#define BETA_PARAM 5
#define BETA(S) ssGetSFcnParam(S,BETA_PARAM)
#define MAP_PARAM 6
#define MAP(S) ssGetSFcnParam(S,MAP_PARAM)
#define CURVE_PARAM 7
#define CURVE(S) ssGetSFcnParam(S,CURVE_PARAM)
#define SEG_PARAM 8
#define SEG(S) ssGetSFcnParam(S,SEG_PARAM)
#define DISCRETE_PARAM 9
#define DISCRETE(S) ssGetSFcnParam(S,DISCRETE_PARAM)

// Name the broken-down parameters
#define E0 (mxGetPr(MAP(S))[0])
#define N0 (mxGetPr(MAP(S))[1])
#define En (mxGetPr(MAP(S))[2])
#define Nn (mxGetPr(MAP(S))[3])
#define NUM_CURVES ((int)(mxGetPr(MAP(S))[4]))
#define NUM_SEGS ((int)(mxGetPr(MAP(S))[5]))
#define mapType (mxGetPr(MAP(S))[6])
#define courseLength (mxGetPr(MAP(S))[7])

#define Ei (mxGetPr(CURVE(S))+NUM_CURVES)
#define Ni (mxGetPr(CURVE(S))+NUM_CURVES*2)
#define psiI (mxGetPr(CURVE(S))+NUM_CURVES*3)
#define Kc (mxGetPr(CURVE(S))+NUM_CURVES*4)
#define Ef (mxGetPr(CURVE(S))+NUM_CURVES*5)
#define Nf (mxGetPr(CURVE(S))+NUM_CURVES*6)
#define psiD (mxGetPr(CURVE(S))+NUM_CURVES*7)

#define segTypes (mxGetPr(SEG(S))+NUM_SEGS)
#define segCurveNums (mxGetPr(SEG(S))+NUM_SEGS*2)
#define segLengths (mxGetPr(SEG(S))+NUM_SEGS*3)
#define segCumLs (mxGetPr(SEG(S))+NUM_SEGS*4)
#define segCrvIs (mxGetPr(SEG(S))+NUM_SEGS*5)
#define segCrvFs (mxGetPr(SEG(S))+NUM_SEGS*6)
#define segMaxEs (mxGetPr(SEG(S))+NUM_SEGS*7)
#define segMaxVs (mxGetPr(SEG(S))+NUM_SEGS*8)
#define segMuBases (mxGetPr(SEG(S))+NUM_SEGS*9)
#define segMuEsts (mxGetPr(SEG(S))+NUM_SEGS*10)
#define segGrades (mxGetPr(SEG(S))+NUM_SEGS*11)
#define segBanks (mxGetPr(SEG(S))+NUM_SEGS*12)

#define discPtr (mxGetPr(DISCRETE(S)))
#define discRes (discPtr[1]-discPtr[0])
#define NUM_DISC (((int)(courseLength/discRes))+1)
#define discSs discPtr
#define discSegNums (discPtr+NUM_DISC)
#define discEasts (discPtr+NUM_DISC*2)
#define discNorths (discPtr+NUM_DISC*3)
#define discUps (discPtr+NUM_DISC*4)
#define discDists (discPtr+NUM_DISC*5)

// Defines specific to this s-function
#define CLOSED        0
#define OPEN          1
#define NOT_LAST_SEG  0
#define LAST_SEG      1
#define COS_FPS       0
#define SIN_FPS       1
#define PIE     3.14159
#define STRGHT 0
#define CLO_IN 1
#define CIR_ARC 2
#define CLO_OUT 3
#define E1 0
#define N1 1
#define E2 2
#define N2 3

// Helper function prototypes
static double fresnelPS(int trigFunc, double lowLim, double uppLim, int numTerms);
static int factorial(int n);
static double sign(double n);

/*
 * Function: mdlCheckParameters
 * Abstract: Validate the parameters to verify they are okay
 */
static void mdlCheckParameters(SimStruct *S)
{
  if (mxGetNumberOfElements(ERR_LIM(S)) != 1)
    ssSetErrorStatus(S,"errLim must be a scalar");
  if (mxGetNumberOfElements(ITER_LIM(S)) != 1)
    ssSetErrorStatus(S,"iterLim must be a scalar");
  if (mxGetNumberOfElements(ALPHA(S)) != 1)
    ssSetErrorStatus(S,"alpha must be a scalar");
  if (mxGetNumberOfElements(NUM_TERMS(S)) != 1)
    ssSetErrorStatus(S,"numTerms must be a scalar");
  if (mxGetNumberOfElements(USE_Z(S)) != 1)
    ssSetErrorStatus(S,"useZ must be a scalar");
  if (mxGetNumberOfElements(BETA(S)) != 1)
    ssSetErrorStatus(S,"beta must be a scalar");
  if (mxGetNumberOfElements(MAP(S)) != 8)
    ssSetErrorStatus(S,"map matrix must have 8 columns");
  if (mxGetNumberOfElements(CURVE(S)) != 8*NUM_CURVES)
    ssSetErrorStatus(S,"curve matrix must have 8 columns, NUM_CURVES rows");
  if (mxGetNumberOfElements(SEG(S)) != 13*NUM_SEGS)
    ssSetErrorStatus(S,"seg matrix must have 13 columns, NUM_SEGS rows");
  if (mxGetNumberOfElements(DISCRETE(S)) != 6*NUM_DISC)
    ssSetErrorStatus(S,"discrete matrix must have 6 columns");
}

/*
 * Function: mdlInitializeSizes
 * Abstract: Setup sizes of the various vectors
 */
static void mdlInitializeSizes(SimStruct *S)
{
    // Declare local variables
    int i;
    
    // Set the number of s-function parameters (constants)
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        ssSetErrorStatus(S, "Incorrect number of parameters");
        return;
    }
    
    // Set the number of discrete and continuous states (x-vector)
    ssSetNumDiscStates(S, NUM_D_STATES);
    ssSetNumContStates(S, NUM_C_STATES);
    
    // Set the number and width of inputs
    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;
    for (i=0; i<NUM_INPUTS; i++) ssSetInputPortWidth(S, i, INPUT_WIDTHS[i]);
    
    // Set feedthrough behavior for each port
    for (i=0; i<NUM_INPUTS; i++) ssSetInputPortDirectFeedThrough(S, i, FEEDTHRU);
    
    // Set the number and width of outputs
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;
    for (i=0; i<NUM_OUTPUTS; i++) ssSetOutputPortWidth(S, i, OUTPUT_WIDTHS[i]);
    
    // Set the type of sample times
    ssSetNumSampleTimes(S, BLOCK_SMPL);
    
    // Set the number of work variables (static module-variables)
    ssSetNumRWork(S, NUM_R_WORK);
    ssSetNumPWork(S, NUM_P_WORK);
    ssSetNumIWork(S, NUM_I_WORK);
    
    // Set the number of modes
    ssSetNumModes(S, NUM_MODES);
    
    // Set the number of non-sampled zero-crossings
    ssSetNumNonsampledZCs(S, NUM_NSZC);
    
    // Set options for this s-function
    // Take care when specifying exception free code - see sfuntmpl_doc.c
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}

/*
 * Function: mdlInitializeSampleTimes
 * Abstract: Specifiy that we inherit our sample time from the driving block
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    // Since we used block sample times, we only have one sample time to set
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, OFFSET_TIMES[0]);
}

/*
 * Function: mdlOutputs
 * Abstract: Perform the map-matching work
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Declare local variables
    
    // Outputs
    double e, dPsi, crv, dst2end;
    double curSegLngth, curCrvI, curCrvF, curSegProg, curMaxV, curMuBase, curGrade, curBank;
    double nxtSegLngth, nxtCrvI, nxtCrvF, nxtMuBase, nxtGrade, nxtBank;
    double twoSegLngth, twoCrvI, twoCrvF, twoMuBase, twoGrade, twoBank;
    double thrSegLngth, thrCrvI, thrCrvF, thrMuBase, thrGrade, thrBank;
    double resErr, guessDist, zErr;
    int numIter, wasMax, finalSeg;
    int curSegNum, nxtSegNum, twoSegNum, thrSegNum;
    
    // Internal variables
    double prevGuess, segStart;
    int i, guessCurve, guessSeg, guessSegNew, segType;
    double delL, totL, fracL, a1, a2, east, eastPrime, north, northPrime, psi;
    double eastStart, northStart, psi2, psiF, f, fPrime, nDelL, newDelL;
    
    // Rename outputs
    real_T *matchOut = ssGetOutputPortRealSignal(S, 0);
    real_T *curOut = ssGetOutputPortRealSignal(S, 1);
    real_T *nxtOut = ssGetOutputPortRealSignal(S, 2);
    real_T *twoOut = ssGetOutputPortRealSignal(S, 3);
    real_T *thrOut = ssGetOutputPortRealSignal(S, 4);
    real_T *debugOut = ssGetOutputPortRealSignal(S, 5);
    
    // Get inputs
    InputRealPtrsType in1 = ssGetInputPortRealSignalPtrs(S, 0);
    double posE = *in1[0];
    double posN = *in1[1];
    double posU = *in1[2];
    double yaw  = *in1[3];
    double vx_cg = *in1[4];
    InputRealPtrsType in2 = ssGetInputPortRealSignalPtrs(S, 1);
    int enable = *in2[0];
    
    // Get static variables as pointers, so we can update them
    real_T *lastDist = ssGetRWork(S);
    int_T *lastSeg = ssGetIWork(S);
    
    // Get parameters
    // The function mxGetPr is mysteriously required to dereference the pointer
    const real_T errLim = *mxGetPr(ERR_LIM(S));
    const int_T iterLim = (const int_T)(*mxGetPr(ITER_LIM(S)));
    const real_T alpha = *mxGetPr(ALPHA(S));
    const int_T numTerms = (const int_T)(*mxGetPr(NUM_TERMS(S)));
    const int_T useZ = (const int_T)(*mxGetPr(USE_Z(S)));
    const real_T beta = *mxGetPr(BETA(S));
    // For the matrix parameters, we'll just use the #defines

    // Only perform map-matching if map-matching is enabled
    if (enable)
    {
        // Declare map scalars and arrays
        double *cumL = (double*) calloc(NUM_SEGS+1, sizeof(double));
        double *straightsE1 = (double*) calloc(NUM_CURVES+1, sizeof(double));
        double *straightsE2 = (double*) calloc(NUM_CURVES+1, sizeof(double));
        double *straightsN1 = (double*) calloc(NUM_CURVES+1, sizeof(double));
        double *straightsN2 = (double*) calloc(NUM_CURVES+1, sizeof(double));
        
        // Initialize map arrays
        for (i = 0; i < NUM_SEGS; i++)
        {
            cumL[i] = segCumLs[i];
        }
        cumL[NUM_SEGS] = courseLength;
        
        straightsE1[0] = E0;
        straightsN1[0] = N0;
        straightsE2[NUM_CURVES] = En;
        straightsN2[NUM_CURVES] = Nn;
        for (i = 0; i < NUM_CURVES; i++)
        {
            straightsE1[i+1] = Ef[i];
            straightsN1[i+1] = Nf[i];
            straightsE2[i] = Ei[i];
            straightsN2[i] = Ni[i];
        }
        
        // The initial guess is the distance along the curve from last time
        // Work vectors are initialized to zero, so the first guess will be 0
        guessDist = *lastDist;
        guessSeg = *lastSeg;
        
        // Initialize variables
        finalSeg = NOT_LAST_SEG;
        
        // Use Newton-Raphson to come up with a better guess
        resErr = 2*errLim;
        numIter = 0;
        while (resErr >= errLim && numIter < iterLim)
        {
            // Duplicate the guess value so we can find resErr later
            prevGuess = guessDist;
            
            // Find the segment type and starting distance
            guessCurve = guessSeg/4;
            segType = (int)fmod((double)guessSeg, 4.0);
            segStart = cumL[guessSeg];
            
            // Find E, E', N, N', psi, crv at this distance
            switch (segType)
            {
                // Straight-line segment
                case STRGHT:
                    
                    // Find distance traveled to segment length ratio
                    delL = guessDist - segStart;
                    totL = segLengths[guessSeg];
                    fracL = delL/totL;
                    
                    // Find required parameters
                    east = fracL*(straightsE2[guessCurve]-straightsE1[guessCurve])+straightsE1[guessCurve];
                    eastPrime = (straightsE2[guessCurve]-straightsE1[guessCurve])/totL;
                    north = fracL*(straightsN2[guessCurve]-straightsN1[guessCurve])+straightsN1[guessCurve];
                    northPrime = (straightsN2[guessCurve]-straightsN1[guessCurve])/totL;
                    if (guessCurve < NUM_CURVES) psi = psiI[guessCurve];
                    else psi = psiI[NUM_CURVES-1]+psiD[NUM_CURVES-1];
                    crv = 0;
                    break;
                    
                // Clothoid from zero curvature segment
                case CLO_IN:
                    
                    // Find preliminary parameters
                    delL = guessDist - segStart;
                    a1 = sqrt(fabs(Kc[guessCurve])/(2*segLengths[guessSeg]));
                    
                    // Find required parameters
                    east = Ei[guessCurve]+(cos(psiI[guessCurve])/a1)*fresnelPS(COS_FPS, 0, a1*delL, numTerms)
                           -(sign(Kc[guessCurve])*sin(psiI[guessCurve])/a1)*fresnelPS(SIN_FPS, 0, a1*delL, numTerms);
                    north = Ni[guessCurve]+(sign(Kc[guessCurve])*cos(psiI[guessCurve])/a1)*fresnelPS(SIN_FPS, 0, a1*delL, numTerms)
                            +(sin(psiI[guessCurve])/a1)*fresnelPS(COS_FPS, 0, a1*delL, numTerms);
                    psi = (Kc[guessCurve]/(2*segLengths[guessSeg]))*pow(delL, 2.0) + psiI[guessCurve];
                    crv = (Kc[guessCurve]/segLengths[guessSeg])*delL;
                    eastPrime = cos(psi);
                    northPrime = sin(psi);
                    break;
                    
                // Circular arc segment
                case CIR_ARC:
                    
                    // Go through the entry clothoid calculation to find the start E and N coordinates
                    delL = segLengths[guessSeg-1];
                    a1 = sqrt(fabs(Kc[guessCurve])/(2*segLengths[guessSeg-1]));
                    eastStart = Ei[guessCurve]+(cos(psiI[guessCurve])/a1)*fresnelPS(COS_FPS, 0, a1*delL, numTerms)
                                -(sign(Kc[guessCurve])*sin(psiI[guessCurve])/a1)*fresnelPS(SIN_FPS, 0, a1*delL, numTerms);
                    northStart = Ni[guessCurve]+(sign(Kc[guessCurve])*cos(psiI[guessCurve])/a1)*fresnelPS(SIN_FPS, 0, a1*delL, numTerms)
                                 +(sin(psiI[guessCurve])/a1)*fresnelPS(COS_FPS, 0, a1*delL, numTerms);
                    psi2 = (Kc[guessCurve]/(2*segLengths[guessSeg-1]))*pow(delL, 2.0) + psiI[guessCurve];
                    
                    // Find preliminary parameters
                    delL = guessDist - segStart;
                    
                    // Find required parameters
                    east = eastStart + (sin(delL*Kc[guessCurve]+psi2)-sin(psi2))/Kc[guessCurve];
                    north = northStart - (cos(delL*Kc[guessCurve]+psi2)-cos(psi2))/Kc[guessCurve];
                    psi = delL*Kc[guessCurve]+psi2;
                    crv = Kc[guessCurve];
                    eastPrime = cos(psi);
                    northPrime = sin(psi);
                    break;
                    
                // Clothoid to zero curvature segment
                case CLO_OUT:
                    
                    // Go through the entry clothoid and circular arc calculations to find the start E and N coordinates
                    psiF = psiI[guessCurve]+psiD[guessCurve];
                    delL = segLengths[guessSeg-2];
                    a1 = sqrt(fabs(Kc[guessCurve])/(2*segLengths[guessSeg-2]));
                    eastStart = Ei[guessCurve]+(cos(psiI[guessCurve])/a1)*fresnelPS(COS_FPS, 0, a1*delL, numTerms)
                                -(sign(Kc[guessCurve])*sin(psiI[guessCurve])/a1)*fresnelPS(SIN_FPS, 0, a1*delL, numTerms);
                    northStart = Ni[guessCurve]+(sign(Kc[guessCurve])*cos(psiI[guessCurve])/a1)*fresnelPS(SIN_FPS, 0, a1*delL, numTerms)
                                 +(sin(psiI[guessCurve])/a1)*fresnelPS(COS_FPS, 0, a1*delL, numTerms);
                    psi2 = (Kc[guessCurve]/(2*segLengths[guessSeg-2]))*pow(delL, 2.0) + psiI[guessCurve];
                    delL = segLengths[guessSeg-1];
                    eastStart += (sin(delL*Kc[guessCurve]+psi2)-sin(psi2))/Kc[guessCurve];
                    northStart -= (cos(delL*Kc[guessCurve]+psi2)-cos(psi2))/Kc[guessCurve];
                    
                    // Find preliminary parameters
                    delL = guessDist - segStart;
                    nDelL = segLengths[guessSeg] - delL;
                    a2 = sqrt(fabs(Kc[guessCurve])/(2*segLengths[guessSeg]));
                    
                    // Find required parameters
                    east = eastStart + (cos(psiF)/a2)*fresnelPS(COS_FPS, a2*nDelL, a2*segLengths[guessSeg], numTerms)
                           +(sign(Kc[guessCurve])*sin(psiF)/a2)*fresnelPS(SIN_FPS, a2*nDelL, a2*segLengths[guessSeg], numTerms);
                    north = northStart - (sign(Kc[guessCurve])*cos(psiF)/a2)*fresnelPS(SIN_FPS, a2*nDelL, a2*segLengths[guessSeg], numTerms)
                            +(sin(psiF)/a2)*fresnelPS(COS_FPS, a2*nDelL, a2*segLengths[guessSeg], numTerms);
                    psi = psiF - (Kc[guessCurve]/(2*segLengths[guessSeg]))*pow(nDelL, 2.0);
                    crv = (Kc[guessCurve]/segLengths[guessSeg])*nDelL;
                    eastPrime = cos(psi);
                    northPrime = sin(psi);
                    break;
            }
            
            // The closest point is perpendicular to that point's heading
            // So we set up a Newton-Raphson update on this to update our guess
            // Thanks to Mark Malhotra for coming up with the dot product derivation
            while (psi > PIE) psi -= 2*PIE;
            while (psi <= -PIE) psi += 2*PIE;
            f = (posN - north)*sin(psi) + (posE - east)*cos(psi);
            fPrime = -northPrime*sin(psi) + (posN - north)*crv*cos(psi) - eastPrime*cos(psi) - (posE - east)*crv*sin(psi);
            newDelL = delL - alpha*f/fPrime;
            guessDist = newDelL + segStart;
            
            // Find resErr, update numIter
            resErr = fabs(guessDist - prevGuess);
            numIter++;
            
            // Make sure the updated guess is actually on the path
            // For an open map, this would mean we quit now
            if (mapType == CLOSED)
            {
                while (guessDist >= courseLength) guessDist -= courseLength;
                while (guessDist < 0) guessDist += courseLength;
            }
            else
            {
                if (guessDist >= courseLength)
                {
                    guessDist = courseLength - errLim;
                    guessSeg = NUM_SEGS - 1;
                    east = straightsE2[NUM_CURVES];
                    eastPrime = (straightsE2[NUM_CURVES]-straightsE1[NUM_CURVES])/totL;
                    north = straightsN2[NUM_CURVES];
                    northPrime = (straightsN2[NUM_CURVES]-straightsN1[NUM_CURVES])/totL;
                    psi = psiI[NUM_CURVES-1]+psiD[NUM_CURVES-1];
                    crv = 0;
                    break;
                }
                
                if (guessDist < 0)
                {
                    guessDist = 0;
                    guessSeg = 0;
                    east = straightsE1[0];
                    eastPrime = (straightsE2[0]-straightsE1[0])/totL;
                    north = straightsN1[0];
                    northPrime = (straightsN2[0]-straightsN1[0])/totL;
                    psi = psiI[0];
                    crv = 0;
                    break;
                }
            }
            
            // Look-up which segment this distance corresponds to
            if (!(guessDist >= cumL[guessSeg] && guessDist < cumL[guessSeg+1]))
            {
                // Search closer segments before further segments
                for (i=1;i<(floor(NUM_SEGS/2)+1);i++)
                {
                    // Look one segment further ahead, if we can
                    guessSegNew = guessSeg+i;
                    while (guessSegNew < 0) guessSegNew += NUM_SEGS;
                    while (guessSegNew >= NUM_SEGS) guessSegNew -= NUM_SEGS;
                    if (guessDist >= cumL[guessSegNew] && guessDist < cumL[guessSegNew+1])
                    {
                        guessSeg = guessSegNew;
                        break;
                    }
                    
                    // Look one segment further behind, if we can
                    guessSegNew = guessSeg-i;
                    while (guessSegNew < 0) guessSegNew += NUM_SEGS;
                    while (guessSegNew >= NUM_SEGS) guessSegNew -= NUM_SEGS;
                    if (guessDist >= cumL[guessSegNew] && guessDist < cumL[guessSegNew+1])
                    {
                        guessSeg = guessSegNew;
                        break;
                    }
                }
                
                // We don't need to worry about never finding the segment
                // If the distance was out of range, that would have been
                //      caught in the loops for distance, not segment
            }
        }
        
        // Find the segment type and starting distance
        guessCurve = guessSeg/4;
        segType = (int)fmod((double)guessSeg, 4.0);
        segStart = cumL[guessSeg];
        
        // If the segment has changed, newDelL needs to be updated
        newDelL = guessDist - segStart;
        
        // Now that we have the closest point, find the outputs we need
        
        // Magnitude of lateral error is just the distance formula
        // Sign of lateral error is + if left of road, - if right of road
        e = sqrt(pow(posE - east, 2.0)+pow(posN - north, 2.0));
        if (((posE-east)*northPrime-(posN-north)*eastPrime)>0) e *= -1;
        
        // This code assumes the GPS yaw angle increases counter-clockwise and
        // is measured from the EAST axis (true for Applanix, not P1/X1)
        // Also, make sure the final answer is between -pi and pi
        dPsi = yaw - psi;
        while (dPsi > PIE) dPsi -= 2*PIE;
        while (dPsi <= -PIE) dPsi += 2*PIE;
        
        // Check if we are on the last segment of an open map
        if (mapType == OPEN && guessCurve == NUM_CURVES) finalSeg = LAST_SEG;
        
        // For an open map, report the distance to the end of the map
        if (mapType == OPEN) dst2end = courseLength - guessDist;
        else dst2end = -1;
        
        // Find outputs that pertain to current segment
        curSegNum = guessSeg;
        curSegLngth = segLengths[curSegNum];
        curCrvI = segCrvIs[curSegNum];
        curCrvF = segCrvFs[curSegNum];
        curSegProg = newDelL;
        curMaxV = segMaxVs[curSegNum];
        curMuBase = segMuBases[curSegNum];
        curGrade = segGrades[curSegNum];
        curBank = segBanks[curSegNum];
        
        // Find outputs that pertain to next segment
        
        // Make sure we aren't looking for a next segment that doesn't exist
        if (mapType == OPEN && guessSeg >= NUM_SEGS-1)
        {
            nxtSegNum = -1;
            nxtSegLngth = -1.0;
            nxtCrvI = -1.0;
            nxtCrvF = -1.0;
            nxtMuBase = -1.0;
            nxtGrade = -1.0;
            nxtBank = -1.0;
        }
        else
        {
            nxtSegNum = guessSeg + 1;
            while (nxtSegNum >= NUM_SEGS) nxtSegNum -= NUM_SEGS;
            while (nxtSegNum < 0) nxtSegNum += NUM_SEGS;
            nxtSegLngth = segLengths[nxtSegNum];
            nxtCrvI = segCrvIs[nxtSegNum];
            nxtCrvF = segCrvFs[nxtSegNum];
            nxtMuBase = segMuBases[nxtSegNum];
            nxtGrade = segGrades[nxtSegNum];
            nxtBank = segBanks[nxtSegNum];
        }
        
        // Find outputs that pertain to two segments ahead
        
        // Make sure we aren't looking for a two-ahead segment that doesn't exist
        if (mapType == OPEN && guessSeg >= NUM_SEGS-2)
        {
            twoSegNum = -1;
            twoSegLngth = -1.0;
            twoCrvI = -1.0;
            twoCrvF = -1.0;
            twoMuBase = -1.0;
            twoGrade = -1.0;
            twoBank = -1.0;
        }
        else
        {
            twoSegNum = guessSeg + 2;
            while (twoSegNum >= NUM_SEGS) twoSegNum -= NUM_SEGS;
            while (twoSegNum < 0) twoSegNum += NUM_SEGS;
            twoSegLngth = segLengths[twoSegNum];
            twoCrvI = segCrvIs[twoSegNum];
            twoCrvF = segCrvFs[twoSegNum];
            twoMuBase = segMuBases[twoSegNum];
            twoGrade = segGrades[twoSegNum];
            twoBank = segBanks[twoSegNum];
        }
        
        // Find outputs that pertain to three segments ahead
        
        // Make sure we aren't looking for a three-ahead segment that doesn't exist
        if (mapType == OPEN && guessSeg >= NUM_SEGS-3)
        {
            thrSegNum = -1;
            thrSegLngth = -1.0;
            thrCrvI = -1.0;
            thrCrvF = -1.0;
            thrMuBase = -1.0;
            thrGrade = -1.0;
            thrBank = -1.0;
        }
        else
        {
            thrSegNum = guessSeg + 3;
            while (thrSegNum >= NUM_SEGS) thrSegNum -= NUM_SEGS;
            while (thrSegNum < 0) thrSegNum += NUM_SEGS;
            thrSegLngth = segLengths[thrSegNum];
            thrCrvI = segCrvIs[thrSegNum];
            thrCrvF = segCrvFs[thrSegNum];
            thrMuBase = segMuBases[thrSegNum];
            thrGrade = segGrades[thrSegNum];
            thrBank = segBanks[thrSegNum];
        }
        
        // Last, set the wasMax flag and zErr value
        wasMax = (numIter == iterLim);
        zErr = 0.0;
        
        // Update static variables and set outputs
        *lastDist = guessDist;
        *lastSeg = guessSeg;
        
        matchOut[0] = e;
        matchOut[1] = dPsi;
        matchOut[2] = crv;
        matchOut[3] = finalSeg;
        matchOut[4] = dst2end;
        
        curOut[0] = curSegNum;
        curOut[1] = curSegLngth;
        curOut[2] = curCrvI;
        curOut[3] = curCrvF;
        curOut[4] = curSegProg;
        curOut[5] = curMaxV;
        curOut[6] = curMuBase;
        curOut[7] = curGrade;
        curOut[8] = curBank;
        
        nxtOut[0] = nxtSegNum;
        nxtOut[1] = nxtSegLngth;
        nxtOut[2] = nxtCrvI;
        nxtOut[3] = nxtCrvF;
        nxtOut[4] = nxtMuBase;
        nxtOut[5] = nxtGrade;
        nxtOut[6] = nxtBank;
        
        twoOut[0] = twoSegNum;
        twoOut[1] = twoSegLngth;
        twoOut[2] = twoCrvI;
        twoOut[3] = twoCrvF;
        twoOut[4] = twoMuBase;
        twoOut[5] = twoGrade;
        twoOut[6] = twoBank;
        
        thrOut[0] = thrSegNum;
        thrOut[1] = thrSegLngth;
        thrOut[2] = thrCrvI;
        thrOut[3] = thrCrvF;
        thrOut[4] = thrMuBase;
        thrOut[5] = thrGrade;
        thrOut[6] = thrBank;
        
        debugOut[0] = numIter;
        debugOut[1] = wasMax;
        debugOut[2] = resErr;
        debugOut[3] = enable;
        debugOut[4] = guessDist;
        debugOut[5] = zErr;
        
        // Free the calloc'd pointers to avoid memory leaks
        free(cumL);
        free(straightsE1);
        free(straightsN1);
        free(straightsE2);
        free(straightsN2);
    }
    else // Map-matching is not enabled
    {
        *lastDist = 0;
        *lastSeg = 0;
        
        matchOut[0] = 0;
        matchOut[1] = 0;
        matchOut[2] = 0;
        matchOut[3] = 0;
        matchOut[4] = 0;
        
        curOut[0] = 0;
        curOut[1] = 0;
        curOut[2] = 0;
        curOut[3] = 0;
        curOut[4] = 0;
        curOut[5] = 0;
        curOut[6] = 0;
        curOut[7] = 0;
        curOut[8] = 0;
        
        nxtOut[0] = 0;
        nxtOut[1] = 0;
        nxtOut[2] = 0;
        nxtOut[3] = 0;
        nxtOut[4] = 0;
        nxtOut[5] = 0;
        nxtOut[6] = 0;
        
        twoOut[0] = 0;
        twoOut[1] = 0;
        twoOut[2] = 0;
        twoOut[3] = 0;
        twoOut[4] = 0;
        twoOut[5] = 0;
        twoOut[6] = 0;
        
        thrOut[0] = 0;
        thrOut[1] = 0;
        thrOut[2] = 0;
        thrOut[3] = 0;
        thrOut[4] = 0;
        thrOut[5] = 0;
        thrOut[6] = 0;
        
        debugOut[0] = 0;
        debugOut[1] = 0;
        debugOut[2] = 0;
        debugOut[3] = 0;
        debugOut[4] = 0;
        debugOut[5] = 0;
    }
}

/*
 * Function: fresnelPS
 * Abstract: Helper function to calculate the power series approximation
 *           to a Fresnel integral
 */
static double fresnelPS(int trigFunc, double lowLim, double uppLim, int numTerms)
{
    // Declare local variables
    double fpsLow, fpsUpp;
    int i;
    
    // Choose the Fresnel integral type
    switch (trigFunc)
    {
        // The trig function is cos
        case COS_FPS:
            
            // Set the summation variables to zero to prepare for a summation
            fpsLow = 0;
            fpsUpp = 0;
            
            // Sum the given number of terms of the cos expansion solution
            for (i=0; i<numTerms; i++)
            {
                fpsLow += pow(-1.0, (double)i)*pow(lowLim, (double)(i*4+1))/((i*4+1)*factorial(i*2));
                fpsUpp += pow(-1.0, (double)i)*pow(uppLim, (double)(i*4+1))/((i*4+1)*factorial(i*2));
            }
            
            // Set the output variable with the difference of these sums
            return fpsUpp - fpsLow;
            break;
            
        // The trig function is sin
        case SIN_FPS:
            
            // Set the summation variables to zero to prepare for a summation
            fpsLow = 0;
            fpsUpp = 0;
            
            // Sum the given number of terms of the sin expansion solution
            for (i=0; i<numTerms; i++)
            {
                fpsLow += pow(-1.0, (double)i)*pow(lowLim, (double)(i*4+3))/((i*4+3)*factorial(i*2+1));
                fpsUpp += pow(-1.0, (double)i)*pow(uppLim, (double)(i*4+3))/((i*4+3)*factorial(i*2+1));
            }
            
            // Set the output variable with the difference of these sums
            return fpsUpp - fpsLow;
            break;
    }
}

/*
 * Function: factorial
 * Abstract: Returns the factorial of the argument n, where factorial
 *           is defined as the product of all integers from 1 up to n.
 *           The factorial of 0 is 1, and does not exist for negative
 *           numbers.
 */
static int factorial(int n)
{
    // Declare local variables
    int product, i;
    
    // Check for reasonable inputs
    if (n < 0) return -1;
    if (n == 0) return 1;
    
    // Set the product variable to 1 to prepare for multiplying
    product = 1;
    
    // Calculate the factorial
    for (i = 1; i <= n; i++) product *= i;
    return product;
}

/*
 * Function: sign
 * Abstract: Returns 1, 0, or -1, depending on whether the input is
 *           positive, zero, or negative, respectively.
 */
static double sign(double n)
{
    if (n < 0) return -1;
    else if (n == 0) return 0;
    else return 1;
}

/*
 * Function: mdlTerminate
 * Abstract: No termination needed, but we are required to have this routine
 */
static void mdlTerminate(SimStruct *S)
{
}

/*=============================*
 * Required S-function trailer *
 *=============================*/
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
