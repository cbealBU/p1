/*
 * File: match_scacs.c
 * Last Updated: 5/12/2010
 * Author: David Hoffert
 *
 *	[e,dPsi,crv,finalSeg,dst2end,
 *   currCrvNum,currSegType,currSegLngth,currSegProg,nxtDst,currCrvI,currCrvF,
 *   nxtCrvNum,nxtSegType,nxtSegLngth,nxtCrvI,nxtCrvF,
 *   twoCrvNum,twoSegType,twoSegLngth,twoCrvI,twoCrvF,
 *   numIter,wasMax,resErr,enable,guessDist,
 *   thrCrvNum,thrSegType,thrSegLngth,thrCrvI,thrCrvF]
 *      =match_scacs(posE,posN,yaw,vx_cg,enable,initSeg)
 *
 * Abstract:
 *	This s-function calculates the lateral and heading error from a
 *  a prescribed map given current GPS data.  Since it does so by
 *  determining the closest point that is on the map, it also reports the
 *  current curvature and whether the vehicle is approaching the extremes
 *  of the path (for an open path.) In order to assist in racing planning,
 *  it reports information about the current, next, two-ahead and three-
 *  ahead segments of the map: curve number, segment type, segment length,
 *  initial segment curvature, and final segment curvature; for the current
 *  segment, it also reports distance along the segment and distance until
 *  the next segment (redundant information current segment length was also
 *  provided.) If the map is open (does not connect back on itself) and the
 *  vehicle is on the final, second-to-last, or third-to-last segment, the
 *  outputs describing future segments are set to -1 appropriately. For
 *  debugging, five additional outputs are provided: numIter, the number of
 *  iterations that occurred in the Newton-Raphson solver; wasMax, whether
 *  that represents the maximum number allowed; resErr, how much the computed
 *  value changed in the final iteration; enable, whether the map-matching
 *  is currently enabled; and guessDist, the distance from the start of the
 *  path currently calculated.  This s-function matches maps of the "scacs"
 *  format (straight-clothoid-circular arc-clothoid-straight).
 *
 *  The s-function takes seven inputs and four parameters.  posE, posN, and
 *  yaw come from the GPS and are used directly in map-matching.  vx_cg
 *  also comes from the GPS and is used for smarter updating of the initial
 *  guess of current position.  Enable determines whether the map-matching
 *  runs at all (for both safe initialization and saving computation time
 *  when the controller does not use map-matching.)  initSeg allows the
 *  user to start the vehicle somewhere other than segment 0 of the map.
 *  errLim is the change in guessed distance during iterations of the
 *  Newton-Raphson solver that is small enough to declare the problem
 *  solved.  iterLim is the maximum number of iterations to perform in the
 *  Newton-Raphson, regardless of the resulting residual error (reported
 *  in all cases anyway.)  alpha, between 0 and 1 but usually between 0.7
 *  and 0.9, limits the updated guess of the solver slightly, to improve
 *  stability.  numTerms is how many terms to use in the power series
 *  representation of clothoids.
 */

// Define the s-function this code implements
#define S_FUNCTION_NAME  match_scacs
#define S_FUNCTION_LEVEL 2

// Include necessary libraries
#include "simstruc.h"
#include <math.h>

// Define initialization constants
#define FEEDTHRU    1
#define NO_FEEDTHRU 0
#define BLOCK_SMPL  1

// Set the numbers of things in this s-function
#define NUM_PARAMS   4
#define NUM_D_STATES 0
#define NUM_C_STATES 0
#define NUM_INPUTS   2
#define NUM_OUTPUTS  6
static int INPUT_WIDTHS[]={4, 2};
static int OUTPUT_WIDTHS[]={5, 7, 5, 5, 5, 5};
#define NUM_R_WORK   1
#define NUM_P_WORK   0
#define NUM_I_WORK   1
#define NUM_MODES    0
#define NUM_NSZC     0

// Set the offset times
static double OFFSET_TIMES[]={0.0};

// Defines specific to this s-function
#define CLOSED        0
#define OPEN          1
#define NOT_LAST_SEG  0
#define LAST_SEG      1
#define COS_FPS       0
#define SIN_FPS       1
#define PIE     3.14159

// Helper function prototypes
static double fresnelPS(int trigFunc, double lowLim, double uppLim, int numTerms);
static int factorial(int n);
static double sign(double n);

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
    double e, dPsi, crv, dst2end, currSegLngth, currSegProg, nxtDst, currCrvI, currCrvF;
    double nxtSegLngth, nxtCrvI, nxtCrvF, twoSegLngth, twoCrvI, twoCrvF, thrSegLngth, thrCrvI, thrCrvF, resErr;
    int finalSeg, currCrvNum, currSegType, nxtSegNum, nxtCrvNum, nxtSegType, twoSegNum, twoCrvNum, twoSegType, thrSegNum, thrCrvNum, thrSegType, numIter, wasMax;
    double courseLength, guessDist, prevGuess, segStart;
    int i, mapType, guessCurve, guessSeg, guessSegNew, segType;
    double delL, totL, fracL, a1, a2, east, eastPrime, north, northPrime, psi;
    double eastStart, northStart, psi2, psiF, f, fPrime, nDelL, newDelL;
    
    // Rename outputs
    real_T *matchOut = ssGetOutputPortRealSignal(S, 0);
    real_T *currOut = ssGetOutputPortRealSignal(S, 1);
    real_T *nextOut = ssGetOutputPortRealSignal(S, 2);
    real_T *twoOut = ssGetOutputPortRealSignal(S, 3);
    real_T *debugOut = ssGetOutputPortRealSignal(S, 4);
    real_T *thrOut = ssGetOutputPortRealSignal(S, 5);
    
    // Get inputs
    InputRealPtrsType in1 = ssGetInputPortRealSignalPtrs(S, 0);
    double posE = *in1[0];
    double posN = *in1[1];
    double yaw  = *in1[2];
    double vx_cg = *in1[3];
    InputRealPtrsType in2 = ssGetInputPortRealSignalPtrs(S, 1);
    int enable = *in2[0];
    int initSeg = *in2[1];
    
    // Get static variables as pointers, so we can update them
    real_T *lastDist = ssGetRWork(S);
    int_T *lastSeg = ssGetIWork(S);
    
    // Get parameters
    // The function mxGetPr is mysteriously required to dereference the pointer
    const real_T errLim = *mxGetPr(ssGetSFcnParam(S, 0));
    const int_T iterLim = (const int_T)(*mxGetPr(ssGetSFcnParam(S, 1)));
    const real_T alpha = *mxGetPr(ssGetSFcnParam(S, 2));
    const int_T numTerms = (const int_T)(*mxGetPr(ssGetSFcnParam(S, 3)));
    
    /********************************
     * BEGIN PUT MAP PARAMETERS HERE
     *******************************/
    
            // fairgrounds_press3

    // Declare map defines
    #define NUM_CURVES 4
    #define NUM_SEGS 17
    #define STRGHT 0
    #define CLO_IN 1
    #define CIR_ARC 2
    #define CLO_OUT 3
    #define E1 0
    #define N1 1
    #define E2 2
    #define N2 3
    #define E_OFF 0
    #define N_OFF 0

    // Declare map scalars and arrays
    double E0 = 29672.630000+E_OFF;
    double N0 = -14582.590000+N_OFF;
    double En = 29672.630000+E_OFF;
    double Nn = -14582.590000+N_OFF;
    double LstrF = 0.000000;
    double Ei[NUM_CURVES];
    double Ni[NUM_CURVES];
    double psiI[NUM_CURVES];
    double Kc[NUM_CURVES];
    double Ef[NUM_CURVES];
    double Nf[NUM_CURVES];
    double psiD[NUM_CURVES];
    double lengths[NUM_CURVES][4];
    double straights[NUM_CURVES+1][4];
    double cumL[NUM_SEGS+1];

    // Only perform map-matching if map-matching is enabled
    if (enable)
    {
        // Initialize map arrays
        Ei[0]=29633.229462+E_OFF; Ni[0]=-14613.373074+N_OFF; psiI[0]=3.804818;
        Ei[1]=29665.736029+E_OFF; Ni[1]=-14638.736889+N_OFF; psiI[1]=6.946410;
        Ei[2]=29705.580777+E_OFF; Ni[2]=-14650.753379+N_OFF; psiI[2]=5.375614;
        Ei[3]=29706.318133+E_OFF; Ni[3]=-14586.726383+N_OFF; psiI[3]=8.517207;

        Kc[0]=0.052506;
        Kc[1]=-0.052535;
        Kc[2]=0.052506;
        Kc[3]=0.052535;

        Ef[0]=29657.855921+E_OFF; Nf[0]=-14644.893504+N_OFF; psiD[0]=3.141593;
        Ef[1]=29699.424162+E_OFF; Nf[1]=-14642.873272+N_OFF; psiD[1]=-1.570796;
        Ef[2]=29737.101207+E_OFF; Nf[2]=-14626.126920+N_OFF; psiD[2]=3.141593;
        Ef[3]=29672.630000+E_OFF; Nf[3]=-14582.590000+N_OFF; psiD[3]=1.570796;

        lengths[0][STRGHT]=50.000000; lengths[0][CLO_IN]=21.002466;
        lengths[0][CIR_ARC]=38.830368; lengths[0][CLO_OUT]=21.002466;
        lengths[1][STRGHT]=10.000000; lengths[1][CLO_IN]=9.551887;
        lengths[1][CIR_ARC]=20.347893; lengths[1][CLO_OUT]=9.551887;
        lengths[2][STRGHT]=10.000000; lengths[2][CLO_IN]=21.002466;
        lengths[2][CIR_ARC]=38.830368; lengths[2][CLO_OUT]=21.002466;
        lengths[3][STRGHT]=50.000000; lengths[3][CLO_IN]=9.551887;
        lengths[3][CIR_ARC]=20.347893; lengths[3][CLO_OUT]=9.551887;

        straights[0][E1]=29672.630000+E_OFF; straights[0][N1]=-14582.590000+N_OFF;
        straights[0][E2]=29633.229462+E_OFF; straights[0][N2]=-14613.373074+N_OFF;
        straights[1][E1]=29657.855921+E_OFF; straights[1][N1]=-14644.893504+N_OFF;
        straights[1][E2]=29665.736029+E_OFF; straights[1][N2]=-14638.736889+N_OFF;
        straights[2][E1]=29699.424162+E_OFF; straights[2][N1]=-14642.873272+N_OFF;
        straights[2][E2]=29705.580777+E_OFF; straights[2][N2]=-14650.753379+N_OFF;
        straights[3][E1]=29737.101207+E_OFF; straights[3][N1]=-14626.126920+N_OFF;
        straights[3][E2]=29706.318133+E_OFF; straights[3][N2]=-14586.726383+N_OFF;
        straights[4][E1]=29672.630000+E_OFF; straights[4][N1]=-14582.590000+N_OFF;
        straights[4][E2]=29672.630000+E_OFF; straights[4][N2]=-14582.590000+N_OFF;

        cumL[0]=0.000000;
        cumL[1]=50.000000;
        cumL[2]=71.002466;
        cumL[3]=109.832834;
        cumL[4]=130.835300;
        cumL[5]=140.835300;
        cumL[6]=150.387187;
        cumL[7]=170.735080;
        cumL[8]=180.286967;
        cumL[9]=190.286967;
        cumL[10]=211.289433;
        cumL[11]=250.119801;
        cumL[12]=271.122267;
        cumL[13]=321.122267;
        cumL[14]=330.674153;
        cumL[15]=351.022047;
        cumL[16]=360.573933;
        cumL[17]=360.573933;


    
        /********************************
         * END PUT MAP PARAMETERS HERE
         ********************************/
        
        // The initial guess is the distance along the curve from last time
        // Work vectors are initialized to zero, so the first guess will be 0
        guessDist = *lastDist;
        guessSeg = *lastSeg;
        
        // Calculate useful values from the map data
        courseLength = cumL[NUM_SEGS];
        if (E0 == En && N0 == Nn) mapType = CLOSED;
        else mapType = OPEN;
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
                    if (guessCurve < NUM_CURVES) totL = lengths[guessCurve][STRGHT];
                    else totL = LstrF;
                    fracL = delL/totL;
                    
                    // Find required parameters
                    east = fracL*(straights[guessCurve][E2]-straights[guessCurve][E1])+straights[guessCurve][E1];
                    eastPrime = (straights[guessCurve][E2]-straights[guessCurve][E1])/totL;
                    north = fracL*(straights[guessCurve][N2]-straights[guessCurve][N1])+straights[guessCurve][N1];
                    northPrime = (straights[guessCurve][N2]-straights[guessCurve][N1])/totL;
                    if (guessCurve < NUM_CURVES) psi = psiI[guessCurve];
                    else psi = psiI[NUM_CURVES-1]+psiD[NUM_CURVES-1];
                    crv = 0;
                    break;
                    
                // Clothoid from zero curvature segment
                case CLO_IN:
                    
                    // Find preliminary parameters
                    delL = guessDist - segStart;
                    a1 = sqrt(fabs(Kc[guessCurve])/(2*lengths[guessCurve][CLO_IN]));
                    
                    // Find required parameters
                    east = Ei[guessCurve]+(cos(psiI[guessCurve])/a1)*fresnelPS(COS_FPS, 0, a1*delL, numTerms)
                           -(sign(Kc[guessCurve])*sin(psiI[guessCurve])/a1)*fresnelPS(SIN_FPS, 0, a1*delL, numTerms);
                    north = Ni[guessCurve]+(sign(Kc[guessCurve])*cos(psiI[guessCurve])/a1)*fresnelPS(SIN_FPS, 0, a1*delL, numTerms)
                            +(sin(psiI[guessCurve])/a1)*fresnelPS(COS_FPS, 0, a1*delL, numTerms);
                    psi = (Kc[guessCurve]/(2*lengths[guessCurve][CLO_IN]))*pow(delL, 2.0) + psiI[guessCurve];
                    crv = (Kc[guessCurve]/lengths[guessCurve][CLO_IN])*delL;
                    eastPrime = cos(psi);
                    northPrime = sin(psi);
                    break;
                    
                // Circular arc segment
                case CIR_ARC:
                    
                    // Go through the entry clothoid calculation to find the start E and N coordinates
                    delL = lengths[guessCurve][CLO_IN];
                    a1 = sqrt(fabs(Kc[guessCurve])/(2*lengths[guessCurve][CLO_IN]));
                    eastStart = Ei[guessCurve]+(cos(psiI[guessCurve])/a1)*fresnelPS(COS_FPS, 0, a1*delL, numTerms)
                                -(sign(Kc[guessCurve])*sin(psiI[guessCurve])/a1)*fresnelPS(SIN_FPS, 0, a1*delL, numTerms);
                    northStart = Ni[guessCurve]+(sign(Kc[guessCurve])*cos(psiI[guessCurve])/a1)*fresnelPS(SIN_FPS, 0, a1*delL, numTerms)
                                 +(sin(psiI[guessCurve])/a1)*fresnelPS(COS_FPS, 0, a1*delL, numTerms);
                    psi2 = (Kc[guessCurve]/(2*lengths[guessCurve][CLO_IN]))*pow(delL, 2.0) + psiI[guessCurve];
                    
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
                    delL = lengths[guessCurve][CLO_IN];
                    a1 = sqrt(fabs(Kc[guessCurve])/(2*lengths[guessCurve][CLO_IN]));
                    eastStart = Ei[guessCurve]+(cos(psiI[guessCurve])/a1)*fresnelPS(COS_FPS, 0, a1*delL, numTerms)
                                -(sign(Kc[guessCurve])*sin(psiI[guessCurve])/a1)*fresnelPS(SIN_FPS, 0, a1*delL, numTerms);
                    northStart = Ni[guessCurve]+(sign(Kc[guessCurve])*cos(psiI[guessCurve])/a1)*fresnelPS(SIN_FPS, 0, a1*delL, numTerms)
                                 +(sin(psiI[guessCurve])/a1)*fresnelPS(COS_FPS, 0, a1*delL, numTerms);
                    psi2 = (Kc[guessCurve]/(2*lengths[guessCurve][CLO_IN]))*pow(delL, 2.0) + psiI[guessCurve];
                    delL = lengths[guessCurve][CIR_ARC];
                    eastStart += (sin(delL*Kc[guessCurve]+psi2)-sin(psi2))/Kc[guessCurve];
                    northStart -= (cos(delL*Kc[guessCurve]+psi2)-cos(psi2))/Kc[guessCurve];
                    
                    // Find preliminary parameters
                    delL = guessDist - segStart;
                    nDelL = lengths[guessCurve][CLO_OUT] - delL;
                    a2 = sqrt(fabs(Kc[guessCurve])/(2*lengths[guessCurve][CLO_OUT]));
                    
                    // Find required parameters
                    east = eastStart + (cos(psiF)/a2)*fresnelPS(COS_FPS, a2*nDelL, a2*lengths[guessCurve][CLO_OUT], numTerms)
                           +(sign(Kc[guessCurve])*sin(psiF)/a2)*fresnelPS(SIN_FPS, a2*nDelL, a2*lengths[guessCurve][CLO_OUT], numTerms);
                    north = northStart - (sign(Kc[guessCurve])*cos(psiF)/a2)*fresnelPS(SIN_FPS, a2*nDelL, a2*lengths[guessCurve][CLO_OUT], numTerms)
                            +(sin(psiF)/a2)*fresnelPS(COS_FPS, a2*nDelL, a2*lengths[guessCurve][CLO_OUT], numTerms);
                    psi = psiF - (Kc[guessCurve]/(2*lengths[guessCurve][CLO_OUT]))*pow(nDelL, 2.0);
                    crv = (Kc[guessCurve]/lengths[guessCurve][CLO_OUT])*nDelL;
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
                    east = straights[NUM_CURVES][E2];
                    eastPrime = (straights[NUM_CURVES][E2]-straights[NUM_CURVES][E1])/totL;
                    north = straights[NUM_CURVES][N2];
                    northPrime = (straights[NUM_CURVES][N2]-straights[NUM_CURVES][N1])/totL;
                    psi = psiI[NUM_CURVES-1]+psiD[NUM_CURVES-1];
                    crv = 0;
                    break;
                }
                
                if (guessDist < 0)
                {
                    guessDist = 0;
                    guessSeg = 0;
                    east = straights[0][E1];
                    eastPrime = (straights[0][E2]-straights[0][E1])/totL;
                    north = straights[0][N1];
                    northPrime = (straights[0][N2]-straights[0][N1])/totL;
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
        if (mapType == OPEN) dst2end = cumL[NUM_SEGS] - guessDist;
        else dst2end = -1;
        
        // Find outputs that pertain to current segment
        currCrvNum = guessCurve;
        currSegType = segType;
        if (currCrvNum < NUM_CURVES) currSegLngth = lengths[currCrvNum][currSegType];
        else currSegLngth = LstrF;
        currSegProg = newDelL;
        switch (currSegType) {
            case STRGHT:
                currCrvI = 0;
                currCrvF = 0;
                break;
            case CLO_IN:
                currCrvI = 0;
                currCrvF = Kc[currCrvNum];
                break;
            case CIR_ARC:
                currCrvI = Kc[currCrvNum];
                currCrvF = Kc[currCrvNum];
                break;
            case CLO_OUT:
                currCrvI = Kc[currCrvNum];
                currCrvF = 0;
        }
        
        // Find outputs that pertain to next segment
        
        // Make sure we aren't looking for a next segment that doesn't exist
        if (mapType == OPEN && guessSeg >= NUM_SEGS-1)
        {
            nxtDst = -1.0;
            nxtCrvNum = -1;
            nxtSegType = -1;
            nxtSegLngth = -1.0;
        }
        else
        {
            // We could still have wrap-around on a closed map
            if (guessSeg < NUM_SEGS-1) nxtDst = cumL[guessSeg+1] - guessDist;
            else nxtDst = cumL[0] - (guessDist - courseLength);
            
            nxtSegNum = guessSeg + 1;
            while (nxtSegNum >= NUM_SEGS) nxtSegNum -= NUM_SEGS;
            while (nxtSegNum < 0) nxtSegNum += NUM_SEGS;
            nxtCrvNum = nxtSegNum/4;
            nxtSegType = (int)fmod((double)nxtSegNum, 4.0);
            if (nxtCrvNum < NUM_CURVES) nxtSegLngth = lengths[nxtCrvNum][nxtSegType];
            else nxtSegLngth = LstrF;
        }
        
        // No matter what, we can report a "next segment" curvature set
        switch (nxtSegType)
        {
            case STRGHT:
                nxtCrvI = 0;
                nxtCrvF = 0;
                break;
            case CLO_IN:
                nxtCrvI = 0;
                nxtCrvF = Kc[nxtCrvNum];
                break;
            case CIR_ARC:
                nxtCrvI = Kc[nxtCrvNum];
                nxtCrvF = Kc[nxtCrvNum];
                break;
            case CLO_OUT:
                nxtCrvI = Kc[nxtCrvNum];
                nxtCrvF = 0;
                break;
            case -1:
                nxtCrvI = -1.0;
                nxtCrvF = -1.0;
        }
        
        // Find outputs that pertain to two segments ahead
        
        // Make sure we aren't looking for a two-ahead segment that doesn't exist
        if (mapType == OPEN && guessSeg >= NUM_SEGS-2)
        {
            twoCrvNum = -1;
            twoSegType = -1;
            twoSegLngth = -1.0;
        }
        else
        {
            twoSegNum = guessSeg + 2;
            while (twoSegNum >= NUM_SEGS) twoSegNum -= NUM_SEGS;
            while (twoSegNum < 0) twoSegNum += NUM_SEGS;
            twoCrvNum = twoSegNum/4;
            twoSegType = (int)fmod((double)twoSegNum, 4.0);
            if (twoCrvNum < NUM_CURVES) twoSegLngth = lengths[twoCrvNum][twoSegType];
            else twoSegLngth = LstrF;
        }
        
        // No matter what, we can report a "two-ahead segment" curvature set
        switch (twoSegType)
        {
            case STRGHT:
                twoCrvI = 0;
                twoCrvF = 0;
                break;
            case CLO_IN:
                twoCrvI = 0;
                twoCrvF = Kc[twoCrvNum];
                break;
            case CIR_ARC:
                twoCrvI = Kc[twoCrvNum];
                twoCrvF = Kc[twoCrvNum];
                break;
            case CLO_OUT:
                twoCrvI = Kc[twoCrvNum];
                twoCrvF = 0;
                break;
            case -1:
                twoCrvI = -1.0;
                twoCrvF = -1.0;
        }
        
        // Find outputs that pertain to three segments ahead
        
        // Make sure we aren't looking for a three-ahead segment that doesn't exist
        if (mapType == OPEN && guessSeg >= NUM_SEGS-3)
        {
            thrCrvNum = -1;
            thrSegType = -1;
            thrSegLngth = -1.0;
        }
        else
        {
            thrSegNum = guessSeg + 3;
            while (thrSegNum >= NUM_SEGS) thrSegNum -= NUM_SEGS;
            while (thrSegNum < 0) thrSegNum += NUM_SEGS;
            thrCrvNum = thrSegNum/4;
            thrSegType = (int)fmod((double)thrSegNum, 4.0);
            if (thrCrvNum < NUM_CURVES) thrSegLngth = lengths[thrCrvNum][thrSegType];
            else thrSegLngth = LstrF;
        }
        
        // No matter what, we can report a "three-ahead segment" curvature set
        switch (thrSegType)
        {
            case STRGHT:
                thrCrvI = 0;
                thrCrvF = 0;
                break;
            case CLO_IN:
                thrCrvI = 0;
                thrCrvF = Kc[thrCrvNum];
                break;
            case CIR_ARC:
                thrCrvI = Kc[thrCrvNum];
                thrCrvF = Kc[thrCrvNum];
                break;
            case CLO_OUT:
                thrCrvI = Kc[thrCrvNum];
                thrCrvF = 0;
                break;
            case -1:
                thrCrvI = -1.0;
                thrCrvF = -1.0;
        }
        
        // Last, set the wasMax flag
        wasMax = (numIter == iterLim);
        
        // Update static variables and set outputs
        *lastDist = guessDist;
        *lastSeg = guessSeg;
        
        matchOut[0] = e;
        matchOut[1] = dPsi;
        matchOut[2] = crv;
        matchOut[3] = finalSeg;
        matchOut[4] = dst2end;
        
        currOut[0] = currCrvNum;
        currOut[1] = currSegType;
        currOut[2] = currSegLngth;
        currOut[3] = currSegProg;
        currOut[4] = nxtDst;
        currOut[5] = currCrvI;
        currOut[6] = currCrvF;
        
        nextOut[0] = nxtCrvNum;
        nextOut[1] = nxtSegType;
        nextOut[2] = nxtSegLngth;
        nextOut[3] = nxtCrvI;
        nextOut[4] = nxtCrvF;
        
        twoOut[0] = twoCrvNum;
        twoOut[1] = twoSegType;
        twoOut[2] = twoSegLngth;
        twoOut[3] = twoCrvI;
        twoOut[4] = twoCrvF;
        
        debugOut[0] = numIter;
        debugOut[1] = wasMax;
        debugOut[2] = resErr;
        debugOut[3] = enable;
        debugOut[4] = guessDist;
        
        thrOut[0] = thrCrvNum;
        thrOut[1] = thrSegType;
        thrOut[2] = thrSegLngth;
        thrOut[3] = thrCrvI;
        thrOut[4] = thrCrvF;
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
        
        currOut[0] = 0;
        currOut[1] = 0;
        currOut[2] = 0;
        currOut[3] = 0;
        currOut[4] = 0;
        currOut[5] = 0;
        currOut[6] = 0;
        
        nextOut[0] = 0;
        nextOut[1] = 0;
        nextOut[2] = 0;
        nextOut[3] = 0;
        nextOut[4] = 0;
        
        twoOut[0] = 0;
        twoOut[1] = 0;
        twoOut[2] = 0;
        twoOut[3] = 0;
        twoOut[4] = 0;
        
        debugOut[0] = 0;
        debugOut[1] = 0;
        debugOut[2] = 0;
        debugOut[3] = 0;
        debugOut[4] = 0;
        
        thrOut[0] = 0;
        thrOut[1] = 0;
        thrOut[2] = 0;
        thrOut[3] = 0;
        thrOut[4] = 0;
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
