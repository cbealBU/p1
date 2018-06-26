/* EDIT, RYH 5/12/07: Set sample time to Ts, not inherited for compatibility
 *with simulation 
 *
 *Also modified simultaneous update handling so that it does not happen if 
 *GPS is not initialized*/

#define S_FUNCTION_NAME  ssest_filter
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>
#include "smatrix.h"

#define NUMOFINPUTS 	18
#define NUMOFOUTPUTS	44

#define NUMBER_OF_ARGS      (31)
#define INIT_STATE_ARGH    ssGetSFcnParam(S,0)     
#define INIT_COV_ARGH      ssGetSFcnParam(S,1) 
#define INIT_STATE_ARGR    ssGetSFcnParam(S,2)
#define INIT_COV_ARGR      ssGetSFcnParam(S,3)
#define INIT_STATE_ARGV    ssGetSFcnParam(S,4)
#define INIT_COV_ARGV      ssGetSFcnParam(S,5)

// sens_varH	ssGetSFcnParam(S,6) (gpsins.hdgfilter.sens_var) /* gyro measurements variance */
// timeconst1H	ssGetSFcnParam(S,7) (gpsins.hdgfilter.timeconst1)  /* filter time constant for sensitivity */
// timeconst2H	ssGetSFcnParam(S,8) (gpsins.hdgfilter.timeconst2) /* filter time constant for bias / sensitivity */
// msr_varH		ssGetSFcnParam(S,9) (gpsins.hdgfilter.msr_var) /* GPS measurements variance */
// init_sf_varH ssGetSFcnParam(S,10) (gpsins.hdgfilter.init_sf_var) /* initial sensitivity (scale factor) variance */
// diff_yaw	    ssGetSFcnParam(S,11) (gpsins.diff_yaw) /* threshold for yaw difference (GPS and integrated) */
// sens_varR	ssGetSFcnParam(S,12) (gpsins.rollfilter.sens_var) /* gyro measurements variance */
// timeconstR	ssGetSFcnParam(S,13) (gpsins.rollfilter.timeconst1)  /* filter time constant for bias */
// msr_varR		ssGetSFcnParam(S,14) (gpsins.rollfilter.msr_var) /* GPS measurements variance */
// diff_roll    ssGetSFcnParam(S,15) (gpsins.diff_roll) /* threshold for roll difference (GPS and integrated) */
// split_delay  ssGetSFcnParam(S,16) (gpsins.splitbeelinedelay) /*The minimum delay for which split integration takes place */
//dist_sensor   ssGetSFcnParam(S,17) (gpsins.dist_sensor)		/* distance from the CG to the sensors */
//height_sensor	ssGetSFcnParam(S,18) (gpsins.height_sensor)	/* height from the roll center to the sensors */
//h_gps_ant		ssGetSFcnParam(S,19) (gpsins.h_gps_ant) 	/* height from sensors to GPS antenna */
//l_gps_ant		ssGetSFcnParam(S,20) (gpsins.l_gps_ant)	/* distance from sensors to GPS antenna */
//sens1_varV	ssGetSFcnParam(S,21) (gpsins.velfilter.sens1_var) /* ax measurements variance */
//sens2_varV	ssGetSFcnParam(S,22) (gpsins.velfilter.sens2_var) /* ay measurements variance */
//timeconst1V	ssGetSFcnParam(S,23) (gpsins.velfilter.timeconst1) /* filter time constant for ax bias */
//timeconst2V	ssGetSFcnParam(S,24) (gpsins.velfilter.timeconst2) /* filter time constant for ay bias */
//msr1_varV	    ssGetSFcnParam(S,25) (gpsins.velfilter.msr1_var)		/* GPS measurements variance */
//msr2_varV	    ssGetSFcnParam(S,26) (gpsins.velfilter.msr2_var)		/* GPS measurements variance */
//diff_vel_x	ssGetSFcnParam(S,27) (gpsins.diff_vel_x)	/* threshold for Vx difference (GPS and integrated) */
//diff_vel_y	ssGetSFcnParam(S,28) (gpsins.diff_vel_y)	/* threshold for Vy difference (GPS and integrated) */
//num_grade_avg ssGetSFcnParam(S,29) (gpsins.num_grade_avg)	/* how long average grade when slow */
//Ts            ssGetSFcnParam(S,30) (Ts from InitModel.m) /*sampling time*/

#define ORDH     3	/* ORD is the order of the plant */
#define ORDUH    0	/* ORDU is the number of inputs into the plant */
#define ORDYH    1 	/* ORDY is the number of outputs out of the plant */
#define ORDNH    3 	/* ORDN is the number of process noise inputs */
#define ORDVH    1 	/* ORDN is the number of time-variants in the plant */

#define ORDR     2	/* ORD is the order of the plant */
#define ORDUR    1	/* ORDU is the number of inputs into the plant */
#define ORDYR    1 	/* ORDY is the number of outputs out of the plant */
#define ORDNR    2 	/* ORDN is the number of process noise inputs */
#define ORDVR    0 	/* ORDN is the number of time-variants in the plant */

#define ORDV     4	/* ORD is the order of the plant */
#define ORDUV    4	/* ORDU is the number of inputs into the plant */
#define ORDYV    2 	/* ORDY is the number of outputs out of the plant */
#define ORDNV    4 	/* ORDN is the number of process noise inputs */
#define ORDVV    3 	/* ORDN is the number of time-variants in the plant */

#define NUMOFSTATESH (ORDH + ORDH*ORDH)
#define NUMOFSTATESR (ORDR + ORDR*ORDR)
#define NUMOFSTATESV (ORDV + ORDV*ORDV)
// #define NUMOFSTATES  (NUMOFSTATESH+NUMOFSTATESV+NUMOFSTATESR)

#define MAX_LATENCY	((int_T)(0.05/Ts))	/* threshold for maximum GPS velocity latency */

#define BUFF_LENGTH	2500 /* past 5 seconds stored in bufferR */
#define BUFF_LENGTH_xH (BUFF_LENGTH*NUMOFSTATESH)
#define BUFF_LENGTH_xV (BUFF_LENGTH*NUMOFSTATESV)
#define BUFF_LENGTH_uV (BUFF_LENGTH*ORDUV)
#define BUFF_LENGTH_xR (BUFF_LENGTH*NUMOFSTATESR)
// #define split_num 2 /*The number of timesteps across which integration will be split for long beeline delays*/
#define MAX_SPLIT 20

#define TRU	1
#define FALS 0

#define PI 3.14159
#define D2R (PI/180) /*Degrees to radians*/
#define R2D (180/PI)	/* radian to degree */
#define G   9.81		/* gravity constant */


/*==================*
 * Global Variables *
 *==================*/
/*HEADING FILTER DECLARATIONS*/
/* Continuous System
 * static real_T Ac[ORD*ORD]   = { 0, r, -1,  0, 0, 0,  0, 0, 0 };
 * static real_T Bc[ORD/*ORDU/]  = { 0, 0, 0 }; 
 * static real_T Cc[ORDY*ORD]  = { 1, 0, 0 };
 * static real_T Dc[ORDY/*ORDU/] = { 0 };
 */
/* Discrete System */
static real_T Ah[ORDH*ORDH];		/* time varying and defined when necessary */
static real_T Bh[ORDH/**ORDU*/] 	= { 0, 0, 0 }; 
static real_T Ch[ORDYH*ORDH] 		= { 1, 0, 0 };
static real_T Dh[ORDYH/**ORDU*/] 	= { 0 };
/* Continuous System Process Noise
 * static real_T Bwc[ORD*ORDN] = { 1, 0, 0,  0, 1, 0,  0, 0, 1 };
 * static real_T Qc[ORDN*ORDN] = { SENSORVAR, 0, 0,  0, timeconst1, 0,
 *								   0, 0, timeconst2 };
 */
/* Discrete System Process Noise	(BwcQcBwc'*Ts) */
static real_T Qh[ORDH*ORDH];  
/* Measurement Noise */
static real_T Rh[ORDYH*ORDYH];

/*EDIT: Separate bufferH into discrete buffers for all variables of interest*/
static real_T bufferH_yawRate[BUFF_LENGTH]; /*yaw buffer*/
static real_T bufferH_x[BUFF_LENGTH_xH]; /*state buffer (including covariances)*/

static int_T initializedFlagH, diffYawCount;
static real_T initStatesH[NUMOFSTATESH];  /* Initial States */

/*ROLL FILTER DECLARATIONS*/
/* Continuous System
 * static real_T Ac[ORD*ORD]   = { 0, -1, 0, 0 };
 * static real_T Bc[ORD*ORDU]  = { 1, 0 }; 
 * static real_T Cc[ORDY*ORD]  = { 1, 0 };
 * static real_T Dc[ORDY*ORDU] = { 0 };
 */
/* Discrete System */
static real_T Ar[ORDR*ORDR];
static real_T Br[ORDR*ORDUR];
static real_T Cr[ORDYR*ORDR]  = { 1, 0 };
static real_T Dr[ORDYR*ORDUR] = { 0 };
/* Continuous System Process Noise
 * static real_T Bwc[ORD*ORDN] = { 1, 0, 0, 1 };
 * static real_T Qc[ORDN*ORDN] = { SENSORVAR, 0, 0, timeconst };
 */
/* Discrete System Process Noise	(BwcQcBwc'*Ts) */
static real_T Qr[ORDR*ORDR]; 
/* Measurement Noise */
static real_T Rr[ORDYR*ORDYR];

//RH static real_T bufferR[BUFFERSIZE]; /* bufferR for reintegration */
static real_T bufferR_rollRate[BUFF_LENGTH]; /*buffer for roll rate*/
static real_T bufferR_x[BUFF_LENGTH_xR]; /*larger state buffer to accommodate 5 seconds for ALL states*/

static int_T initializedFlagR, diffRollCount;
static real_T initStatesR[NUMOFSTATESR];  /* Initial States */

/*VELOCITY FILTER*/
/* Continuous System
 * static real_T Ac[ORD*ORD]   = { 0, -1, r, 0,	 //  0, 0, 0, 0,
 								   -r, 0, 0, -1, //  0, 0, 0, 0 };
 * static real_T Bc[ORD*ORDU]  = { 1, 0, 0, -G,  //  0, 0, 0, 0,
 								   0, 1, -G, 0,  //  0, 0, 0, 0 }; 
 * static real_T Cc[ORDY*ORD]  = { 1, 0, 0, 0,   //  0, 0, 1, 0 };
 * static real_T Dc[ORDY*ORDU] = { 0, 0, 0, 0,   //  0, 0, 0, 0 };
 */
/* Discrete System */
static real_T Av[ORDV*ORDV];		/* time varying and defined when necessary */
static real_T Bv[ORDV*ORDUV];		/* time varying and defined when necessary */
static real_T Cv[ORDYV*ORDV]  = { 1, 0, 0, 0,  0, 0, 1, 0 };
static real_T Dv[ORDYV*ORDUV] = { 0, 0, 0, 0,  0, 0, 0, 0 };
/* Continuous System Process Noise
 * static real_T Bwc[ORD*ORDN] = I;
 * static real_T Qc[ORDN*ORDN] = { sens1_var, 0, 0, 0,//0, timeconst1, 0, 0,
 *								   0, 0, sens2_var, 0,//0, 0, 0, timeconst2 };
 */
/* Discrete System Process Noise	(BwcQcBwc'*Ts) */
static real_T Qv[ORDV*ORDV]; 
/* Measurement Noise */
static real_T Rv[ORDYV*ORDYV]; 

//RH static real_T bufferV[BUFFERSIZE];	/* bufferV for reintegration */
static real_T bufferV_x[BUFF_LENGTH_xV]; /*buffer for storage of states*/
static real_T bufferV_u[BUFF_LENGTH_uV]; /*buffer for storage of inputs*/
// static real_T bufferV_yawRate[BUFF_LENGTH];
// static real_T bufferV_rollRate[BUFF_LENGTH];
// static real_T bufferV_yawAngle[BUFF_LENGTH];
// static real_T bufferV_rollAngle[BUFF_LENGTH];

static int_T initializedFlagV, diffVelCount;
static real_T gpsGradeV;				/* Grade from GPS */
static real_T initStatesV[NUMOFSTATESV];  /* Initial States */

static int_T buff_counter; /*Buffer counter*/

/*SPLIT INTEGRATION VARIABLES*/
static int_T SplitIntegrationH, SplitCounterH; /*Detection and tracking of split integration for heading filter*/
static int_T SplitIntegrationR, SplitCounterR; /*Detection and tracking of split integration for roll filter*/
static real_T bufferH_SplitData[BUFF_LENGTH_xH]; /*Temporary storage of heading filter states during split integration*/
static real_T bufferR_SplitData[BUFF_LENGTH_xR]; /*Temporary storage of roll filter states during split integration*/
static int_T split_num; /*Number of splits required to integrate beeline GPS delay according to rules above*/
static int_T SplitIntervals[MAX_SPLIT]; /*Intervals (in sample times) for each cycle of split integration*/
static int_T delayBeeline_global; /*Copy delay into global variable when splitting up integration*/
static int_T last_indexH, last_indexR; /*Last buffer index integrated during previous integration step*/

/*SIMULTANEOUS UPDATE VARIABLES*/
static int_T SimulUpdate; /*Flag activated if Beeline and OEM4 are updated simultaneously*/
static int_T SimulUpdate_prev; /*Value of SimulUpdate in previous time step*/
static int_T delayOEM4_global; /*Used to store OEM4 delay in the event of simultaneous updates*/

/*================*
 * help functions *
 *================*/
#include "smatrix.c"
static real_T AngleMod180(real_T ang);
static real_T AngleMod180(real_T ang)
{
	real_T result;

	result = fmod(ang + 180, 360);
	result += result < 0 ? 360 : 0;

	return result - 180;
}
/*====================*
 * S-function methods *
 *====================*/
static void mdlInitializeSizes(SimStruct *S)
{
    char_T msg[256];

  	ssSetNumSFcnParams(S, NUMBER_OF_ARGS);  
  	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
    	sprintf(msg,"Wrong number of input arguments passed.\n%d arguments are expected\n",NUMBER_OF_ARGS);
        ssSetErrorStatus(S,msg);
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, NUMOFSTATESH+NUMOFSTATESR+NUMOFSTATESV);

    if (!ssSetNumInputPorts(S, 1)) {
    	return;
    }
    ssSetInputPortWidth(S, 0, NUMOFINPUTS);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) {
    	return;
    }
    ssSetOutputPortWidth(S, 0, NUMOFOUTPUTS);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, 0);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    const real_T Ts = mxGetPr(ssGetSFcnParam(S,30))[0];
    
//     ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetSampleTime(S, 0, Ts);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *x0 = ssGetRealDiscStates(S);
	int_T i;
    real_T x0H[NUMOFSTATESH];
    real_T x0R[NUMOFSTATESR];
    real_T x0V[NUMOFSTATESV];
    
    const real_T init_sf_varH = mxGetPr(ssGetSFcnParam(S,10))[0];
    const real_T msr_varH = mxGetPr(ssGetSFcnParam(S,9))[0];
    const real_T msr_varR = mxGetPr(ssGetSFcnParam(S,14))[0];
    const real_T msr1_varV = mxGetPr(ssGetSFcnParam(S,25))[0];
    const real_T msr2_varV = mxGetPr(ssGetSFcnParam(S,26))[0];
    
	/* initializing states */
    /*HEADING FILTER*/
    
    for (i = 0; i < NUMOFSTATESH; i++) {
        x0H[i] = 0;
    }
    Rh[0] = msr_varH;   
	/* setting the initial heading angle as 180 which is not a normal value */
	x0H[0] = 180;
    x0H[1] = (real_T)mxGetPr(INIT_STATE_ARGH)[0]; /* sensitivity */
    if (x0H[1] == 0) {
        x0H[1] = 1;
    }
    x0H[2] = (real_T)mxGetPr(INIT_STATE_ARGH)[1]; /* bias */

    /* setting the diagonal elements of the Covarianve matrix */
    for (i = 0; i < ORDH; i++) {
        x0H[ORDH + i*ORDH + i] = (real_T)mxGetPr(INIT_COV_ARGH)[i];
    }
    if (x0H[ORDH + 0*ORDH + 0] == 0) {
        x0H[ORDH + 0*ORDH + 0] = Rh[0*ORDYH + 0];
    }

    if (x0H[ORDH + 1*ORDH + 1] == 0) {
        x0H[ORDH + 1*ORDH + 1] = init_sf_varH;
    }
    if (x0H[ORDH + 2*ORDH + 2] == 0) {
        x0H[ORDH + 2*ORDH + 2] = 1;
    }
    
    for (i = 0; i < NUMOFSTATESH; i++) {
        initStatesH[i] = x0H[i];
    }

    /* to initialize yaw to gps value at first valid gps */
    initializedFlagH = FALS;
    /* initialize diffYawCount */
    diffYawCount = 0;
    
    /*ROLL FILTER*/
    for (i = 0; i < NUMOFSTATESR; i++) {
        x0R[i] = 0;
    }
        
    Rr[0] = msr_varR;
	/* setting the initial heading angle as 180 which is not a normal value */
	x0R[0] = 180;
    x0R[1] = (real_T)mxGetPr(INIT_STATE_ARGR)[0]; /* bias */

    /* setting the diagonal elements of the Covarianve matrix */
    for (i = 0; i < ORDR; i++) {
        x0R[ORDR + i*ORDR + i] = (real_T)mxGetPr(INIT_COV_ARGR)[i];
    }
    if (x0R[ORDR + 0*ORDR + 0] == 0) {
        x0R[ORDR + 0*ORDR + 0] = Rr[0*ORDYR + 0];
    }
    if (x0R[ORDR + 1*ORDR + 1] == 0) {
        x0R[ORDR + 1*ORDR + 1] = 1;
    }

    for (i = 0; i < NUMOFSTATESR; i++) {
        initStatesR[i] = x0R[i];
    }

	/* to initialize roll to gps value at first valid gps */
    initializedFlagR = FALS;
    /* initialize diffRollCount */
    diffRollCount = 0;
    
    /*VELOCITY FILTER*/
    for (i = 0; i < NUMOFSTATESV; i++) {
        x0V[i] = 0;
    }
    Rv[0] = msr1_varV;
    Rv[1] = 0;
    Rv[2] = 0;
    Rv[3] = msr2_varV;

	/* initializing states */
    x0V[1] = (real_T)mxGetPr(INIT_STATE_ARGV)[0]; /* ax bias */
    x0V[3] = (real_T)mxGetPr(INIT_STATE_ARGV)[1]; /* ay bias */

    /* setting the diagonal elements of the Covariance matrix */
    for (i = 0; i < ORDV; i++) {
        x0V[ORDV + i*ORDV + i] = (real_T)mxGetPr(INIT_COV_ARGV)[i];
    }
    if (x0V[ORDV + 0*ORDV + 0] == 0) {
        x0V[ORDV + 0*ORDV + 0] = Rv[0*ORDYV + 0];
    }
    if (x0V[ORDV + 1*ORDV + 1] == 0) {
        x0V[ORDV + 1*ORDV + 1] = 1;
    }
    if (x0V[ORDV + 2*ORDV + 2] == 0) {
        x0V[ORDV + 2*ORDV + 2] = Rv[1*ORDYV + 1];
    }
    if (x0V[ORDV + 3*ORDV + 3] == 0) {
        x0V[ORDV + 3*ORDV + 3] = 1;
    }

    for (i = 0; i < NUMOFSTATESV; i++) {
        initStatesV[i] = x0V[i];
    }
    
	/* flag for the first valid GPS and heading from heading filter */
    initializedFlagV = FALS;
    gpsGradeV = 0;		/* initialize GPS grade */
	diffVelCount = 0;	/* initialize diffVelCountV */
    
    /*Store heading filter initial states*/
    for (i = 0; i<NUMOFSTATESH; i++) {
        x0[i] = x0H[i];
    }
    /*Store roll filter initial states*/
    for (i = 0; i<NUMOFSTATESR; i++) {
        x0[NUMOFSTATESH+i] = x0R[i];
    }
    /*Store velocity filter initial states*/
    for (i = 0; i<NUMOFSTATESV; i++) {
        x0[NUMOFSTATESH+NUMOFSTATESR+i] = x0V[i];
    }
    
    /*Initialize all buffers*/
    /*HEADING FILTER BUFFERS*/
    for (i = 0; i<BUFF_LENGTH; i++) {
        bufferH_yawRate[i] = 0;
    }
    for (i = 0; i<BUFF_LENGTH_xH; i++) {
        bufferH_x[i] = 0;
    }
    
    /*ROLL FILTER BUFFERS*/
    for (i = 0; i<BUFF_LENGTH; i++) {
        bufferR_rollRate[i] = 0;
    }
    for (i = 0; i<BUFF_LENGTH_xR; i++) {
        bufferR_x[i] = 0;
    }
    
    /*VELOCITY FILTER BUFFERS*/
    for (i = 0; i<BUFF_LENGTH_uV; i++) {
        bufferV_u[i] = 0;
    }
    for (i = 0; i<BUFF_LENGTH_xV; i++) {
        bufferV_x[i] = 0;
    }
//     for (i = 0; i<BUFF_LENGTH; i++) {
//         bufferV_yawRate[i] = 0;
//     }
//     for (i = 0; i<BUFF_LENGTH; i++) {
//         bufferV_rollRate[i] = 0;
//     }
//     for (i = 0; i<BUFF_LENGTH; i++) {
//         bufferV_yawAngle[i] = 0;
//     }
//     for (i = 0; i<BUFF_LENGTH; i++) {
//         bufferV_rollAngle[i] = 0;
//     }
        
    /*Initialize Split Integration Variables*/
    SplitIntegrationH = FALS;
    SplitCounterH = 0;
    SplitIntegrationR = FALS;
    SplitCounterR = 0;
    for (i = 0; i < MAX_SPLIT; i++) {
        SplitIntervals[i] = 0;
    }
    delayBeeline_global = 0;
    last_indexH = 0;
    last_indexR = 0;
    
    /*Initialize Split Integration Buffers*/
    for (i = 0; i<BUFF_LENGTH_xH; i++) {
        bufferH_SplitData[i] = 0;
    }
    for (i = 0; i<BUFF_LENGTH_xR; i++) {
        bufferR_SplitData[i] = 0;
    } 
    
    buff_counter = 0;
    
    SimulUpdate = FALS;
    SimulUpdate_prev = FALS;
    delayOEM4_global = 0;
}

/*********************************************************************
 *
 *  START OF MDLOUTPUTS
 *
 *********************************************************************/
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T *x = ssGetRealDiscStates(S);
    real_T t = ssGetT(S);
    int_T tr;
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    const real_T Ts = mxGetPr(ssGetSFcnParam(S,30))[0];
    const int_T split_delay = mxGetPr(ssGetSFcnParam(S,16))[0];
    int_T gpsUpdateBeeline, delayBeeline;
    int_T gpsUpdateOEM4, delayOEM4;
    int_T i, j;
    int_T rem;
    real_T oneSatBeeline;
    int_T gpsFlagBeeline, oneSatFlagBeeline, angleFlagBeeline;
    int_T gpsFlagOEM4;
    
    /*HEADING FILTER DECLARATIONS*/
    real_T yawRate, yawGPS, yawAngle;
    int_T buff_indexH, buff_indexH_copy;
    
	real_T AtH[ORDH*ORDH], CtH[ORDH*ORDYH], uuH[1/*ORDUH*/];
	real_T xhatH[NUMOFSTATESH], xplusH[NUMOFSTATESH], y_mH[ORDYH];
	real_T KH[ORDH*ORDYH], tempK1H[ORDH*ORDYH], tempK2H[ORDYH*ORDH];
	real_T tempK3H[ORDYH*ORDYH], tempK4H[ORDYH*ORDYH];
	real_T tempEyeH[ORDH*ORDH], tempP1H[ORDH*ORDH], tempP2H[ORDH*ORDH];
	real_T tempX1H[ORDH], tempX2H[ORDH], tempY1H[ORDYH], tempY2H[ORDYH];
    
    const real_T sens_varH = mxGetPr(ssGetSFcnParam(S, 6))[0];
    const real_T timeconst1H = mxGetPr(ssGetSFcnParam(S,7))[0];
    const real_T timeconst2H = mxGetPr(ssGetSFcnParam(S,8))[0];
    const real_T msr_varH = mxGetPr(ssGetSFcnParam(S,9))[0];    
    const real_T diff_yaw = mxGetPr(ssGetSFcnParam(S,11))[0];
    
    /*ROLL FILTER DECLARATIONS*/
    real_T rollRate, rollGPS, rollAngle, diff;
    int_T buff_indexR, buff_indexR_copy;

	real_T AtR[ORDR*ORDR], CtR[ORDR*ORDYR], uuR[ORDUR];
	real_T xhatR[NUMOFSTATESR], xplusR[NUMOFSTATESR], y_mR[ORDYR];
	real_T KR[ORDR*ORDYR], tempK1R[ORDR*ORDYR], tempK2R[ORDYR*ORDR];
	real_T tempK3R[ORDYR*ORDYR], tempK4R[ORDYR*ORDYR];
	real_T tempEyeR[ORDR*ORDR], tempP1R[ORDR*ORDR], tempP2R[ORDR*ORDR];
	real_T tempX1R[ORDR], tempX2R[ORDR], tempY1R[ORDYR], tempY2R[ORDYR];
    
    const real_T sens_varR = mxGetPr(ssGetSFcnParam(S,12))[0];
    const real_T timeconstR = mxGetPr(ssGetSFcnParam(S,13))[0];
    const real_T msr_varR = mxGetPr(ssGetSFcnParam(S,14))[0];
    const real_T diff_roll = mxGetPr(ssGetSFcnParam(S,15))[0];
        
    /*VELOCITY FILTER DECLARATIONS*/
    real_T ax, ay, gpsSpeed, gpsVerSpeed, gpsVdir, yawRateCorr, rollRateCorr;
	real_T slipAngleGPS, VxGPS, VyGPS, VyCg, slipAngleCg, axCg, ayCg;
	int_T buff_indexV, buff_indexV_copy;
    int_T latency;
   
	real_T AtV[ORDV*ORDV], CtV[ORDV*ORDYV], uuV[ORDUV];
	real_T xhatV[NUMOFSTATESV], xplusV[NUMOFSTATESV], y_mV[ORDYV];
	real_T KV[ORDV*ORDYV], tempK1V[ORDV*ORDYV], tempK2V[ORDYV*ORDV];
	real_T tempK3V[ORDYV*ORDYV], tempK4V[ORDYV*ORDYV];
	real_T tempEyeV[ORDV*ORDV], tempP1V[ORDV*ORDV], tempP2V[ORDV*ORDV];
	real_T tempX1V[ORDV], tempX2V[ORDV], tempY1V[ORDYV], tempY2V[ORDYV];
   
    const real_T dist_sensor = mxGetPr(ssGetSFcnParam(S,17))[0];
    const real_T height_sensor = mxGetPr(ssGetSFcnParam(S,18))[0];
    const real_T h_gps_ant = mxGetPr(ssGetSFcnParam(S,19))[0];
    const real_T l_gps_ant = mxGetPr(ssGetSFcnParam(S,20))[0];
    const real_T sens1_varV = mxGetPr(ssGetSFcnParam(S,21))[0];
    const real_T sens2_varV = mxGetPr(ssGetSFcnParam(S,22))[0];
    const real_T timeconst1V = mxGetPr(ssGetSFcnParam(S,23))[0];
    const real_T timeconst2V = mxGetPr(ssGetSFcnParam(S,24))[0];
    const real_T msr1_varV = mxGetPr(ssGetSFcnParam(S,25))[0];
    const real_T msr2_varV = mxGetPr(ssGetSFcnParam(S,26))[0];
    const real_T diff_vel_x = mxGetPr(ssGetSFcnParam(S,27))[0];
    const real_T diff_vel_y = mxGetPr(ssGetSFcnParam(S,28))[0];
    const int_T num_grade_avg = mxGetPr(ssGetSFcnParam(S,29))[0];
    const int_T MaxLatency = 0.05/Ts;
    
    gpsUpdateBeeline = (int_T) *uPtrs[0]; /* Time Pulse from UTC time */
 	delayBeeline = (int_T) *uPtrs[1]; /* Delay of Beeline GPS in number of samples */
    gpsUpdateOEM4 = (int_T) *uPtrs[2]; /*Time Pulse from UTC time - redundant with one receiver!*/
    delayOEM4 = (int_T) *uPtrs[3]; /*Delay of OEM4 GPS in number of samples*/
    oneSatBeeline = *uPtrs[4]; /* Avg_One_Sat_Yaw (for beeline set to 0)*/
	gpsFlagBeeline = (int_T) *uPtrs[5]; /* UseGPSFlag (1 when gps is valid) */
	oneSatFlagBeeline = (int_T) *uPtrs[6]; /* Avg_One_Sat_Flag (0 for beeline) */
    gpsFlagOEM4 = (int_T) *uPtrs[7];
        
    /*YAW FILTER INPUTS*/
	yawRate = *uPtrs[8]; /* Yaw Rate from gyro (deg/s) */
	yawGPS = *uPtrs[9]; /* Yaw Angle from GPS (deg) */
        
    /*ROLL FILTER INPUTS*/
    rollRate = *uPtrs[10]; /* Roll Rate from gyro (deg/s) */
	rollGPS = *uPtrs[11]; /* Roll Angle from GPS (deg) */
    
    /*VELOCITY FILTER INPUTS*/
	ax = *uPtrs[12];				/* longitudinal acceleration, ax (m/s^2) */
	ay = *uPtrs[13];				/* lateral acceleration, ay (m/s^2) */
	gpsSpeed = *uPtrs[14];		/* GPS horizontal speed (m/s) */
	gpsVdir = AngleMod180(*uPtrs[15]) * D2R;	/* GPS velocity direction (rad) */
//     printf("%f\r\n", gpsVdir);
	gpsVerSpeed = *uPtrs[16];		/* GPS vertical speed (m/s) */
    latency = (int_T) (*uPtrs[17] / Ts); 	/* GPS velocity latency (converted to number of samples) */
    latency = 0;
    delayOEM4 = delayOEM4 + latency;
    
    tr = floor(t*1000+0.5);
	
    /*Initialize constant-valued covariance and system matrices*/
    Qh[0] = sens_varH*Ts;
    Qh[1] = 0;
    Qh[2] = 0;
    Qh[3] = 0;
    Qh[4] = timeconst1H*Ts;
    Qh[5] = 0;
    Qh[6] = 0;
    Qh[7] = 0;
    Qh[8] = timeconst2H*Ts;
    
    Rh[0] = msr_varH;
    
    Qr[0] = sens_varR*Ts;
    Qr[1] = 0;
    Qr[2] = 0;
    Qr[3] = timeconstR*Ts;

    Rr[0] = msr_varR;
        
    Ar[0] = 1;
    Ar[1] = -Ts;
    Ar[2] = 0;
    Ar[3] = 1;
    
    Br[0] = Ts;
    Br[1] = 0;
    
    Rv[0] = msr1_varV;
    Rv[1] = 0;
    Rv[2] = 0;
    Rv[3] = msr2_varV;
    
    Qv[0] = sens1_varV*Ts;
    Qv[1] = 0;
    Qv[2] = 0;
    Qv[3] = 0;
    Qv[4] = 0;
    Qv[5] = timeconst1V*Ts;
    Qv[6] = 0;
    Qv[7] = 0;
    Qv[8] = 0;
    Qv[9] = 0;
    Qv[10] = sens2_varV*Ts;
    Qv[11] = 0;
    Qv[12] = 0;
    Qv[13] = 0;
    Qv[14] = 0;
    Qv[15] = timeconst2V*Ts;

	if (gpsFlagBeeline) {
		angleFlagBeeline = TRU;
		yawAngle = yawGPS;
        rollAngle = rollGPS;
	}
	else if (oneSatFlagBeeline) {
		angleFlagBeeline = TRU;
		yawAngle = oneSatBeeline;
        rollAngle = oneSatBeeline;
	}
	else {
		angleFlagBeeline = FALS;
		yawAngle = 0;
        rollAngle = 0;
	}
    
    SimulUpdate_prev = SimulUpdate;
    
    if ((angleFlagBeeline && gpsUpdateBeeline && gpsFlagOEM4 && gpsUpdateOEM4) || (gpsFlagOEM4 && gpsUpdateOEM4 && SplitIntegrationH)) {
        SimulUpdate = TRU;
    }
    else {
        SimulUpdate = FALS;
    }
    
    /*MDLOUTPUTS FOR HEADING FILTER*/
    /*store yaw rate*/
    bufferH_yawRate[buff_counter] = yawRate;
	/* store states and covariances */
	for (i = 0; i < NUMOFSTATESH; i++) {
        bufferH_x[buff_counter*NUMOFSTATESH+i] = x[i];
	}
	/* start reintegrating after initialized and relevant data exists */
	if ((!initializedFlagH && buff_counter < delayBeeline) || delayBeeline < 0) {
		gpsUpdateBeeline = FALS;
	}
    
	if (angleFlagBeeline && gpsUpdateBeeline) { /* when new GPS measurement available */
		if (delayBeeline >= BUFF_LENGTH) {
			printf("Buffer is not of sufficient size... [hdgfilter]\n");
		}
        /*If split integration was supposed to continue this time step, cancel, reset variables*/
        if (SplitIntegrationH) {
            SplitCounterH = 0;
            SplitIntegrationH = FALS;
        }
		buff_indexH = buff_counter - delayBeeline; /* account for delay */
		buff_indexH += buff_indexH < 0 ? BUFF_LENGTH : 0; /* [0, BUFF_LENGTH) */
		for (i = 0; i < NUMOFSTATESH; i++) {	/* retrieve states */
//RH 			xhat[i] = bufferH[buff_index*BUFF_NOSIGS + ORDU + ORDV + i];
            xhatH[i] = bufferH_x[buff_indexH*NUMOFSTATESH+i];
		}
		/* measurements */
		yawAngle = AngleMod180(yawAngle);  /* to the range of [-180,180) */
		/* at first valid GPS, initialize yaw to GPS value */
		if (!initializedFlagH) {
		    for (i = 0; i < NUMOFSTATESH; i++) {
		        xhatH[i] = initStatesH[i];
		    }
			xhatH[0] = yawAngle;
			initializedFlagH = TRU;
		}
		/* checking that gps and ins yaw are close. if not don't trust GPS */
		if (fabs(AngleMod180(xhatH[0] - yawAngle)) < diff_yaw) {
			diffYawCount = 0;
			/* Kalman Gain K = P(-)Ch'*inv[CP(-)Ch'+Rh] */
			sMatTrsp(Ch, CtH, ORDYH, ORDH);	/* transpose Ch */
			sMatMult(xhatH+ORDH, CtH, tempK1H, ORDH, ORDH, ORDYH);	/* P(-)*Ch' */
			sMatMult(Ch, xhatH+ORDH, tempK2H, ORDYH, ORDH, ORDH); 	/* CP(-) */
			sMatMult(tempK2H, CtH, tempK3H, ORDYH, ORDH, ORDYH);	/* CP(-)Ch' */
			sMatAdd(tempK3H, Rh, tempK4H, ORDYH, ORDYH);			/* CP(-)Ch'+Rh */
			sMatInv(tempK4H, tempK3H, ORDYH);				/* inv(CP(-)Ch'+Rh) */
			sMatMult(tempK1H, tempK3H, KH, ORDH, ORDYH, ORDYH);	/* K */
			/* error covariance update P(+) = [I - KC]P(-)*/
			sMatEye(tempEyeH, ORDH);							/* I */
			sMatMult(KH, Ch, tempP1H, ORDH, ORDYH, ORDH);			/* KC */
			sMatSub(tempEyeH, tempP1H, tempP2H, ORDH, ORDH);		/* I - KC */
			sMatMult(tempP2H, xhatH+ORDH, xplusH+ORDH, ORDH, ORDH, ORDH);	/* P(+) */
			/* state estimate update x(+) = x(-) + K[y_m - Cx(-)] */
			y_mH[0] = yawAngle;
			sMatMult(Ch, xhatH, tempY1H, ORDYH, ORDH, 1);	/* Cx(-) */
			sMatSub(y_mH, tempY1H, tempY2H, ORDYH, 1); 		/* y_m - Cx(-) */
			tempY2H[0] = AngleMod180(tempY2H[0]);		/* should be in range */
			sMatMult(KH, tempY2H, tempX1H, ORDH, ORDYH, 1);	/* K[y_m - Cx(-)] */
			sMatAdd(xhatH, tempX1H, xplusH, ORDH, 1);		/* x(+) */
			/* update the bufferH */
			xplusH[0] = AngleMod180(xplusH[0]); /* angle to [-180,180) */
			for (i = 0; i < NUMOFSTATESH; i++) {
//RH 				bufferH[buff_index*BUFF_NOSIGS + ORDU + ORDV + i] = xplus[i];
                bufferH_x[buff_indexH*NUMOFSTATESH+i] = xplusH[i];
			}
            
			/* reintegrate until the present time if beeline GPS is below split threshold */
            if (delayBeeline < split_delay) {
                for (j = 0; j < delayBeeline; j++) {
                    /* retrieve inputs */
    //RH 				for (i = 0; i < ORDU; i++) {
    // 					uu[i] = bufferH[buff_index*BUFF_NOSIGS + i];
    // 				}
                    uuH[0] = 0;	/* since ORDU is zero, no input */
                    /* retrieve yawRate */
                    yawRate = bufferH_yawRate[buff_indexH];
                    /* define Ah */
                    for (i = 0; i < ORDH*ORDH; i++) {
                        Ah[i] = 0;	/* initialize Ah */
                    }
                    Ah[0*ORDH + 0] = Ah[1*ORDH + 1] = Ah[2*ORDH + 2] = 1;	/* I */
                    Ah[0*ORDH + 1] = yawRate * Ts;
                    Ah[0*ORDH + 2] = -Ts;
                    /* transpose Ah */
                    sMatTrsp(Ah, AtH, ORDH, ORDH);
                    /* x(k+1) = Ax(k) + Bu(k) */
                    sMatMult(Ah, xplusH, tempX1H, ORDH, ORDH, 1);		/* Ax */
                    for (i = 0; i < ORDH; i++) {
                        xplusH[i] = tempX1H[i];
                    }
    //RH 				sMatMult(Bh, uu, tempX2, ORD, 1/*ORDU*/, 1);		/* Bu */  These steps are unncessary, as B = [0 0 0]' and u = 0
    // 				sMatAdd(tempX1, tempX2, xplus, ORD, 1);	/* Ax(k) + Bu(k)*/
                    /* P(k+1) = AP(k)Ah' + Qh */
                    sMatMult(Ah, xplusH+ORDH, tempP1H, ORDH, ORDH, ORDH);	/* AP */
                    sMatMult(tempP1H, AtH, tempP2H, ORDH, ORDH, ORDH);	/* APA' */
                    sMatAdd(tempP2H, Qh, xplusH+ORDH, ORDH, ORDH);		/* APA' + Qh*/
                    /* increase the bufferH index */
                    buff_indexH = ++buff_indexH < BUFF_LENGTH ? buff_indexH : 0;
                    /* update the bufferH */
                    xplusH[0] = AngleMod180(xplusH[0]); /* angle to [-180,180) */
                    for (i = 0; i < NUMOFSTATESH; i++) {
                        bufferH_x[buff_indexH*NUMOFSTATESH + i] = xplusH[i];
                    }
                }   /* for (j = 0; j < delay; j++) */
                for (i = 0; i < NUMOFSTATESH; i++) {
                    x[i] = xplusH[i];
                }
            }
            /*If delay is ABOVE split threshold...*/
            else {
                SplitCounterH++; /*Augment the split counter to 1*/
                SplitIntegrationH = TRU; /*Activate the split integration flag*/
                /*Compute how many processor cycles will be used for integration*/
                rem = delayBeeline % split_delay;
                split_num = (delayBeeline-rem)/split_delay + 1;
                /*Update delay to reflect additional processing cycles for integration*/
                delayBeeline_global = delayBeeline + (split_num-1);
                /*Determine integration intervals for each processing cycle, according to rules above*/
                for (i = 0; i < split_num; i++) {
                    if (i == split_num-1) {
                        SplitIntervals[i] = rem + (split_num-1);
                    }
                    else {
                        SplitIntervals[i] = split_delay;
                    }
                }
                /*Old code, used when user specified split number*/
//                 rem = delay % split_num;
                
//                 for (i = 0; i < split_num; i++) {
//                     if (i == split_num-1) {
//                         SplitIntervals[i] = (delay-rem)/split_num + rem;
//                     }
//                     else {
//                         SplitIntervals[i] = (delay-rem)/split_num;
//                     }
//                 }                
                /*Re-integrate from buff_index across first split interval 
                 (to buff_index+SplitIntervals[SplitCounterH-1]*/
                for (j = 0; j < SplitIntervals[SplitCounterH-1]; j++) {
                    /* retrieve inputs */
    //RH 				for (i = 0; i < ORDU; i++) {
    // 					uu[i] = bufferH[buff_index*BUFF_NOSIGS + i];
    // 				}
                    uuH[0] = 0;	/* since ORDU is zero, no input */
                    /* retrieve yawRate */
                    yawRate = bufferH_yawRate[buff_indexH];
                    /* define Ah */
                    for (i = 0; i < ORDH*ORDH; i++) {
                        Ah[i] = 0;	/* initialize Ah */
                    }
                    Ah[0*ORDH + 0] = Ah[1*ORDH + 1] = Ah[2*ORDH + 2] = 1;	/* I */
                    Ah[0*ORDH + 1] = yawRate * Ts;
                    Ah[0*ORDH + 2] = -Ts;
                    /* transpose Ah */
                    sMatTrsp(Ah, AtH, ORDH, ORDH);
                    /* x(k+1) = Ax(k) + Bu(k) */
                    sMatMult(Ah, xplusH, tempX1H, ORDH, ORDH, 1);		/* Ax */
                    for (i = 0; i < ORDH; i++) {
                        xplusH[i] = tempX1H[i];
                    }
    //RH 				sMatMult(Bh, uu, tempX2, ORD, 1/*ORDU*/, 1);		/* Bu */  These steps are unncessary, as B = [0 0 0]' and u = 0
    // 				sMatAdd(tempX1, tempX2, xplus, ORD, 1);	/* Ax(k) + Bu(k)*/
                    /* P(k+1) = AP(k)Ah' + Qh */
                    sMatMult(Ah, xplusH+ORDH, tempP1H, ORDH, ORDH, ORDH);	/* AP */
                    sMatMult(tempP1H, AtH, tempP2H, ORDH, ORDH, ORDH);	/* APA' */
                    sMatAdd(tempP2H, Qh, xplusH+ORDH, ORDH, ORDH);		/* APA' + Qh*/
                    /* increase the bufferH index */
                    buff_indexH = ++buff_indexH < BUFF_LENGTH ? buff_indexH : 0;
                    /* update the bufferH */
                    xplusH[0] = AngleMod180(xplusH[0]); /* angle to [-180,180) */
                    /*Temporarily store split integrated data in bufferH_SplitData*/
                    for (i = 0; i < NUMOFSTATESH; i++) {
                        bufferH_SplitData[buff_indexH*NUMOFSTATESH + i] = xplusH[i];
                    }
                }
                /*Store buff_indexH at last integration step for retrieval in next cycle*/
                last_indexH = buff_indexH;
                /*Since this code only runs for split_num > 1 the below code is not executed presently*/
                /*(This is used when the user specifies a split number of 1, to disable split integration)*/
                if (SplitCounterH == split_num) {
                    /*Copy data from bufferH_SplitData to bufferH_x upon completion of split integration*/
                    buff_indexH_copy = buff_counter - delayBeeline_global;
                    buff_indexH_copy += buff_indexH_copy < 0 ? BUFF_LENGTH : 0;
                    for (j = 0; j < delayBeeline_global; j++) {
                        for (i = 0; i < NUMOFSTATESH; i++) {
                            bufferH_x[buff_indexH_copy*NUMOFSTATESH + i] = bufferH_SplitData[buff_indexH_copy*NUMOFSTATESH + i];                    
                        }
                        buff_indexH_copy = ++buff_indexH_copy < BUFF_LENGTH ? buff_indexH_copy : 0;
                    }
                    /*Update current state with result from re-integration*/
                    for (i = 0; i < NUMOFSTATESH; i++) {
                        x[i] = xplusH[i];
                    }
                    /*Reset split integration flag and counter*/
                    SplitIntegrationH = FALS;
                    SplitCounterH = 0;    
                }               
            }
		}   /* if (fabs(AngleMod180(xhat[0] - yawAngle)) < diff_yaw) */
		else {
			diffYawCount++;
		}
	}   /* if (angleFlag && gpsUpdate) */
    
    /*If the SplitIntegrationH flag is active and there is no GPS update, resume integration*/
    if  (SplitIntegrationH && !gpsUpdateBeeline) {
        /*Retrieve last index integrated and associated state*/
        buff_indexH = last_indexH;
        buff_indexH += buff_indexH < 0 ? BUFF_LENGTH : 0; /* [0, BUFF_LENGTH) */
        for (i = 0; i < NUMOFSTATESH; i++)  {
            xplusH[i] = bufferH_SplitData[buff_indexH*NUMOFSTATESH + i];
        }
        /*Augment split counter*/
        SplitCounterH++;
        /*Integrate from last_indexH to last_indexH+SplitIntervals[SplitCounterH-1]
         *(From last_indexH to last_indexH+SplitIntervals[SplitCounterH-1]*/
        for (j = 0; j < SplitIntervals[SplitCounterH-1]; j++) {
                /* retrieve inputs */
//RH 				for (i = 0; i < ORDU; i++) {
// 					uu[i] = bufferH[buff_indexH*BUFF_NOSIGS + i];
// 				}
                uuH[0] = 0;	/* since ORDU is zero, no input */
                /* retrieve yawRate */
                yawRate = bufferH_yawRate[buff_indexH];
                /* define Ah */
                for (i = 0; i < ORDH*ORDH; i++) {
                    Ah[i] = 0;	/* initialize Ah */
                }
                Ah[0*ORDH + 0] = Ah[1*ORDH + 1] = Ah[2*ORDH + 2] = 1;	/* I */
                Ah[0*ORDH + 1] = yawRate * Ts;
                Ah[0*ORDH + 2] = -Ts;
                /* transpose Ah */
                sMatTrsp(Ah, AtH, ORDH, ORDH);
                /* x(k+1) = Ax(k) + Bu(k) */
                sMatMult(Ah, xplusH, tempX1H, ORDH, ORDH, 1);		/* Ax */
                for (i = 0; i < ORDH; i++) {
                    xplusH[i] = tempX1H[i];
                }
//RH 				sMatMult(Bh, uu, tempX2, ORD, 1/*ORDU*/, 1);		/* Bu */  These steps are unncessary, as B = [0 0 0]' and u = 0
// 				sMatAdd(tempX1, tempX2, xplus, ORD, 1);	/* Ax(k) + Bu(k)*/
                /* P(k+1) = AP(k)Ah' + Qh */
                sMatMult(Ah, xplusH+ORDH, tempP1H, ORDH, ORDH, ORDH);	/* AP */
                sMatMult(tempP1H, AtH, tempP2H, ORDH, ORDH, ORDH);	/* APA' */
                sMatAdd(tempP2H, Qh, xplusH+ORDH, ORDH, ORDH);		/* APA' + Qh*/
                /* increase the bufferH index */
                buff_indexH = ++buff_indexH < BUFF_LENGTH ? buff_indexH : 0;
                /* update the bufferH */
                xplusH[0] = AngleMod180(xplusH[0]); /* angle to [-180,180) */
                /*Temporarily store split integrated data in bufferH_SplitData*/
                for (i = 0; i < NUMOFSTATESH; i++) {
                    bufferH_SplitData[buff_indexH*NUMOFSTATESH + i] = xplusH[i];
                }
        }
        /*Store buff_indexH at last integration step for retrieval in next cycle, if necessary*/
        last_indexH = buff_indexH;
        /*If final cycle in split integration is completed:*/
        if (SplitCounterH == split_num) {
            /*Copy data from bufferH_SplitData to bufferH_x upon completion of split integration*/
            buff_indexH_copy = buff_counter - delayBeeline_global;
            buff_indexH_copy += buff_indexH_copy < 0 ? BUFF_LENGTH : 0;
            for (j = 0; j < delayBeeline_global; j++) {
                for (i = 0; i < NUMOFSTATESH; i++) {
                    bufferH_x[buff_indexH_copy*NUMOFSTATESH + i] = bufferH_SplitData[buff_indexH_copy*NUMOFSTATESH + i];                    
                }
                buff_indexH_copy = ++buff_indexH_copy < BUFF_LENGTH ? buff_indexH_copy : 0;
            }
            /*Update current state using result of re-integration*/
            for (i = 0; i < NUMOFSTATESH; i++) {
                x[i] = xplusH[i];
            }
            /*Reset split integration flag and counter*/
            SplitIntegrationH = FALS;
            SplitCounterH = 0;    
        }
    }
// 	/* increase the bufferH counter */
// 	bufferH_counter = ++buff_counter < BUFF_LENGTH ? buff_counter : 0;


	/*output states and convariances */
	y[0] = x[0];							/* yaw angle */
	y[1] = x[1] == 0 ? 0 : 1 / x[1];		/* sensitivity */
	y[2] = x[1] == 0 ? x[2] : x[2] / x[1];	/* bias */
	for (i = ORDH; i < NUMOFSTATESH; i++) {
		y[i] = x[i];
	}
	/* output corrected yaw rate */
	y[i] = yawRate * x[1] - x[2];	/* corrected yaw rate*/
//     bufferV_yawAngle[buff_counter] = y[0]*D2R;
//     bufferV_yawRate[buff_counter] = y[NUMOFSTATESH]*D2R;
    
    /*MDLOUPUTS FOR ROLL FILTER*/
    /*store roll rate*/
    bufferR_rollRate[buff_counter] = rollRate;
	/* store states and covariances */
	for (i = 0; i < NUMOFSTATESR; i++) {
//RH 		bufferR[buff_counter*BUFF_NOSIGS + ORDU + ORDV + i] = x[i];
        bufferR_x[buff_counter*NUMOFSTATESR+i] = x[NUMOFSTATESH+i];
	}
	/* start reintegrating after initialized and relevant data exists */
	if ((!initializedFlagR && buff_counter < delayBeeline) || delayBeeline < 0) {
		gpsUpdateBeeline = FALS;
	}

	if (angleFlagBeeline && gpsUpdateBeeline) { /* when new GPS measurement available */
		if (delayBeeline >= BUFF_LENGTH) {
			printf("Buffer is not of sufficient size... [rollfilter]\n");
		}
        /*If split integration was supposed to continue this time step, cancel, reset variables*/
        if (SplitIntegrationR) {
            SplitCounterR = 0;
            SplitIntegrationR = FALS;
        }
		buff_indexR = buff_counter - delayBeeline; /* account for delay */
		buff_indexR += buff_indexR < 0 ? BUFF_LENGTH : 0; /* [0, BUFF_LENGTH) */
		for (i = 0; i < NUMOFSTATESR; i++) {	/* retrieve states */
//RH 			xhat[i] = bufferR[buff_indexR*BUFF_NOSIGS + ORDU + ORDV + i];
            xhatR[i] = bufferR_x[buff_indexR*NUMOFSTATESR+i];
		}
		/* measurements */
		rollAngle = AngleMod180(rollAngle);  /* to the range of [-180,180) */
		/* at first valid GPS, initialize roll to GPS value */
		if (!initializedFlagR) {
		    for (i = 0; i < NUMOFSTATESR; i++) {
		        xhatR[i] = initStatesR[i];
		    }
			xhatR[0] = rollAngle;
			initializedFlagR = TRU;
		}
       
		/* checking that gps and ins roll are close. if not don't trust GPS */
		if (fabs(AngleMod180(xhatR[0] - rollAngle)) < diff_roll) {
			diff = xhatR[0] - rollAngle;
            diffRollCount = 0;
			/* Kalman Gain K = P(-)Cr'*inv[CP(-)Cr'+Rr] */
			sMatTrsp(Cr, CtR, ORDYR, ORDR);	/* transpose Cr */
			sMatMult(xhatR+ORDR, CtR, tempK1R, ORDR, ORDR, ORDYR);	/* P(-)*Cr' */
			sMatMult(Cr, xhatR+ORDR, tempK2R, ORDYR, ORDR, ORDR); 	/* CP(-) */
			sMatMult(tempK2R, CtR, tempK3R, ORDYR, ORDR, ORDYR);	/* CP(-)Cr' */
			sMatAdd(tempK3R, Rr, tempK4R, ORDYR, ORDYR);			/* CP(-)Cr'+Rr */
			sMatInv(tempK4R, tempK3R, ORDYR);				/* inv(CP(-)Cr'+Rr) */
			sMatMult(tempK1R, tempK3R, KR, ORDR, ORDYR, ORDYR);	/* K */
			/* error covariance update P(+) = [I - KC]P(-)*/
			sMatEye(tempEyeR, ORDR);							/* I */
			sMatMult(KR, Cr, tempP1R, ORDR, ORDYR, ORDR);			/* KC */
			sMatSub(tempEyeR, tempP1R, tempP2R, ORDR, ORDR);		/* I - KC */
			sMatMult(tempP2R, xhatR+ORDR, xplusR+ORDR, ORDR, ORDR, ORDR);	/* P(+) */
			/* state estimate update x(+) = x(-) + K[y_m - Cx(-)] */
			y_mR[0] = rollAngle;
			sMatMult(Cr, xhatR, tempY1R, ORDYR, ORDR, 1);	/* Cx(-) */
			sMatSub(y_mR, tempY1R, tempY2R, ORDYR, 1); 		/* y_m - Cx(-) */
			tempY2R[0] = AngleMod180(tempY2R[0]);		/* should be in range */
			sMatMult(KR, tempY2R, tempX1R, ORDR, ORDYR, 1);	/* K[y_m - Cx(-)] */
			sMatAdd(xhatR, tempX1R, xplusR, ORDR, 1);		/* x(+) */
			/* update the bufferR */
			xplusR[0] = AngleMod180(xplusR[0]); /* angle to [-180,180) */
            
			for (i = 0; i < NUMOFSTATESR; i++) {
//RH 				bufferR[buff_indexR*BUFF_NOSIGS + ORDU + ORDV + i] = xplus[i];
                bufferR_x[buff_indexR*NUMOFSTATESR+i] = xplusR[i];
			}
			/* reintegrate until the present time if beeline GPS is below split threshold*/
            if (delayBeeline < split_delay) {                
                for (j = 0; j < delayBeeline; j++) {
                    /* retrieve inputs */
                    sMatTrsp(Ar, AtR, ORDR, ORDR);	/* transpose Ar */
                    for (i = 0; i < ORDUR; i++) {
    //RH 					uu[i] = bufferR[buff_indexR*BUFF_NOSIGS + i];
                        uuR[i] = bufferR_rollRate[buff_indexR+i];
                    }                   
                    /* x(k+1) = Ax(k) + Bu(k) */
                    sMatMult(Ar, xplusR, tempX1R, ORDR, ORDR, 1);	/* Ax */
                    sMatMult(Br, uuR, tempX2R, ORDR, ORDUR, 1);		/* Bu */
                    sMatAdd(tempX1R, tempX2R, xplusR, ORDR, 1);	/* Ax(k) + Bu(k)*/
                    /* P(k+1) = AP(k)Ar' + Qr */
                    sMatMult(Ar, xplusR+ORDR, tempP1R, ORDR, ORDR, ORDR);	/* AP */
                    sMatMult(tempP1R, AtR, tempP2R, ORDR, ORDR, ORDR);	/* APA' */
                    sMatAdd(tempP2R, Qr, xplusR+ORDR, ORDR, ORDR);		/* APA' + Qr*/
                    /* increase the bufferR index */
                    buff_indexR = ++buff_indexR < BUFF_LENGTH ? buff_indexR : 0;
                    /* update the bufferR */
                    xplusR[0] = AngleMod180(xplusR[0]); /* angle to [-180,180) */
                    for (i = 0; i < NUMOFSTATESR; i++) {
    //RH 					bufferR[buff_indexR*BUFF_NOSIGS + ORDU + ORDV + i] = xplus[i];
                        bufferR_x[buff_indexR*NUMOFSTATESR+i] = xplusR[i];
                    }
                }   /* for (j = 0; j < delayBeeline; j++) */
                for (i = 0; i < NUMOFSTATESR; i++) {
                    x[NUMOFSTATESH+i] = xplusR[i];
                }
            }
            /*If delay is ABOVE split threshold..*/
            else {                               
                SplitCounterR++; /*Augment the split counter to 1*/
                SplitIntegrationR = TRU; /*Activate split integration flag*/
                /*Compute how many processor cycles will be used for integration*/ /*SPLIT INTERVALS ALREADY DEFINED IN HEADING FILTER*/
//                 rem = delayBeeline % split_delay;  
//                 split_num = (delayBeeline-rem)/split_delay + 1;
//                 /*Update delay to reflect additional processing cycles for integration*/
//                 delayBeeline = delayBeeline + (split_num-1);
//                 delayBeeline_global = delayBeeline;  
//                 /*Determine integration intervals for each processing cycle, according to rules above*/
//                 for (i = 0; i < split_num; i++) {
//                     if (i == split_num-1) {
//                         SplitIntervals[i] = rem + (split_num-1);
//                     }
//                     else {
//                         SplitIntervals[i] = split_delay;
//                     }
//                 }
                /*Old code, used when user specified split number*/                
//                 rem = delayBeeline % split_num;
//                 for (i = 0; i < split_num; i++) {
//                     if (i == split_num-1) {
//                         SplitIntervals[i] = (delayBeeline-rem)/split_num + rem;
//                     }
//                     else {
//                         SplitIntervals[i] = (delayBeeline-rem)/split_num;
//                     }
//                 }                
                sMatTrsp(Ar, AtR, ORDR, ORDR);	/* transpose Ar */
                /*Re-integrate from buff_indexR across first split interval 
                 (to buff_indexR+SplitIntervals[SplitCounterR-1]*/
                for (j = 0; j < SplitIntervals[SplitCounterR-1]; j++) {
                    /* retrieve inputs */
                    for (i = 0; i < ORDUR; i++) {
    //RH 					uu[i] = bufferR[buff_indexR*BUFF_NOSIGS + i];
                        uuR[i] = bufferR_rollRate[buff_indexR+i];
                    }
                    /* x(k+1) = Ax(k) + Bu(k) */
                    sMatMult(Ar, xplusR, tempX1R, ORDR, ORDR, 1);	/* Ax */
                    sMatMult(Br, uuR, tempX2R, ORDR, ORDUR, 1);		/* Bu */
                    sMatAdd(tempX1R, tempX2R, xplusR, ORDR, 1);	/* Ax(k) + Bu(k)*/
                    /* P(k+1) = AP(k)Ar' + Qr */
                    sMatMult(Ar, xplusR+ORDR, tempP1R, ORDR, ORDR, ORDR);	/* AP */
                    sMatMult(tempP1R, AtR, tempP2R, ORDR, ORDR, ORDR);	/* APA' */
                    sMatAdd(tempP2R, Qr, xplusR+ORDR, ORDR, ORDR);		/* APA' + Qr*/
                    /* increase the bufferR index */
                    buff_indexR = ++buff_indexR < BUFF_LENGTH ? buff_indexR : 0;
                    /* update the bufferR */
                    xplusR[0] = AngleMod180(xplusR[0]); /* angle to [-180,180) */
                    /*Temporarily store split integrated data in bufferR_SplitData*/
                    for (i = 0; i < NUMOFSTATESR; i++) {
                        bufferR_SplitData[buff_indexR*NUMOFSTATESR+i] = xplusR[i];
                    }
                }
                
                /*Store buff_indexR at last integration step for retrieval in next cycle*/
                last_indexR = buff_indexR;
                /*Since this code only runs for split_num > 1 the below code is not executed presently*/
                /*(This is used when the user specifies a split number of 1, to disable split integration)*/
                if (SplitCounterR == split_num) {
                    /*Copy data from bufferR_SplitData to bufferR_x upon completion of split integration*/
                    buff_indexR_copy = buff_counter - delayBeeline_global;
                    buff_indexR_copy += buff_indexR_copy < 0 ? BUFF_LENGTH : 0;
                    for (j = 0; j < delayBeeline_global; j++) {
                        for (i = 0; i < NUMOFSTATESR; i++) {
                            bufferR_x[buff_indexR_copy*NUMOFSTATESR + i] = bufferR_SplitData[buff_indexR_copy*NUMOFSTATESR + i];                    
                        }
                        buff_indexR_copy = ++buff_indexR_copy < BUFF_LENGTH ? buff_indexR_copy : 0;
                    }
                    /*Update current state with result from re-integration*/
                    for (i = 0; i < NUMOFSTATESR; i++) {
                        x[NUMOFSTATESH+i] = xplusR[i];
                    }
                    SplitIntegrationR = FALS;
                    SplitCounterR = 0;    
                }
            }
//             }

		}   /* if (fabs(AngleMod180(xhat[0] - rollAngle)) < diff_roll) */
		else {
			diffRollCount++;
		}
	}   /* if (angleFlag && gpsUpdate) */
    
    /*If the split integration flag is active and there is no GPS update, resume integration*/
    if  (SplitIntegrationR && !gpsUpdateBeeline) {        
        /*Retrieve last index integrated and associated state*/
        buff_indexR = last_indexR;
        buff_indexR += buff_indexR < 0 ? BUFF_LENGTH : 0; /* [0, BUFF_LENGTH) */
        for (i = 0; i < NUMOFSTATESR; i++)  {
            xplusR[i] = bufferR_SplitData[buff_indexR*NUMOFSTATESR+i];
        }
        /*Augment split counter*/
        SplitCounterR++;
        /*Integrate from last_indexR across SplitIntervals[SplitCounterR-1]
         *(From last_indexR to last_indexR+SplitIntervals[SplitCounterR-1]*/
        for (j = 0; j < SplitIntervals[SplitCounterR-1]; j++) {
            sMatTrsp(Ar, AtR, ORDR, ORDR);	/* transpose Ar */
            /* retrieve inputs */
            for (i = 0; i < ORDUR; i++) {
//RH 					uu[i] = bufferR[buff_indexR*BUFF_NOSIGS + i];
                uuR[i] = bufferR_rollRate[buff_indexR+i];
            }
            /* x(k+1) = Ax(k) + Bu(k) */
            sMatMult(Ar, xplusR, tempX1R, ORDR, ORDR, 1);	/* Ax */
            sMatMult(Br, uuR, tempX2R, ORDR, ORDUR, 1);		/* Bu */
            sMatAdd(tempX1R, tempX2R, xplusR, ORDR, 1);	/* Ax(k) + Bu(k)*/
            /* P(k+1) = AP(k)Ar' + Qr */
            sMatMult(Ar, xplusR+ORDR, tempP1R, ORDR, ORDR, ORDR);	/* AP */
            sMatMult(tempP1R, AtR, tempP2R, ORDR, ORDR, ORDR);	/* APA' */
            sMatAdd(tempP2R, Qr, xplusR+ORDR, ORDR, ORDR);		/* APA' + Qr*/
            /* increase the bufferR index */
            buff_indexR = ++buff_indexR < BUFF_LENGTH ? buff_indexR : 0;
            /* update the bufferR */
            xplusR[0] = AngleMod180(xplusR[0]); /* angle to [-180,180) */
            /*Temporarily store split integrated data in bufferR_SplitData*/
            for (i = 0; i < NUMOFSTATESR; i++) {
//RH 					bufferR[buff_indexR*BUFF_NOSIGS + ORDU + ORDV + i] = xplus[i];
                bufferR_SplitData[buff_indexR*NUMOFSTATESR+i] = xplusR[i];
            }
        }
        /*Store buff_indexR at last integration step for retrieval in next cycle, if necessary*/
        last_indexR = buff_indexR;
        /*If final cycle in split integration is completed:*/
        if (SplitCounterR == split_num) {
            /*Copy data from bufferR_SplitData to bufferR_x upon completion of split integration*/
            buff_indexR_copy = buff_counter - delayBeeline_global;
            buff_indexR_copy += buff_indexR_copy < 0 ? BUFF_LENGTH : 0;
            for (j = 0; j < delayBeeline_global; j++) {
                for (i = 0; i < NUMOFSTATESR; i++) {
                    bufferR_x[buff_indexR_copy*NUMOFSTATESR + i] = bufferR_SplitData[buff_indexR_copy*NUMOFSTATESR + i];                    
                }
                buff_indexR_copy = ++buff_indexR_copy < BUFF_LENGTH ? buff_indexR_copy : 0;
            }
            /*Update current state using result of re-integration*/
            for (i = 0; i < NUMOFSTATESR; i++) {
                x[NUMOFSTATESH+i] = xplusR[i];
            }
            /*Reset split integration flag and counter*/
            SplitIntegrationR = FALS;
            SplitCounterR = 0;    
        }
    }

// 	/* increase the bufferR counter */
// 	bufferR_counter = ++buff_counter < BUFF_LENGTH ? buff_counter : 0;

	/* output states and convariances */
	for (i = 0; i < NUMOFSTATESR; i++) {
		y[NUMOFSTATESH+1+i] = x[NUMOFSTATESH+i];
	}
	/* output corrected roll rate */
	y[NUMOFSTATESH+1+i] = rollRate - x[NUMOFSTATESH+1];	/* measured roll rate - bias*/
//     bufferV_rollRate[buff_counter] = y[NUMOFSTATESH+1+6]*D2R;
//     bufferV_rollAngle[buff_counter] = y[NUMOFSTATESH+1]*D2R;

    /*BE SURE TO CONVERT ATTITUDE FILTER OUTPUTS TO RAD FOR VELFILTER!*/
    /*MDLOUTPUTS FOR VELOCITY FILTER*/
    	/* grade from velocity ratio only when moving and gps is good */
    buff_indexV = buff_counter - delayOEM4; //-1; 			/* account for delay */
	buff_indexV += buff_indexV < 0 ? BUFF_LENGTH : 0; /* [0, BUFF_LENGTH) */
    
    /* at first valid GPS, initialize velocities to GPS value */
    /* CEB: This should be moved to where it will execute regardless
     * of simultaneous update status or not, otherwise it may
     * never get executed. */
    if (!initializedFlagV && gpsFlagOEM4 && latency <= MAX_LATENCY) {
    	/* start reintegrating after initialized and relevant data exists */
        if (buff_counter < delayOEM4) {
            gpsUpdateOEM4 = FALS;
        }
        else {	/* check whether yaw and roll is ready */
            if(!initializedFlagH || !initializedFlagR) {
            //if (bufferV_u[buff_indexV+2] >= PI || bufferH_x[buff_indexV*NUMOFSTATESH]*D2R >= PI) {
                gpsUpdateOEM4 = FALS;
            }
            else {
                for (i = 0; i < NUMOFSTATESV; i++) {
                    xhatV[i] = initStatesV[i];
                }
                /* define the angle between velocity vector and vehicle heading */
                slipAngleGPS = gpsVdir - yawAngle;
                /* measurements */
                VxGPS =  gpsSpeed * cos(slipAngleGPS);	/* Vx */
                VyGPS =  gpsSpeed * sin(slipAngleGPS);	/* Vy */
                
                xhatV[0] = VxGPS;
                xhatV[2] = VyGPS;
                
                initializedFlagV = TRU;
                
            }
        }
    }
    /* Determine whether there is enough excitation to determine the
     * road grade from GPS data. If so, calculate it by the relative
     * horizontal and vertical speeds. */
	if (gpsSpeed > 1 && gpsFlagOEM4 && latency <= MAX_LATENCY) {
		gpsGradeV = atan2(gpsVerSpeed, gpsSpeed);
	}
    else { // if not enough excitation, output a zero
		gpsGradeV = 0;
		for (i = 1; i <= num_grade_avg; i++) { /* average last values */
			j = buff_indexV - i;
			j += j < 0 ? BUFF_LENGTH : 0;
//RH 			gpsGradeV += bufferV[j*BUFF_NOSIGS + 3];
            gpsGradeV += bufferV_u[j*ORDUV + 3];
		}
		gpsGradeV /= num_grade_avg;
	}
    
    bufferV_u[buff_counter*ORDUV] = ax;
    bufferV_u[buff_counter*ORDUV + 1] = ay;
    bufferV_u[buff_counter*ORDUV + 2] = x[NUMOFSTATESH]*D2R; /*Roll Angle-This is updated later on using bufferR_x*/    
//     bufferV_u[buff_counter*ORDUV + 2] = bufferV_rollAngle[buff_counter];
    bufferV_u[buff_counter*ORDUV + 3] = gpsGradeV;
    
	/* store states and covariances */
	for (i = 0; i < NUMOFSTATESV; i++) {
// 		bufferV[buff_counter*BUFF_NOSIGS + ORDU + ORDV + i] = x[i];
        bufferV_x[buff_counter*NUMOFSTATESV+i] = x[NUMOFSTATESH+NUMOFSTATESR+i];
	}

	/* check if latency is too big or delay is negative */
	if (latency > MAX_LATENCY || delayOEM4 < 0) {
		gpsUpdateOEM4 = FALS;
	}
	
    if (gpsFlagOEM4 && gpsUpdateOEM4 && !SimulUpdate) { /* when new GPS measurement available */
        SimulUpdate_prev = FALS;
        if (delayOEM4 >= BUFF_LENGTH) {
			printf("Buffer is not of sufficient size... [hdgfilter]\n");
		}

        yawRateCorr = (bufferH_yawRate[buff_indexV]*bufferH_x[buff_indexV*NUMOFSTATESH+1]-bufferH_x[buff_indexV*NUMOFSTATESH+2])*D2R;
        yawAngle = bufferH_x[buff_indexV*NUMOFSTATESH]*D2R;
        rollRateCorr = (bufferR_rollRate[buff_indexV]-bufferR_x[buff_indexV*NUMOFSTATESR+1])*D2R; /*Correct stored roll rate by bias*/
        
//         yawRateCorr = bufferV_yawRate[buff_indexV];
//         rollRateCorr = bufferV_rollRate[buff_indexV];
//         yawAngle = bufferV_yawAngle[buff_indexV];
        
		for (i = 0; i < NUMOFSTATESV; i++) {	/* retrieve states */
//RH 			xhat[i] = bufferV[buff_indexV*BUFF_NOSIGS + ORDU + ORDV + i];
            xhatV[i] = bufferV_x[buff_indexV*NUMOFSTATESV + i];
		}
		/* define the angle between velocity vector and vehicle heading */
		slipAngleGPS = gpsVdir - yawAngle;
		/* measurements */
        VxGPS =  gpsSpeed * cos(slipAngleGPS);	/* Vx */
        VyGPS =  gpsSpeed * sin(slipAngleGPS);	/* Vy */
        VyGPS += h_gps_ant * rollRateCorr;	/* roll rate correction */
        VyGPS -= l_gps_ant * yawRateCorr;	/* yaw rate correction */
        
		/* checking that gps and ins yaw are close. if not don't trust GPS */
        if (fabs(xhatV[0] - VxGPS) < diff_vel_x
				&& fabs(xhatV[2] - VyGPS) < diff_vel_y) {
			diffVelCount = 0;
            
			/* Kalman Gain K = P(-)Cv'*inv[CP(-)Cv'+Rv] */
			sMatTrsp(Cv, CtV, ORDYV, ORDV);	/* transpose Cv */
			sMatMult(xhatV+ORDV, CtV, tempK1V, ORDV, ORDV, ORDYV);	/* P(-)*Cv' */
			sMatMult(Cv, xhatV+ORDV, tempK2V, ORDYV, ORDV, ORDV); 	/* CP(-) */
			sMatMult(tempK2V, CtV, tempK3V, ORDYV, ORDV, ORDYV);	/* CP(-)Cv' */
			sMatAdd(tempK3V, Rv, tempK4V, ORDYV, ORDYV);			/* CP(-)Cv'+Rv */
			sMatInv(tempK4V, tempK3V, ORDYV);				/* inv(CP(-)Cv'+Rv) */
			sMatMult(tempK1V, tempK3V, KV, ORDV, ORDYV, ORDYV);	/* K */
			/* error covariance update P(+) = [I - KC]P(-)*/
			sMatEye(tempEyeV, ORDV);							/* I */
			sMatMult(KV, Cv, tempP1V, ORDV, ORDYV, ORDV);			/* KC */
			sMatSub(tempEyeV, tempP1V, tempP2V, ORDV, ORDV);		/* I - KC */
			sMatMult(tempP2V, xhatV+ORDV, xplusV+ORDV, ORDV, ORDV, ORDV);	/* P(+) */
			/* state estimate update x(+) = x(-) + K[y_m - Cx(-)] */
			y_mV[0] = VxGPS;	y_mV[1] = VyGPS;
			sMatMult(Cv, xhatV, tempY1V, ORDYV, ORDV, 1);	/* Cx(-) */
			sMatSub(y_mV, tempY1V, tempY2V, ORDYV, 1); 		/* y_m - Cx(-) */
			sMatMult(KV, tempY2V, tempX1V, ORDV, ORDYV, 1);	/* K[y_m - Cx(-)] */
			sMatAdd(xhatV, tempX1V, xplusV, ORDV, 1);		/* x(+) */
			/* update the bufferV */
			for (i = 0; i < NUMOFSTATESV; i++) {
//RH 				bufferV[buff_indexV*BUFF_NOSIGS + ORDU + ORDV + i] = xplus[i];
                bufferV_x[buff_indexV*NUMOFSTATESV + i] = xplusV[i];
			}
			/* reintegrate until the present time */
			for (j = 0; j < delayOEM4; j++) {
				/* retrieve inputs */
				for (i = 0; i < ORDUV; i++) {
//RH 					uu[i] = bufferV[buff_indexV*BUFF_NOSIGS + i];
                    uuV[i] = bufferV_u[buff_indexV*ORDUV + i];
				}
				uuV[2] = bufferR_x[buff_indexV*NUMOFSTATESR]*D2R; 
                uuV[3] = gpsGradeV;	/* gpsGradeV due to delay */
                
				/* retrieve yawRate with bias correction */
                yawRateCorr = (bufferH_yawRate[buff_indexV]*bufferH_x[buff_indexV*NUMOFSTATESH+1]-bufferH_x[buff_indexV*NUMOFSTATESH+2])*D2R; 
//                 yawRateCorr = bufferV_yawRate[buff_indexV];
				/* define Av */
				for (i = 0; i < ORDV*ORDV; i++) {
					Av[i] = 0;	/* initialize Av */
				}
				Av[0*ORDV+0] = cos(yawRateCorr*Ts);
				Av[0*ORDV+1] = yawRateCorr==0 ? -Ts : -sin(yawRateCorr*Ts) / yawRateCorr;
				Av[0*ORDV+2] = sin(yawRateCorr*Ts);
				Av[0*ORDV+3] = yawRateCorr==0 ? 0 : (cos(yawRateCorr*Ts) - 1) / yawRateCorr;
				Av[1*ORDV+1] = 1;
				Av[2*ORDV+0] = -sin(yawRateCorr*Ts);
				Av[2*ORDV+1] = yawRateCorr==0 ? 0 : (1 - cos(yawRateCorr*Ts)) / yawRateCorr;
				Av[2*ORDV+2] = cos(yawRateCorr*Ts);
				Av[2*ORDV+3] = yawRateCorr==0 ? -Ts : -sin(yawRateCorr*Ts) / yawRateCorr;
				Av[3*ORDV+3] = 1;
				/* define Bv */
				for (i = 0; i < ORDV*ORDUV; i++) {
					Bv[i] = 0;	/* initialize Bv */
				}
				Bv[0*ORDUV+0] = yawRateCorr==0 ? Ts : sin(yawRateCorr*Ts) / yawRateCorr;
				Bv[0*ORDUV+1] = yawRateCorr==0 ? 0 : (1 - cos(yawRateCorr*Ts)) / yawRateCorr;
				Bv[0*ORDUV+2] = yawRateCorr==0 ? 0 : G*(cos(yawRateCorr*Ts)-1) / yawRateCorr;
				Bv[0*ORDUV+3] = yawRateCorr==0 ? -G*Ts : -G*sin(yawRateCorr*Ts)/ yawRateCorr;
				Bv[2*ORDUV+0] = yawRateCorr==0 ? 0 : (cos(yawRateCorr*Ts) - 1) / yawRateCorr;
				Bv[2*ORDUV+1] = yawRateCorr==0 ? Ts : sin(yawRateCorr*Ts) / yawRateCorr;
				Bv[2*ORDUV+2] = yawRateCorr==0 ? -G*Ts : -G*sin(yawRateCorr*Ts)/ yawRateCorr;
				Bv[2*ORDUV+3] = yawRateCorr==0 ? 0 : G*(1-cos(yawRateCorr*Ts)) / yawRateCorr;
				/* transpose Av */
				sMatTrsp(Av, AtV, ORDV, ORDV);
				/* x(k+1) = Ax(k) + Bu(k) */
				sMatMult(Av, xplusV, tempX1V, ORDV, ORDV, 1);	/* Ax */
				sMatMult(Bv, uuV, tempX2V, ORDV, ORDUV, 1);		/* Bu */
				sMatAdd(tempX1V, tempX2V, xplusV, ORDV, 1);		/* Ax(k) + Bu(k)*/
				/* P(k+1) = AP(k)Av' + Qv */
				sMatMult(Av, xplusV+ORDV, tempP1V, ORDV, ORDV, ORDV);	/* AP */
				sMatMult(tempP1V, AtV, tempP2V, ORDV, ORDV, ORDV);	/* APA' */
				sMatAdd(tempP2V, Qv, xplusV+ORDV, ORDV, ORDV);		/* APA' + Qv*/
				/* increase the bufferV index */
				buff_indexV = ++buff_indexV < BUFF_LENGTH ? buff_indexV : 0;
				/* update the bufferV */
				for (i = 0; i < NUMOFSTATESV; i++) {
//RH 					bufferV[buff_indexV*BUFF_NOSIGS + ORDU + ORDV + i] = xplus[i];
                    bufferV_x[buff_indexV*NUMOFSTATESV + i] = xplusV[i];
				}
			}   /* for (j = 0; j < delay; j++) */
			for (i = 0; i < NUMOFSTATESV; i++) {
				x[NUMOFSTATESH+NUMOFSTATESR+i] = xplusV[i];
			}
		}   /* if (fabs(xhat[0] - VxGPS) < diff_vel_x */
		else {
			diffVelCount++;
		}
	}   /* if (gpsFlag && gpsUpdate) */
    
    if (SimulUpdate_prev && !SimulUpdate && initializedFlagV) {
        buff_indexV_copy = buff_counter - delayOEM4_global;
        buff_indexV_copy += buff_indexV_copy < 0 ? BUFF_LENGTH : 0; /* [0, BUFF_LENGTH) */
        
        if (delayOEM4 >= BUFF_LENGTH) {
			printf("Buffer is not of sufficient size... [hdgfilter]\n");
		}

        yawRateCorr = (bufferH_yawRate[buff_indexV_copy]*bufferH_x[buff_indexV_copy*NUMOFSTATESH+1]-bufferH_x[buff_indexV_copy*NUMOFSTATESH+2])*D2R;
        yawAngle = bufferH_x[buff_indexV_copy*NUMOFSTATESH]*D2R;
        rollRateCorr = (bufferR_rollRate[buff_indexV_copy]-bufferR_x[buff_indexV_copy*NUMOFSTATESR+1])*D2R; /*Correct stored roll rate by bias*/
        
//         yawRateCorr = bufferV_yawRate[buff_indexV_copy];
//         rollRateCorr = bufferV_rollRate[buff_indexV_copy];
//         yawAngle = bufferV_yawAngle[buff_indexV_copy];
        
		for (i = 0; i < NUMOFSTATESV; i++) {	/* retrieve states */
//RH 			xhat[i] = bufferV[buff_indexV_copy*BUFF_NOSIGS + ORDU + ORDV + i];
            xhatV[i] = bufferV_x[buff_indexV_copy*NUMOFSTATESV + i];
		}
        /* define the angle between velocity vector and vehicle heading */
		slipAngleGPS = gpsVdir - yawAngle;
		/* measurements */
        VxGPS =  gpsSpeed * cos(slipAngleGPS);	/* Vx */
        VyGPS =  gpsSpeed * sin(slipAngleGPS);	/* Vy */
        VyGPS += h_gps_ant * rollRateCorr;	/* roll rate correction */
        VyGPS -= l_gps_ant * yawRateCorr;	/* yaw rate correction */
		/* at first valid GPS, initialize velocities to GPS value */
// 		if (!initializedFlagV) {
// 		    for (i = 0; i < NUMOFSTATESV; i++) {
// 		        xhatV[i] = initStatesV[i];
// 		    }
// 			xhatV[0] = VxGPS;
// 			xhatV[2] = VyGPS;
// 			initializedFlagV = TRU;
// 		}
		/* checking that gps and ins yaw are close. if not don't trust GPS */
        
		if (fabs(xhatV[0] - VxGPS) < diff_vel_x || fabs(xhatV[2] - VyGPS) < diff_vel_y) {
			diffVelCount = 0;
			/* Kalman Gain K = P(-)Cv'*inv[CP(-)Cv'+Rv] */
			sMatTrsp(Cv, CtV, ORDYV, ORDV);	/* transpose Cv */
			sMatMult(xhatV+ORDV, CtV, tempK1V, ORDV, ORDV, ORDYV);	/* P(-)*Cv' */
			sMatMult(Cv, xhatV+ORDV, tempK2V, ORDYV, ORDV, ORDV); 	/* CP(-) */
			sMatMult(tempK2V, CtV, tempK3V, ORDYV, ORDV, ORDYV);	/* CP(-)Cv' */
			sMatAdd(tempK3V, Rv, tempK4V, ORDYV, ORDYV);			/* CP(-)Cv'+Rv */
			sMatInv(tempK4V, tempK3V, ORDYV);				/* inv(CP(-)Cv'+Rv) */
			sMatMult(tempK1V, tempK3V, KV, ORDV, ORDYV, ORDYV);	/* K */
			/* error covariance update P(+) = [I - KC]P(-)*/
			sMatEye(tempEyeV, ORDV);							/* I */
			sMatMult(KV, Cv, tempP1V, ORDV, ORDYV, ORDV);			/* KC */
			sMatSub(tempEyeV, tempP1V, tempP2V, ORDV, ORDV);		/* I - KC */
			sMatMult(tempP2V, xhatV+ORDV, xplusV+ORDV, ORDV, ORDV, ORDV);	/* P(+) */
			/* state estimate update x(+) = x(-) + K[y_m - Cx(-)] */
			y_mV[0] = VxGPS;	y_mV[1] = VyGPS;
			sMatMult(Cv, xhatV, tempY1V, ORDYV, ORDV, 1);	/* Cx(-) */
			sMatSub(y_mV, tempY1V, tempY2V, ORDYV, 1); 		/* y_m - Cx(-) */
			sMatMult(KV, tempY2V, tempX1V, ORDV, ORDYV, 1);	/* K[y_m - Cx(-)] */
			sMatAdd(xhatV, tempX1V, xplusV, ORDV, 1);		/* x(+) */
			/* update the bufferV */
			for (i = 0; i < NUMOFSTATESV; i++) {
//RH 				bufferV[buff_indexV_copy*BUFF_NOSIGS + ORDU + ORDV + i] = xplus[i];
                bufferV_x[buff_indexV_copy*NUMOFSTATESV + i] = xplusV[i];
			}
			/* reintegrate until the present time */
			for (j = 0; j < delayOEM4; j++) {
				/* retrieve inputs */
				for (i = 0; i < ORDUV; i++) {
//RH 					uu[i] = bufferV[buff_indexV_copy*BUFF_NOSIGS + i];
                    uuV[i] = bufferV_u[buff_indexV_copy*ORDUV + i];
				}
				uuV[2] = bufferR_x[buff_indexV_copy*NUMOFSTATESR]*D2R; 
                uuV[3] = gpsGradeV;	/* gpsGradeV due to delay */
                
				/* retrieve yawRate with bias correction */
                yawRateCorr = (bufferH_yawRate[buff_indexV_copy]*bufferH_x[buff_indexV_copy*NUMOFSTATESH+1]-bufferH_x[buff_indexV_copy*NUMOFSTATESH+2])*D2R; 
//                 yawRateCorr = bufferV_yawRate[buff_indexV_copy];
				/* define Av */
				for (i = 0; i < ORDV*ORDV; i++) {
					Av[i] = 0;	/* initialize Av */
				}
				Av[0*ORDV+0] = cos(yawRateCorr*Ts);
				Av[0*ORDV+1] = yawRateCorr==0 ? -Ts : -sin(yawRateCorr*Ts) / yawRateCorr;
				Av[0*ORDV+2] = sin(yawRateCorr*Ts);
				Av[0*ORDV+3] = yawRateCorr==0 ? 0 : (cos(yawRateCorr*Ts) - 1) / yawRateCorr;
				Av[1*ORDV+1] = 1;
				Av[2*ORDV+0] = -sin(yawRateCorr*Ts);
				Av[2*ORDV+1] = yawRateCorr==0 ? 0 : (1 - cos(yawRateCorr*Ts)) / yawRateCorr;
				Av[2*ORDV+2] = cos(yawRateCorr*Ts);
				Av[2*ORDV+3] = yawRateCorr==0 ? -Ts : -sin(yawRateCorr*Ts) / yawRateCorr;
				Av[3*ORDV+3] = 1;
				/* define Bv */
				for (i = 0; i < ORDV*ORDUV; i++) {
					Bv[i] = 0;	/* initialize Bv */
				}
				Bv[0*ORDUV+0] = yawRateCorr==0 ? Ts : sin(yawRateCorr*Ts) / yawRateCorr;
				Bv[0*ORDUV+1] = yawRateCorr==0 ? 0 : (1 - cos(yawRateCorr*Ts)) / yawRateCorr;
				Bv[0*ORDUV+2] = yawRateCorr==0 ? 0 : G*(cos(yawRateCorr*Ts)-1) / yawRateCorr;
				Bv[0*ORDUV+3] = yawRateCorr==0 ? -G*Ts : -G*sin(yawRateCorr*Ts)/ yawRateCorr;
				Bv[2*ORDUV+0] = yawRateCorr==0 ? 0 : (cos(yawRateCorr*Ts) - 1) / yawRateCorr;
				Bv[2*ORDUV+1] = yawRateCorr==0 ? Ts : sin(yawRateCorr*Ts) / yawRateCorr;
				Bv[2*ORDUV+2] = yawRateCorr==0 ? -G*Ts : -G*sin(yawRateCorr*Ts)/ yawRateCorr;
				Bv[2*ORDUV+3] = yawRateCorr==0 ? 0 : G*(1-cos(yawRateCorr*Ts)) / yawRateCorr;
				/* transpose Av */
				sMatTrsp(Av, AtV, ORDV, ORDV);
				/* x(k+1) = Ax(k) + Bu(k) */
				sMatMult(Av, xplusV, tempX1V, ORDV, ORDV, 1);	/* Ax */
				sMatMult(Bv, uuV, tempX2V, ORDV, ORDUV, 1);		/* Bu */
				sMatAdd(tempX1V, tempX2V, xplusV, ORDV, 1);		/* Ax(k) + Bu(k)*/
				/* P(k+1) = AP(k)Av' + Qv */
				sMatMult(Av, xplusV+ORDV, tempP1V, ORDV, ORDV, ORDV);	/* AP */
				sMatMult(tempP1V, AtV, tempP2V, ORDV, ORDV, ORDV);	/* APA' */
				sMatAdd(tempP2V, Qv, xplusV+ORDV, ORDV, ORDV);		/* APA' + Qv*/
				/* increase the bufferV index */
				buff_indexV_copy = ++buff_indexV_copy < BUFF_LENGTH ? buff_indexV_copy : 0;
				/* update the bufferV */
				for (i = 0; i < NUMOFSTATESV; i++) {
//RH 					bufferV[buff_indexV_copy*BUFF_NOSIGS + ORDU + ORDV + i] = xplus[i];
                    bufferV_x[buff_indexV_copy*NUMOFSTATESV + i] = xplusV[i];
				}
			}   /* for (j = 0; j < delay; j++) */
			for (i = 0; i < NUMOFSTATESV; i++) {
				x[NUMOFSTATESH+NUMOFSTATESR+i] = xplusV[i];
			}
		}   /* if (fabs(xhat[0] - VxGPS) < diff_vel_x */
		else {
			diffVelCount++;
		}
    }
    
    if (SimulUpdate) {
        if (SimulUpdate_prev) {
            delayOEM4_global = delayOEM4_global+1;
        }
        else  {
            delayOEM4_global = delayOEM4 + 1;
        }
    }

	/* output states and convariances */ /*DOUBLE CHECK CORRECTNESS OF INDICES HERE*/
	for (i = 0; i < NUMOFSTATESV; i++) {
		y[NUMOFSTATESH+NUMOFSTATESR+2+i] = x[NUMOFSTATESH+NUMOFSTATESR+i];
	}
    rollAngle = x[NUMOFSTATESH]*D2R;
    yawRateCorr = y[NUMOFSTATESH]*D2R;
    rollRateCorr  = y[NUMOFSTATESH+NUMOFSTATESR+1]*D2R;
//     rollAngle = bufferV_rollAngle[buff_counter];
//     yawRateCorr = bufferV_yawRate[buff_counter];
//     rollRateCorr = bufferV_rollRate[buff_counter];
	/* output Vy and slip angle at CG */
	/* VyCg = x[2] + yawRateCorr * dist_sensor; */
	VyCg = x[NUMOFSTATESH+NUMOFSTATESR+2] + yawRateCorr * dist_sensor + rollRateCorr * height_sensor;
	slipAngleCg = atan2(VyCg, y[NUMOFSTATESH+NUMOFSTATESR+2+0]) * R2D;
	y[40] = slipAngleCg;
	y[41] = VyCg;
	/* output ay and ax at CG */
    axCg = ax - x[NUMOFSTATESH+NUMOFSTATESR+1]- G * sin(gpsGradeV)- yawRateCorr * yawRateCorr * dist_sensor - rollRateCorr * yawRateCorr * height_sensor;
    ayCg = ay - x[NUMOFSTATESH+NUMOFSTATESR+3] - G * sin(rollAngle);
	y[42] = axCg;
	y[43] = ayCg;
	if ((x[NUMOFSTATESH+NUMOFSTATESR+0] * x[NUMOFSTATESH+NUMOFSTATESR+0] + x[NUMOFSTATESH+NUMOFSTATESR+2] * x[NUMOFSTATESH+NUMOFSTATESR+2]) < 1) {
	    y[21] = 0;
        y[22] = 0;
	    y[40] = 0;
	    y[41] = 0;
	}
    buff_counter = ++buff_counter < BUFF_LENGTH ? buff_counter : 0;   
}

#define MDL_UPDATE  /* Change to #undef to remove function */
static void mdlUpdate(SimStruct *S, int_T tid)
{
    real_T *x = ssGetRealDiscStates(S);
    real_T t = ssGetT(S);
    real_T tr;
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    const real_T Ts = mxGetPr(ssGetSFcnParam(S,30))[0];
    int_T i;
    
    /*HEADING FILTER DECLARATIONS*/
    real_T AtH[ORDH*ORDH], uuH[1/*ORDU*/], xH[NUMOFSTATESH];	/* since ORDU is 0 */
	real_T tempX1H[ORDH], tempX2H[ORDH], tempP1H[ORDH*ORDH], tempP2H[ORDH*ORDH];
	real_T yawRate;    

    const real_T sens_varH = mxGetPr(ssGetSFcnParam(S,6))[0];
    const real_T timeconst1H = mxGetPr(ssGetSFcnParam(S,7))[0];
    const real_T timeconst2H = mxGetPr(ssGetSFcnParam(S,8))[0];
    
    /*ROLL FILTER DECLARATIONS*/
    real_T AtR[ORDR*ORDR], uuR[ORDUR], xR[NUMOFSTATESR];
	real_T tempX1R[ORDR], tempX2R[ORDR], tempP1R[ORDR*ORDR], tempP2R[ORDR*ORDR];
    
    const real_T sens_varR = mxGetPr(ssGetSFcnParam(S,12))[0];
    const real_T timeconstR = mxGetPr(ssGetSFcnParam(S,13))[0];
    
    /*VELOCITY FILTER DECLARATIONS*/
    real_T AtV[ORDV*ORDV], uuV[ORDUV], xV[NUMOFSTATESV];
	real_T tempX1V[ORDV], tempX2V[ORDV], tempP1V[ORDV*ORDV], tempP2V[ORDV*ORDV];
	real_T yawRateCorr;
    
    const real_T sens1_varV = mxGetPr(ssGetSFcnParam(S,21))[0];
    const real_T sens2_varV = mxGetPr(ssGetSFcnParam(S,22))[0];
    const real_T timeconst1V = mxGetPr(ssGetSFcnParam(S,23))[0];
    const real_T timeconst2V = mxGetPr(ssGetSFcnParam(S,24))[0];
    
    /*Separate out states for each filter*/
    for (i = 0; i < NUMOFSTATESH; i++) {
        xH[i] = x[i];
    }
    for (i = 0; i < NUMOFSTATESR; i++) {
        xR[i] = x[NUMOFSTATESH+i];
    }
    for (i= 0; i< NUMOFSTATESV; i++) {
        xV[i] = x[NUMOFSTATESH+NUMOFSTATESR+i];
    }
    
    tr = floor(t*1000+0.5);
    
    
    /*Define constant-valued co-variance and system matrices*/
    Qh[0] = sens_varH*Ts;
    Qh[1] = 0;
    Qh[2] = 0;
    Qh[3] = 0;
    Qh[4] = timeconst1H*Ts;
    Qh[5] = 0;
    Qh[6] = 0;
    Qh[7] = 0;
    Qh[8] = timeconst2H*Ts;
    
    Qr[0] = sens_varR*Ts;
    Qr[1] = 0;
    Qr[2] = 0;
    Qr[3] = timeconstR*Ts;
        
    Ar[0] = 1;
    Ar[1] = -Ts;
    Ar[2] = 0;
    Ar[3] = 1;
    
    Br[0] = Ts;
    Br[1] = 0;
    
    Qv[0] = sens1_varV*Ts;
    Qv[1] = 0;
    Qv[2] = 0;
    Qv[3] = 0;
    Qv[4] = 0;
    Qv[5] = timeconst1V*Ts;
    Qv[6] = 0;
    Qv[7] = 0;
    Qv[8] = 0;
    Qv[9] = 0;
    Qv[10] = sens2_varV*Ts;
    Qv[11] = 0;
    Qv[12] = 0;
    Qv[13] = 0;
    Qv[14] = 0;
    Qv[15] = timeconst2V*Ts;
  
    /*HEADING FILTER UPDATE*/
	/* copy inputs */
// 	for (i = 0; i < ORDU; i++) {
// 		uu[i] = *uPtrs[i];
// 	}
	uuH[0] = 0;				/* since ORDU is zero, no input */
	yawRate = *uPtrs[8];	/* yaw rate from yaw rate gyro */
	/* define Ah */
	for (i = 0; i < ORDH*ORDH; i++) {
		Ah[i] = 0;	/* initialize Ah */
	}
	Ah[0*ORDH + 0] = Ah[1*ORDH + 1] = Ah[2*ORDH + 2] = 1;	/* I */
	Ah[0*ORDH + 1] = yawRate * Ts;
	Ah[0*ORDH + 2] = -Ts;
    
	/* transpose Ah */
	sMatTrsp(Ah, AtH, ORDH, ORDH);

	/* start updating after initialized */
	if (initializedFlagH) {
		/* x(k+1) = Ax(k) + Bu(k) */
		sMatMult(Ah, xH, tempX1H, ORDH, ORDH, 1);	/* Ax */
        for (i = 0; i < ORDH; i++) {
            xH[i] = tempX1H[i];
        }
           
//RH 		sMatMult(Bh, uu, tempX2, ORD, 1/*ORDU*/, 1);	/* Bu */
// 		sMatAdd(tempX1, tempX2, x, ORD, 1);		/* Ax(k) + Bu(k)*/
	
	 	xH[0] = AngleMod180(xH[0]); /* heading to the range of [-180,180) */
	
		/* P(k+1) = AP(k)Ah' + Qh */
		sMatMult(Ah, xH+ORDH, tempP1H, ORDH, ORDH, ORDH);		/* AP */
		sMatMult(tempP1H, AtH, tempP2H, ORDH, ORDH, ORDH);	/* APA' */
		sMatAdd(tempP2H, Qh, xH+ORDH, ORDH, ORDH); 			/* APA' + Qh*/
	}
	else {
		for (i = 0; i < NUMOFSTATESH; i++) {
			xH[i] = xH[i];
		}
	}    
    /*ROLL FILTER UPDATE*/
	/* copy inputs */
// 	for (i = 0; i < ORDUR; i++) {
// 		uu[i] = *uPtrs[i];
// 	}
    uuR[0] = *uPtrs[10]; /*Roll rate input*/
    
	/* transpose Ar */
	sMatTrsp(Ar, AtR, ORDR, ORDR);

	/* start updating after initialized */
	if (initializedFlagR) {
		/* x(k+1) = Ax(k) + Bu(k) */
		sMatMult(Ar, xR, tempX1R, ORDR, ORDR, 1);	/* Ax */
		sMatMult(Br, uuR, tempX2R, ORDR, ORDUR, 1);	/* Bu */
		sMatAdd(tempX1R, tempX2R, xR, ORDR, 1);		/* Ax(k) + Bu(k)*/
	
	 	xR[0] = AngleMod180(xR[0]); /* roll angle to the range of [-180,180) */
	
		/* P(k+1) = AP(k)Ar' + Qr */
		sMatMult(Ar, xR+ORDR, tempP1R, ORDR, ORDR, ORDR);		/* AP */
		sMatMult(tempP1R, AtR, tempP2R, ORDR, ORDR, ORDR);	/* APA' */
		sMatAdd(tempP2R, Qr, xR+ORDR, ORDR, ORDR); 			/* APA' + Qr*/
	}
	else {
		for (i = 0; i < NUMOFSTATESR; i++) {
			xR[i] = xR[i];
		}
	}
    
    /*VELOCITY FILTER UPDATE*/
//     yawRateCorr = bufferV_yawRate[buff_counter-1];
    yawRateCorr = (yawRate*x[1] - x[2])*D2R;
        
	/* copy inputs */
	uuV[0] = *uPtrs[12];	/* ax */
	uuV[1] = *uPtrs[13];	/* ay */
	uuV[2] = x[NUMOFSTATESH] * D2R;	/* roll angle (rad) */
//     uuV[2] = bufferV_rollAngle[buff_counter-1];
	uuV[3] = gpsGradeV;
    
	/* define Av */
	for (i = 0; i < ORDV*ORDV; i++) {
		Av[i] = 0;	/* initialize Av */
	}
	Av[0*ORDV + 0] = cos(yawRateCorr*Ts);
	Av[0*ORDV + 1] = yawRateCorr == 0 ? -Ts : -sin(yawRateCorr*Ts) / yawRateCorr;
	Av[0*ORDV + 2] = sin(yawRateCorr*Ts);
	Av[0*ORDV + 3] = yawRateCorr == 0 ? 0 : (-1 + cos(yawRateCorr*Ts)) / yawRateCorr;
	Av[1*ORDV + 1] = 1;
	Av[2*ORDV + 0] = -sin(yawRateCorr*Ts);
	Av[2*ORDV + 1] = yawRateCorr == 0 ? 0 : (1 - cos(yawRateCorr*Ts)) / yawRateCorr;
	Av[2*ORDV + 2] = cos(yawRateCorr*Ts);
	Av[2*ORDV + 3] = yawRateCorr == 0 ? -Ts : -sin(yawRateCorr*Ts) / yawRateCorr;
	Av[3*ORDV + 3] = 1;
	/* define Bv */
	for (i = 0; i < ORDV*ORDUV; i++) {
		Bv[i] = 0;	/* initialize Bv */
	}
	Bv[0*ORDUV + 0] = yawRateCorr == 0 ? Ts : sin(yawRateCorr*Ts) / yawRateCorr;
	Bv[0*ORDUV + 1] = yawRateCorr == 0 ? 0 : (1 - cos(yawRateCorr*Ts)) / yawRateCorr;
	Bv[0*ORDUV + 2] = yawRateCorr == 0 ? 0 : G * (-1 + cos(yawRateCorr*Ts)) / yawRateCorr;
	Bv[0*ORDUV + 3] = yawRateCorr == 0 ? -G * Ts : -G * sin(yawRateCorr*Ts) / yawRateCorr;
	Bv[2*ORDUV + 0] = yawRateCorr == 0 ? 0 : (-1 + cos(yawRateCorr*Ts)) / yawRateCorr;
	Bv[2*ORDUV + 1] = yawRateCorr == 0 ? Ts : sin(yawRateCorr*Ts) / yawRateCorr;
	Bv[2*ORDUV + 2] = yawRateCorr == 0 ? -G * Ts : -G * sin(yawRateCorr*Ts) / yawRateCorr;
	Bv[2*ORDUV + 3] = yawRateCorr == 0 ? 0 : G * (1 - cos(yawRateCorr*Ts)) / yawRateCorr;
	/* transpose Av */
	sMatTrsp(Av, AtV, ORDV, ORDV);

	/* start updating after initialized */
	if (initializedFlagV) {
		/* x(k+1) = Ax(k) + Bu(k) */
		sMatMult(Av, xV, tempX1V, ORDV, ORDV, 1);	/* Ax */
		sMatMult(Bv, uuV, tempX2V, ORDV, ORDUV, 1);	/* Bu */
		sMatAdd(tempX1V, tempX2V, xV, ORDV, 1);		/* Ax(k) + Bu(k)*/

		/* P(k+1) = AP(k)Av' + Qv */
		sMatMult(Av, xV+ORDV, tempP1V, ORDV, ORDV, ORDV);		/* AP */
		sMatMult(tempP1V, AtV, tempP2V, ORDV, ORDV, ORDV);	/* APA' */
		sMatAdd(tempP2V, Qv, xV+ORDV, ORDV, ORDV); 			/* APA' + Qv*/
	}
	else {
		for (i = 0; i < NUMOFSTATESV; i++) {
			xV[i] = xV[i];
		}
	}
    for (i = 0; i < NUMOFSTATESH; i++) {
        x[i] = xH[i];
    }
    for (i = 0; i < NUMOFSTATESR; i++) {
        x[NUMOFSTATESH+i] = xR[i];
    }
    for (i = 0; i < NUMOFSTATESV; i++) {
        x[NUMOFSTATESH+NUMOFSTATESR+i] = xV[i];
    }
}

static void mdlTerminate(SimStruct *S)
{
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
    



