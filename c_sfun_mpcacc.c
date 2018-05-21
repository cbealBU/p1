#define S_FUNCTION_NAME  c_sfun_mpcacc
#define S_FUNCTION_LEVEL 2

#define DEBUG
#include "H_mpc_sfun_opt.h"
#include "H_mpc_params.h"

/*Inputs and States of controlled object */
real_T uLast[uNum];
real_T vf;
real_T Xk[xNum];
real_T Xprev[xNum];
/*External Disturbances */
real_T vDst[vNum]; 
real_T Vdst[(P+1)*vNum];
real_T Vplus[P*vNum];
/*Cost function and constraints of QP problem*/
real_T Ymax[Omega*xNum];
real_T Ymin[Omega*xNum];
real_T fopt[L+1];                 // optimization matrix of QP
real_T bcstr[maxbNum];    
real_T ZEopt[L+1];             // Optimized res
/* Tableau, Tableau etc for QP solving */
long int numc, nuc, iret;    // Used for dantaig algorithm
real_T tab[(bcstrNum+L+1)*(bcstrNum+L+1)]; 
real_T basisi[maxbNum+L+1];
real_T rhsc[maxbNum]; 
real_T rhsa[L+1];                                       
long int ibi[bcstrNum+L+1];
long int ili[bcstrNum+L+1]; 
/*Temp variables for matrix computing*/
real_T slrVar;
real_T vecVar[vecDim];
real_T matVar[matDim];
real_T tempMat[matDim];                           // for Matrix computing
/*Temp variables for QP computing */
real_T term1[Omega*xNum];
real_T term2[Omega*xNum];
real_T term3[Omega*xNum];
real_T term4[Omega*xNum];
real_T term5[Omega*xNum];
/*Display mpc control percentage */
long int ctrlPercent, lastCtrlPercent, m; 

static void mdlInitializeSizes (SimStruct *S)   /*Initialise the sizes array */
{
    // define variable name of block parameters
    // define system states     
    // define internal varaibles
    /// Check of block parameters number
    ssSetNumSFcnParams(S, NPARAMS); 
    /// Number of system states
	/* No continuous states */
	ssSetNumContStates(S, 0);
	/* Discrete states when using RT, work vectors will do */
    ssSetNumDiscStates(S, uNum+xNum); /* register lastx and lastu as states */    
	/// Number of input ports
	if (!ssSetNumInputPorts(S, INPUTNUM)) return;
    /// Dimensions of input ports
	ssSetInputPortWidth(S,0,vNum);  
    ssSetInputPortWidth(S,1,xNum);  //Setpoint of U
    ssSetInputPortWidth(S,2,uNum);
    ssSetInputPortWidth(S,3,1);         //Vehicle speed(m/s)
    /// Number of output ports
	if (!ssSetNumOutputPorts(S,OUTPUTNUM))  return; 
    ///Dimensions of output ports
    ssSetOutputPortVectorDimension(S, 0, uNum);
    ssSetOutputPortVectorDimension(S, 1, 1); // epsilon    
    /// Direct Feedthrough setting
	if (ctrlMode>0 ) //closed loop control
    {
        ssSetInputPortDirectFeedThrough(S,0,1); 
        ssSetInputPortDirectFeedThrough(S,1,1); 
        ssSetInputPortDirectFeedThrough(S,2,1); 
        ssSetInputPortDirectFeedThrough(S,3,1); 
    } 
    else                   // open loop control
    {
        ssSetInputPortDirectFeedThrough(S,0,0); 
        ssSetInputPortDirectFeedThrough(S,1,0); 
        ssSetInputPortDirectFeedThrough(S,2,0); 
        ssSetInputPortDirectFeedThrough(S,3,0);   
	}     
	/// Number of sampling time - One sample time 
    ssSetNumSampleTimes(S, 1);    
    /// Number of work dynamic variables
	ssSetNumPWork(S,NPWORK);
	ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE); 
    #ifdef DEBUG
    dispSTR("------------ C Mex S-function of MPC MTCACC(Opt Ver.) ------------");
    dispSTR("mdlInitializeSizes( )");
    #endif
}

/*
static void mdlInitializeSampleTimes(SimStruct *S)
{
  real_T ts           = (real_T)*mxGetPr(p_Ts(S));  
  
  if (( ctrlMode > 0) & (ts > 0))  /*AB: ts was set as the sampling time only if ts>0 
	    ssSetSampleTime(S, 0, ts);
  else
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    #ifdef DEBUG
    dispSTR("mdlInitializeSampleTimes( )");
    #endif
}
*/
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);   /* This usually works, do not modify */
    ssSetOffsetTime(S, 0, 0.0);
}


#define MDL_INITIALIZE_CONDITIONS
#if defined(MDL_INITIALIZE_CONDITIONS)
static void mdlInitializeConditions(SimStruct *S)
{
    int i;
    real_T *discState;    
    
    discState = ssGetRealDiscStates(S);
    for (i=0; i<xNum; i++)   // Last system states 
    {
        discState[i] = 0.0; 
    }
    for (i=xNum; i<xNum+uNum; i++)   // Last control input 
    {
        discState[i] = 0.0; 
    }
    discState[xNum+uNum] = 0.0; // cout of mpc cycle
    lastCtrlPercent=(long int)0;
    m=(long int)1;
    #ifdef DEBUG
    dispSTR("mdlInitializeConditions( )");
    #endif
}
#endif /* MDL_INITIALIZE_CONDITIONS */

static void mdlOutputs(SimStruct *S, int_T tid) 
{
    int i, j, k;
    // Block inputs
    InputRealPtrsType       mdPtrs = ssGetInputPortRealSignalPtrs(S,0); 
    InputRealPtrsType       xPtrs = ssGetInputPortRealSignalPtrs(S,1); 
    InputRealPtrsType       uLastPtrs = ssGetInputPortRealSignalPtrs(S,2); 
    InputRealPtrsType       vfPtrs = ssGetInputPortRealSignalPtrs(S,3); 
    // define variable name of block parameters
    real_T Ts             = (int_T)*mxGetPr(p_Ts(S)); 
    real_T *A             = mxGetPr(p_A(S));
    real_T *B             = mxGetPr(p_B(S));
    real_T *G             = mxGetPr(p_G(S));
    real_T *H             = mxGetPr(p_H(S));    
    real_T *Mctrl        = mxGetPr(p_Mctrl(S));
    real_T *T_u1           = mxGetPr(p_T_u1(S));
    real_T *MT            = mxGetPr(p_MT(S));
    real_T *Umax         = mxGetPr(p_Umax(S));
    real_T *Umin         = mxGetPr(p_Umin(S));
    real_T *dUmax      = mxGetPr(p_dUmax(S));
    real_T *dUmin       = mxGetPr(p_dUmin(S));
    real_T *Ymax_P   = mxGetPr(p_Ymax_P(S));
    real_T *Ymin_P    = mxGetPr(p_Ymin_P(S));
    real_T *Dsafe      = mxGetPr(p_Dsafe(S));
    real_T *Tsafe      = mxGetPr(p_Tsafe(S));
    real_T *Xmin       = mxGetPr(p_Xmin(S));
    real_T *aH           = mxGetPr(p_aH(S));
    real_T *Hinv        = mxGetPr(p_Hinv(S));
    real_T *TAB        = mxGetPr(p_TAB(S));
    int_T maxIter       = (int_T)*mxGetPr(p_maxIter(S));
    real_T *A_cstrXmin     = mxGetPr(p_A_cstrXmin(S));
    real_T *AHaH               =  mxGetPr(p_AHaH(S));
    real_T *AHS_uS_x      = mxGetPr(p_AHS_uS_x(S));
    real_T *AHS_uSu1      =  mxGetPr(p_AHS_uSu1(S));
    real_T *AHS_uHv        =  mxGetPr(p_AHS_uHv(S));
    real_T *AHS_uSeH     = mxGetPr(p_AHS_uSeH(S));
    real_T *AHTduTu1      =  mxGetPr(p_AHTduTu1(S));
    real_T *MYS_x            = mxGetPr(p_MYS_x(S));
    real_T *MYSu1            = mxGetPr(p_MYSu1(S));
    real_T *MYHv              = mxGetPr(p_MYHv(S));
    real_T *MYSeH           = mxGetPr(p_MYSeH(S));
    real_T *AsS_x             = mxGetPr(p_AsS_x(S));
    real_T *AsSu1             = mxGetPr(p_AsSu1(S));
    real_T *AsHv               = mxGetPr(p_AsHv(S));
    real_T *AsSeH            = mxGetPr(p_AsSeH(S));  
    real_T *AyS_x            = mxGetPr(p_AyS_x(S));
    real_T *AySu1            = mxGetPr(p_AySu1(S));
    real_T *AyHv               = mxGetPr(p_AyHv(S));
    real_T *AySeH            = mxGetPr(p_AySeH(S));
    real_T *AuTu1            = mxGetPr(p_AuTu1(S));
    real_T *S_uS_x            = mxGetPr(p_S_uS_x(S));
    real_T *S_uSu1            = mxGetPr(p_S_uSu1(S));
    real_T *S_uHv               = mxGetPr(p_S_uHv(S));
    real_T *S_uSeH            = mxGetPr(p_S_uSeH(S));
    real_T *TduTu1            = mxGetPr(p_TduTu1(S));
    // define system states    
    real_T *uCtrl = ssGetOutputPortRealSignal(S,0);
    real_T *epsilon = ssGetOutputPortRealSignal(S,1);
    // define internal varaibles
    real_T *discState; 
    //**************************************************************//  
    // Get input signals from input ports
    vDst[0]    =*mdPtrs[0]; 
    uLast[0]  =*uLastPtrs[0]; 
    vf            =*vfPtrs[0];
    for(i=0; i<xNum; i++)     Xk[i]=*xPtrs[i]; 
    discState = ssGetRealDiscStates(S);
    for(i=0; i<xNum; i++)     Xprev[i]= discState[i]; 
    numc = (bcstrNum+L+1);
    nuc   = 0;    
    #ifdef DEBUG
//     dispVEC(uLast, uNum, "uLast");
//     dispVEC(Xk, xNum, "Xk");
//     dispVEC(Xprev, xNum, "Xprev");
    #endif
    //**************************************************************//  
    // Modify constraint of system output Y 
     for(i=0; i<Omega*xNum; i++)
    {
        Ymax[i] = Ymax_P[i] ;
        Ymin[i]  = Ymin_P[i] ;
    }
    #ifdef DEBUG
//     dispVEC(Ymax, Omega*xNum, "Ymax");
    #endif
    //**************************************************************//  
    // Predictive external disturbance- Zero Order Hold   
    for (i=0; i<(P+1); i++)
    {
        Vdst[i]= vDst[0];
        if (i<=P) Vplus[i]= vDst[0];
    }
    #ifdef DEBUG
    //dispVEC(Vdst, (P+1)*uNum, "Ymin");
    #endif
    //**************************************************************//  
    // Coefficient of Linear Term of QP problem -- fopt
    MVP(S_uS_x, Xk, L, xNum);
    memcpy(matVar, vecVar, L*sizeof(real_T));    
    MVP(S_uSu1, uLast, L, uNum);    
    MMA(matVar, vecVar, L, 1);    
    MVP(S_uHv, Vdst, L, (P+1)*vNum);
    MMA(matVar, vecVar, L, 1);    
    MVP(S_uSeH, Xprev, L, xNum);
    MMS(matVar, vecVar, L, 1);    
    MVP(TduTu1, uLast, L, uNum);
    MMA(matVar, vecVar, L, 1); 
    memcpy(fopt, matVar, L*sizeof(real_T));
    fopt[L]=0.0;
    #ifdef DEBUG
    //dispVEC(Xnew, P*xNum, "Xnew");
//     dispVEC(fopt, (L+1),  "fopt");
    #endif
    
    //**************************************************************//  
    // Coefficient b in linear constraints Ax<b
    // The 1st term
    MMP(T_u1, uLast, Omega*uNum, uNum, 1); 
    MMS(Umax, matVar, Omega*uNum, 1);
    memcpy(bcstr, matVar, Omega*uNum*sizeof(real_T));     
    // The 2nd term    
    MMP(vecVar, uLast, Omega*uNum, uNum, 1); 
    MMS(matVar, Umin, Omega*uNum, 1);
    memcpy(bcstr+Omega*uNum, matVar, Omega*uNum*sizeof(real_T));    
    // The 3nd term
    memcpy(bcstr+2*Omega*uNum, dUmax, Omega*uNum*sizeof(real_T));    
    // The 4nd term
    for(i=0; i<Omega*uNum; i++) vecVar[i]=(-1)*dUmin[i]; 
    memcpy(bcstr+3*Omega*uNum, vecVar, Omega*uNum*sizeof(real_T));    
    // The 5st term    
    MVP(MYS_x, Xk, Omega*xNum, xNum);
    memcpy(term1, vecVar, Omega*xNum*sizeof(real_T)); 
    MVP(MYSu1, uLast, Omega*xNum, uNum);
    memcpy(term2, vecVar, Omega*xNum*sizeof(real_T));
    MVP(MYHv, Vdst, Omega*xNum, (P+1)*vNum);
    memcpy(term3, vecVar, Omega*xNum*sizeof(real_T)); 
    MVP(MYSeH, Xprev, Omega*xNum, xNum);
    memcpy(term4, vecVar, Omega*xNum*sizeof(real_T));
    for(i=0; i<(Omega*xNum); i++)
    {
        vecVar[i]=Ymax[i]-term1[i]-term2[i]-term3[i]+term4[i];
    }
    memcpy(bcstr+4*Omega*uNum, vecVar, Omega*xNum*sizeof(real_T));
     // The 6st term
    for(i=0; i<(Omega*xNum); i++)
    {
        vecVar[i]=term1[i]+term2[i]+term3[i]-term4[i]-Ymin[i];
    }
    memcpy(bcstr+4*Omega*uNum+Omega*xNum, vecVar, Omega*xNum*sizeof(real_T));     
    // The 7st term
    MVP(AsS_x, Xk, Omega*xNum, xNum);
    memcpy(term1, vecVar, Omega*xNum*sizeof(real_T));
    MVP(AsSu1, uLast, Omega*xNum, uNum);
    memcpy(term2, vecVar, Omega*xNum*sizeof(real_T));
    MVP(AsHv, Vdst, Omega*xNum, (P+1));
    memcpy(term3, vecVar, Omega*xNum*sizeof(real_T));
    MVP(AsSeH, Xprev, Omega*xNum, xNum);
    memcpy(term4, vecVar, Omega*xNum*sizeof(real_T));
    for(i=0; i<(Omega*xNum); i++)
    {
        vecVar[i]=term1[i]+term2[i]+term3[i]-term4[i];
    }
    memcpy(term1, vecVar, Omega*xNum*sizeof(real_T));
    MVP(Tsafe, Vplus, Omega*xNum, P*vNum);
    memcpy(term2, vecVar, Omega*xNum*sizeof(real_T)); 
    MMS(term1, term2, Omega*xNum, 1);
    MMS(matVar, Dsafe, Omega*xNum,1);    
    memcpy(bcstr+4*Omega*uNum+2*Omega*xNum, matVar, SAFEpoint*xNum*sizeof(real_T)); 
    // The 8st term
    MVP(AyS_x, Xk, xNum, xNum);
    memcpy(term1, vecVar, xNum*sizeof(real_T));
    MVP(AySu1, uLast, xNum, uNum);
    memcpy(term2, vecVar, xNum*sizeof(real_T));
    MVP(AyHv, Vdst, xNum, (P+1));
    memcpy(term3, vecVar, xNum*sizeof(real_T));
    MVP(AySeH, Xprev, xNum, xNum);
    memcpy(term4, vecVar,xNum*sizeof(real_T));
    for(i=0; i<xNum; i++)
    {
        vecVar[i]=(-1)*term1[i]-term2[i]-term3[i]+term4[i];
    } 
    memcpy(bcstr+4*Omega*uNum+2*Omega*xNum+SAFEpoint*xNum, vecVar, xNum*sizeof(real_T)); 
    // the 9th term
    MMP(AuTu1, uLast, uNum, uNum, 1);
    for(i=0; i<xNum; i++) matVar[i]=(-1)*matVar[i];
    memcpy(bcstr+4*Omega*uNum+(2*Omega+1)*xNum+SAFEpoint*xNum, matVar, uNum*sizeof(real_T)); 
    #ifdef DEBUG
//     dispSLR(bcstrNum, "bNum");
//     dispVEC(bcstr, bcstrNum,  "bcstr");
    #endif
    //************************************************************//    
    // Pre processing for Dantzig wolfe algorithm
    // rhsc
    for (i=0; i<bcstrNum; i++) 
    { 
        rhsc[i]= bcstr[i] - A_cstrXmin[i] ; 
    }
    // rhsa
    for (i=0; i<(L+1); i++) rhsa[i]=aH[i]-fopt[i];
    // basisi
    MVP(Hinv, rhsa, (L+1), (L+1));
    memcpy(basisi, vecVar, (L+1)*sizeof(real_T));    
    MVP(AHS_uS_x, Xk, bcstrNum, xNum);
    memcpy(matVar, vecVar, bcstrNum*sizeof(real_T));    
    MVP(AHS_uSu1, uLast, bcstrNum, uNum);    
    MMA(matVar, vecVar, bcstrNum, 1); 
    MVP(AHS_uHv, Vdst, bcstrNum, (P+1)*vNum);
    MMA(matVar, vecVar, bcstrNum, 1);
    MVP(AHS_uSeH, Xprev, bcstrNum, xNum);
    MMS(matVar, vecVar, bcstrNum, 1);    
    MVP(AHTduTu1, uLast, bcstrNum, uNum);
    MMA(matVar, vecVar, bcstrNum, 1);    
    MMS(matVar, AHaH, bcstrNum, 1);    
    for (i=0; i<bcstrNum; i++) basisi[L+1+i]=rhsc[i]+matVar[i];
    // ibi and ili
    for(i=0; i<numc; i++) 
    {
        ili[i]=i+1;
        ibi[i]=-ili[i];
      } 
    // TAB
    memcpy(tab, TAB, numc*numc*sizeof(real_T));
    #ifdef DEBUG
//     dispSLR(numc,  "numc");
//     dispVEC(rhsc, bcstrNum, "rhsc");
//     dispVEC(rhsa, (L+1), "rhsa");
//     dispMAT(tab, numc, numc, "tab");    
//     dispVEC(basisi, numc,  "basisi");
//     dispVEC(ibi,  numc,  "ibi");
    #endif
    //************************************************************//    
    // Dantzig wolfe algorithm    
    iret=dantzg(tab, &numc, &numc, &nuc, basisi, ibi, ili, &maxIter);
    //************************************************************//   
    // After processing of Dantzig wolfe algorithm    
    if (iret > 0)
	{        
        for (i=0; i<(L+1); i++) 
        {
            if (ili[i] <= 0) 
                { ZEopt[i]=Xmin[i]; }
            else 
                  { ZEopt[i]=basisi[ili[i]-1]+Xmin[i]; }
        } // for i
	} 
    else
    {
        ZEopt[0] = 0;
        ZEopt[L] =  -1;
        if(Xk[0]>0)
        {
            ZEopt[0] = dUmax[0];
            ZEopt[L] =  -1;
        }
        if(Xk[0]<0)
        {
            ZEopt[0] = dUmin[0];
            ZEopt[L] =  -1;
        }        
    }
    
    MMP(MT, ZEopt, P*uNum, L, 1);       // matVar --- deltUopt
    MMP(matVar, Mctrl, 1, P*uNum, 1);              // matVar --- deltUopt(1) 
    uCtrl[0] = uLast[0] + matVar[0];
    epsilon[0] = ZEopt[L];                         // epsilon
    #ifdef DEBUG
//     dispVEC(uCtrl, uNum, "uCtrl");
//     dispVEC(ZEopt, (L+1), "ZEopt");
//     dispVEC((&maxIter), 1, "maxIter");
    //dispVEC(epsilon, 1, "epsilon");
    #endif
    /********************************************************/
    // System states prediction based SS model
    #ifdef DEBUG
//     dispVEC(Xprev, xNum, "Xprev");
//     dispVEC(uLast, uNum, "uLast");
    #endif
    // ek=xk-xp;
    MMS(Xk, Xprev, xNum, 1);    // ek    
    // xp=A*xk+B*uCtrl+G*vDst+H*ek;
    MMP(matVar, H, 1, xNum, xNum); // term 4
    memcpy(term4, matVar, xNum*sizeof(real_T));
    MMP(G, vDst, xNum, vNum, 1); // term 3
    memcpy(term3, matVar, xNum*sizeof(real_T));
    MMP(B, uCtrl, xNum, uNum, 1); // term 2
    memcpy(term2, matVar, xNum*sizeof(real_T));
    MMP(A, Xk, xNum, xNum, 1); // term 1
    memcpy(term1, matVar, xNum*sizeof(real_T));
    MMA(term1, term2, xNum, 1);
    MMA(matVar, term3, xNum, 1);
    MMA(matVar, term4, xNum, 1);
    memcpy(Xprev, matVar, xNum*sizeof(real_T));
    for (i = 0; i<xNum; i++)
    {
        discState[i] = Xprev[i];
    }
    #ifdef DEBUG
//     dispVEC(Xprev, xNum, "Xprev");
    #endif
    
    /*Comput MPC control time and disp control precent*/
    ctrlPercent = (long int)(m*100/simCycNum);
    if(ctrlPercent > lastCtrlPercent)
    {
        dispCtrlPercent(ctrlPercent);
        lastCtrlPercent = (long int)ctrlPercent;
    }
    m++;    
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{  
}

static void mdlTerminate(SimStruct *S)
{   
  int i;
  /* Free all work vectors */
  for (i = 0; i<ssGetNumPWork(S); i++) {
    if (ssGetPWorkValue(S,i) != NULL) {
      free(ssGetPWorkValue(S,i));
    }
  }
}



#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
