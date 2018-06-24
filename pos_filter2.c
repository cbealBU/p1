/*  File    : pos_filter_realtime.c
 *  Author  : Eric Rossetter
 *  Version : 1.1
 *  Abstract:
 *
        This is a real time Kalman filter to determine the Vx and Vy
        The inputs to this function are:

                                                
                                                U(0)=ax (m/s^2)
                                                U(1)=ay (m/s^2)
                                                U(2)=yaw angle (deg) From heading filter
                                                U(3)=GPS position North (m)
                                                U(4)=GPS position East (m)
                                                U(5)=Status
                                                U(6)=UTC pulse
                                                U(7)=Delay (in #of samples)

  The outputs of this filter are the states, covariances, slipangle at cg, and Vycg

                                                Y(0)=Pn (m)
                                                Y(1)=Vn (m/s)
                                                Y(2)=axbias
                                                Y(3)=Pe (m)
                                                Y(4)=Ve (m/s)
                                                Y(5)=aybias
  
 */

#define S_FUNCTION_NAME pos_filter2
#define S_FUNCTION_LEVEL 2

#include <math.h>
#include <stdio.h>
#include "simstruc.h"

// double conv_to_mod360( double angle);

#define U(element) (*uPtrs[element])  /* Pointer to Input Port0 */

#define ORD     6 /* ORD is the Order of the plant */
#define ORDU    2 /* ORDU is the number of inputs into the plant */
#define ORDY    2 /* ORDY is the number of outputs out of the plant */
#define ORDN    6 /* ORDN is the number of process noise inputs */

#define PI      3.14159 

#define OM_NOSIGNALS 45 //Number of signals to be stored in the buffer
                        

#define OM_BUFLENGTH 1000 // OM_BUFLENGTH is the length of the om_buffer 


#define Ts 0.002 //The sample time
#define INS_NO 3  //Number of INS inputs to be buffered r, ax, ay, psi

#define NO_POS_DATA 0.0 //Input status when OEM4 is in bestvel mode (no position data)

/* The matrix A is time varying and is defined in the mdlDerivatives */

/* Define the B matrix of dimension 4 x 2 */

static real_T B[ORD][ORDU] = { {0.0, 0.0},
                                {1.0, 0.0},
                                {0.0, 0.0},
                                {0.0, 0.0},
                                {0.0, 1.0},
                                {0.0, 0.0}
                             }; 

static real_T Bd[ORD][ORDU]= {  {0.0, 0.0},
                                {1.0*Ts, 0.0},
                                {0.0, 0.0},
                                {0.0, 0.0},
                                {0.0, 1.0*Ts},
                                {0.0, 0.0}
                             }; 

/* Define the C Matrix */

static real_T C[ORDY][ORD] = {  {1, 0, 0, 0, 0, 0},
                                 {0, 0, 0, 1, 0, 0} };

/* Define the process noise matrix Q of dimension 4 x 4 */
  
static real_T Q[ORDN][ORDN] = { {1.0e-6, 0.0, 0.0, 0.0, 0.0, 0.0},
                                {0.0, 1.0e-3, 0.0, 0.0, 0.0, 0.0},
                                {0.0, 0.0, 1.0e-6, 0.0, 0.0, 0.0},
                                {0.0, 0.0, 0.0, 1.0e-6, 0.0, 0.0},
                                {0.0, 0.0, 0.0, 0.0, 1.0e-3, 0.0},
                                {0.0, 0.0, 0.0, 0.0, 0.0, 1.0e-6}
                              };   


//define Q digital so we get BwQdBw=Ts(BwQBw)
static real_T Qd[ORDN][ORDN] = { {1.0e-6*Ts, 0.0, 0.0, 0.0, 0.0, 0.0},
                                {0.0, 1.0e-3*Ts, 0.0, 0.0, 0.0, 0.0},
                                {0.0, 0.0, 1.0e-6*Ts, 0.0, 0.0, 0.0},
                                {0.0, 0.0, 0.0, 1.0e-6*Ts, 0.0, 0.0},
                                {0.0, 0.0, 0.0, 0.0, 1.0e-3*Ts, 0.0},
                                {0.0, 0.0, 0.0, 0.0, 0.0, 1.0e-6*Ts}
                              };  

/*Measurement Noise*/

static real_T R[ORDY][ORDY] = {  {1.0e-4, 0},
                                  {0, 1.0e-4} }; 

/* Define Bw Matrix of dimension 6 x 6 */ 
                                                             
static real_T Bw[ORD][ORDN] = { {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                   {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
                                   {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
                                   {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
                                   {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                                   {0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
                                 };
static real_T VecGammaQGammaTr[ORD*ORD] ;
static real_T xhat[ORD], z[ORDY];

static real_T K[ORD][ORDY];
static real_T om_vec[OM_NOSIGNALS] ; // Temporary vector to store the variables that will go in the om_buffer
static int_T om_bufcount ; // om_bufcount is the position in the buffer the present value is stored
static real_T om_buff[OM_NOSIGNALS*OM_BUFLENGTH + 1] ;  // om_buff is the buffer containing om values.  
// The last value in the buffer is bufcount. Also, note that at t=0, om_buff[0 ... OM_NOSIGNALS -1]
// is set to 0 (the initial value), om_buff[OM_NOSIGNALS*OM_BUFLENGTH] is set to om_bufcount (which is
// initially 0 and incremented by 1). 


/***********Temporary Variables***********/
static real_T P[ORD][ORD], Pplus[ORD][ORD], xplus[ORD], err[ORDY], correction[ORD];
static real_T temp, tempK[ORD][ORDY], tempInv[ORDY][ORDY];
static real_T tempCP[ORDY][ORD], tempCPCt[ORDY][ORDY], tempCPCtR[ORDY][ORDY];
static real_T tempKC[ORD][ORD], tempP[ORD][ORD], tempDet;
static real_T tempE1[ORD], tempE2[ORD], tempE3[ORD*ORD],tempE5[ORD*ORDN];



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
    ssSetNumSFcnParams(S, 2);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

      
    ssSetNumContStates(S, 0); /* States include Kalman Filter states (4) plus Covariance Elements (4x4) */
    ssSetNumDiscStates(S, ORD+ORD*ORD);

    if (!ssSetNumInputPorts(S, 1)) return;
    
    ssSetInputPortWidth(S, 0, 9 );  /* Inputs are ax+ay+heading+Pn+Pe+valid flag+pulse+delay+validFlag */
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, ORD + ORD*ORD);  /* Outputs are States (4) + Covariance (4x4) */

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we have a continuous sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, Ts);
    ssSetOffsetTime(S, 0, 0.0);
}



#define MDL_INITIALIZE_CONDITIONS
/* Function: mdlInitializeConditions ========================================
 * Abstract:
 *    Initialize both continuous states to zero.
 */
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *x0 = ssGetDiscStates(S);
    const real_T initPosE = mxGetPr(ssGetSFcnParam(S,0))[0];
    const real_T initPosN = mxGetPr(ssGetSFcnParam(S,1))[0];
    int_T  i, j, k;
    
    /* Initially set all states to 0 */

    for (i=0;i<(ORD+ORD*ORD);i++) { 
        x0[i] = 0.0;
    }
    x0[0]=initPosN;
    x0[3]=initPosE;
    /* Setting the diagonal elements of the Covarianve matrix to non-zero values */
    for (i=0; i<ORD; i++)
    {
        x0[i+i*ORD+ORD] = 1.0 ;
    } 
   
    for (i = 0; i < ORD; i++) { 
        xhat[i] = 0.0;
    }

    for (i = 0; i < ORD; i++) {
        for (j = 0; j < ORDY; j++) {
            K[i][j] = 0.0 ;
        }
    }
    
 
        // Calculating GammaQGamma'
        // ------------------------
        for (i=0; i<ORD ; i++)
    {
        for (j=0; j<ORDN ; j++)
        {
            tempE5[j+i*ORDN] = 0.0 ;
            for (k=0; k<ORDN ; k++)
            {
                tempE5[j+i*ORDN] += Bw[i][k]*Qd[k][j] ;
                /* In general for any matrix A of dimensions m by n (m rows, n columns) then A[i][j] = vec(A)[n*i + j] */
            }
        }
    }
    
    for (i=0; i<ORD ; i++)
    {
        for (j=0; j<ORD ; j++)
        {
            VecGammaQGammaTr[j+i*ORD] = 0.0 ;
            for (k=0; k<ORDN ; k++)
            {
                VecGammaQGammaTr[j+i*ORD] += tempE5[i*ORDN + k]*Bw[j][k] ;
                /* In general for any matrix A of dimensions m by n (m rows, n columns) then A[i][j] = vec(A)[n*i + j] */
            }
        }
    }

        om_bufcount=0; // Initialize counter to 0
        // Initializing om_buffer to 0
        // ----------------------
        for (i=0; i<(OM_NOSIGNALS*OM_BUFLENGTH); i++)
        {
                om_buff[i] = 0.0 ;
        }
        om_buff[OM_NOSIGNALS*OM_BUFLENGTH] = om_bufcount ;

    
}



/* Function: mdlOutputs =======================================================
 * Abstract:
 *      y = Cx + Du 
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T            *y    = ssGetOutputPortRealSignal(S,0);
    real_T            *x    = ssGetDiscStates(S);
    real_T             time = ssGetT(S) ;
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    int_T i, j, k, delay, om_counter,count, temp1_counter, temp2_counter;  
    real_T  A[ORD][ORD], AT[ORD][ORD], Ad[ORD][ORD], AdP[ORD][ORD], AdPAd[ORD][ORD], PhatMatrix[ORD][ORD], ax, ay, an, ae, heading, gpsPn, gpsPe;
    real_T   gpsUpdate, heading_delayed, ytest[ORD];
    real_T   validFlag, status;


    ax=U(0);
    ay=U(1);
    heading=U(2)*PI/180.0;
    gpsPn=U(3);
    gpsPe=U(4);
    validFlag=U(5);
    gpsUpdate = U(6); /* GPS update pulse*/
    delay = (int_T) (U(7)); /* Delay in number of samples*/
    status = U(8);

//     delay = 1;
    if (status != NO_POS_DATA && delay >= 0) {
        an=ax*cos(heading)-ay*sin(heading); /* Conversion to acceleration in the north and east directions */
        ae=-ax*sin(heading)-ay*cos(heading);

             /* Define the A matrix that depends on yaw rate */
        for (i=0; i<ORD; i++)
        {
            for(j=0; j<ORD; j++)
            {
                A[i][j]=0.0;
             }
         }


         A[0][1]=1.0;
         A[1][2]=-cos(heading);
         A[1][5]=sin(heading);
         A[3][4]=1.0;
         A[4][2]=sin(heading);
         A[4][5]=cos(heading);

          for (i=0; i<ORD; i++)
          {
            for(j=0; j<ORD; j++)
               {
                     AT[i][j]=A[j][i];
               }
          }

       // UNUSED_ARG(tid); /* not used in single tasking mode */


    /* Enter the Inputs into the buffer */
            for (i=0 ; i < OM_NOSIGNALS ; i++)
            {
                    if (i == 0) {
                            om_buff[om_bufcount*OM_NOSIGNALS+i]=an;
                    }
                    else if (i == 1) {
                            om_buff[om_bufcount*OM_NOSIGNALS+i]=ae;
                    }
                    else if (i == 2) {
                            om_buff[om_bufcount*OM_NOSIGNALS+i]=heading;
                    }
                    else
                    {
                            om_buff[om_bufcount*OM_NOSIGNALS+i]=x[i-INS_NO];
                    }
            }



            om_buff[OM_NOSIGNALS*OM_BUFLENGTH] = om_bufcount ;


        for (i = 0; i < ORD; i++) {
            xhat[i] = x[i];  /* states */

        }


        for (i = 0; i < ORD; i++) {
            for (j = 0; j < ORD; j++) {
                P[i][j] = x[i*ORD+j+ORD]; /* covariance matrix */
            }
        }

     /* IF valid GPS signal start of large IF statement */
        if (gpsUpdate && validFlag && (gpsPn*gpsPn<10000000000) && (gpsPe*gpsPe<10000000000)) {

                    /* Gets xhat  and Phat from back in buffer based on delay */    
                    if (delay >= OM_BUFLENGTH)
                    {
                            printf("time = %f, WARNING: Buffer is not of sufficient size\n",time) ;
                    }
                    om_counter = om_bufcount - delay ;
                    if (om_counter < 0)
                    {
                            om_counter = OM_BUFLENGTH + om_counter ;
                    }

                    for (i=0; i<ORD; i++)
                    {
                            xhat[i] = om_buff[om_counter*OM_NOSIGNALS + INS_NO + i] ;
                    }

                    for (i=0; i<ORD; i++)
                    {
                            for (j=0; j<ORD; j++)
                            {
                                    P[i][j] = om_buff[om_counter*OM_NOSIGNALS + INS_NO + ORD + ORD*i + j] ;
                            }
                    }


            z[0] =  gpsPn; // North Position from GPS
            z[1] =  gpsPe; // East Position from GPS

            /* K = PC'*inv[CPC'+R] */
            /* tempK = P*C' */
            for (i = 0; i < ORD; i++) {
                for (j = 0; j < ORDY; j++) {
                    temp = 0;
                    for (k = 0; k < ORD; k++) {
                        temp += P[i][k]*C[j][k];
                    }
                    tempK[i][j] = temp;
                }
            }
            /* tempInv = inv[CPC'+R] */
            /* tempCP = CP */
            for (i = 0; i < ORDY; i++) {
                for (j = 0; j < ORD; j++) {
                    temp = 0;
                    for (k = 0; k < ORD; k++) {
                        temp += C[i][k]*P[k][j];
                    }
                    tempCP[i][j] = temp;
                }
            }
            /* tempCPCt = CP*C' */
            for (i = 0; i < ORDY; i++) {
                for (j = 0; j < ORDY; j++) {
                    temp = 0;
                    for (k = 0; k < ORD; k++) {
                        temp += tempCP[i][k]*C[j][k];
                    }
                    tempCPCt[i][j] = temp;
                }
            }
            /* tempCPCtR = tempCPCt+R */
            for (i = 0; i < ORDY; i++) {
                for (j = 0; j < ORDY; j++) {
                    tempCPCtR[i][j] = tempCPCt[i][j]+R[i][j];
                }
            }
            /* tempInv = inv(tempCPCtR) */
            tempDet = tempCPCtR[0][0]*tempCPCtR[1][1]-tempCPCtR[0][1]*tempCPCtR[1][0];
            tempInv[0][0] = tempCPCtR[1][1] / tempDet;
            tempInv[0][1] = -tempCPCtR[0][1] / tempDet;
            tempInv[1][0] = -tempCPCtR[1][0] / tempDet;
            tempInv[1][1] = tempCPCtR[0][0] / tempDet;
            /* K = tempK*tempInv */
            for (i = 0; i < ORD; i++) {
                for (j = 0; j < ORDY; j++) {
                    temp = 0;
                    for (k = 0; k < ORDY; k++) {
                        temp += tempK[i][k]*tempInv[k][j];
                    }
                    K[i][j] = temp;
                    //printf("K=%f\n",K[i][j]) ;
                }
            }

            /* P = [I-KC]P */
            /* tempKC = -KC */
            for (i = 0; i < ORD; i++) {
                for (j = 0; j < ORD; j++) {
                    temp = 0;
                    for (k = 0; k < ORDY; k++) {
                        temp += K[i][k]*C[k][j];
                    }
                    tempKC[i][j] = -temp;
                }
            }
            /* tempKC = I+tempKC */
            for (i = 0; i < ORD; i++) {
                tempKC[i][i] += 1;
            }
            /* tempP = tempKC*P */
            for (i = 0; i < ORD; i++) {
                for (j = 0; j < ORD; j++) {
                    temp = 0;
                    for (k = 0; k < ORD; k++) {
                        temp += tempKC[i][k]*P[k][j];
                    }
                    tempP[i][j] = temp;
                }
            }
            /* Pplus = tempP */
            for (i = 0; i < ORD; i++) {
                for (j = 0; j < ORD; j++) {
                    Pplus[i][j] = tempP[i][j];
                   // printf("Pplus=%f\n",Pplus[i][j]);
                }
            }


            /* err = z - C*xhat */
            for (i = 0; i < ORDY; i++) {
                temp = 0;
                for (k = 0; k < ORD; k++) {
                    temp += C[i][k]*xhat[k];
                }
                err[i] = z[i] - temp;
            }


            /* correction = K*err */
            for (i = 0; i < ORD; i++) {
                temp = 0;
                for (k = 0; k < ORDY; k++) {
                    temp += K[i][k]*err[k];
                }
                correction[i] = temp;
                // printf("Correction=%f\n",correction[i]);
            }

                    /* y = states */
                    for (i = 0; i < ORD; i++) {
                            xplus[i] = xhat[i] + correction[i];
                             //printf("xplus=%f\n",xplus[i]);
                     }


                    // Updating the new value at time t = t - tdelay 
                    // ---------------------------------------------
                    for (i=0; i<ORD; i++)
                    {
                            om_buff[om_counter*OM_NOSIGNALS + INS_NO + i] = xplus[i] ;

                    }

                    for (i=0; i<ORD; i++)
                    {
                            for (j=0; j<ORD; j++)
                            {
                                    om_buff[om_counter*OM_NOSIGNALS + INS_NO + ORD + ORD*i + j] = Pplus[i][j] ;
                            }
                    }

                    // Updating the entire buffer until the present time
                    // -------------------------------------------------


                    for (count=1; count<(delay+1); count++)
                    {
                            //printf("Do we enter the stupid loop!\n");
                            temp1_counter = om_counter + count ;
                            if (temp1_counter >= OM_BUFLENGTH)
                            {
                                    temp1_counter = temp1_counter - OM_BUFLENGTH ;
                            }               

                            temp2_counter = om_counter + count - 1 ;
                            if (temp2_counter >= OM_BUFLENGTH)
                            {
                                    temp2_counter = temp2_counter - OM_BUFLENGTH ;
                            }

                            heading_delayed=om_buff[temp2_counter*OM_NOSIGNALS+2];
                                 // Define the A matrix that depends on yaw rate 
                             for (i=0; i<ORD; i++)
                            {
                                for(j=0; j<ORD; j++)
                                {
                                    A[i][j]=0.0;
                                }
                            }


                            A[0][1]=1.0;
                            A[1][2]=-cos(heading_delayed);
                            A[1][5]=sin(heading_delayed);
                            A[3][4]=1.0;
                            A[4][2]=sin(heading_delayed);
                            A[4][5]=cos(heading_delayed);

                            for (i=0; i<ORD; i++)
                            {
                                 for(j=0; j<ORD; j++)
                                    {
                                        if (i==j)
                                        {
                                            Ad[i][j]=1+Ts*A[i][j];
                                        }
                                        else
                                        {
                                           Ad[i][j]=Ts*A[i][j];
                                        }
                                    }
                            }




                            // Updating xhat
                            // -------------
                            for (i=0; i<ORD; i++)
                            {
                                    tempE1[i] = 0.0 ;
                                    for (j=0; j<ORD; j++)
                                    {
                                            if (i==j)
                                            {
                                                    tempE1[i] += (1+Ts*A[i][j])*om_buff[temp2_counter*OM_NOSIGNALS + INS_NO + j] ;
                                            }else
                                            {
                                                    tempE1[i] += Ts*A[i][j]*om_buff[temp2_counter*OM_NOSIGNALS + INS_NO + j] ;
                                            }
                                    }
                            }

                            for (i=0; i<ORD; i++)
                            {
                                    tempE2[i] = 0.0 ;
                                    for (j=0; j<ORDU; j++)
                                    {
                                            tempE2[i] += Bd[i][j]*om_buff[temp2_counter*OM_NOSIGNALS + j] ;
                                    }
                            }

                            for (i=0; i< ORD; i++)
                            {
                                    om_buff[temp1_counter*OM_NOSIGNALS + INS_NO + i] = tempE1[i] + tempE2[i] ;
                                     //printf("newstates=%f\n",om_buff[temp1_counter*OM_NOSIGNALS + INS_NO + i]);
                            }


                             // Updating Phat
                            // -------------
                            for (i=0; i<ORD; i++)
                            {
                                for (j=0; j<ORD; j++)
                                {
                                    PhatMatrix[i][j]=om_buff[temp2_counter*OM_NOSIGNALS + INS_NO + ORD + ORD*i + j];
                                }
                            }

                            //Calculate Ad*P
                            for (i = 0; i < ORD; i++)
                            {
                                for (j = 0; j < ORD; j++)
                                {
                                    temp = 0;
                                    for (k = 0; k < ORD; k++)
                                    {
                                        temp += Ad[i][k]*PhatMatrix[k][j];
                                    }
                                    AdP[i][j] = temp;
                                 }
                             }
                            //Calculate AdPAd
                            for (i = 0; i < ORD; i++)
                            {
                                for (j = 0; j < ORD; j++)
                                {
                                    temp = 0;
                                    for (k = 0; k < ORD; k++)
                                    {
                                        temp += AdP[i][k]*Ad[j][k];
                                    }
                                    AdPAd[i][j] = temp;
                                    tempE3[i+j*ORD]=AdPAd[i][j];
                                 }
                             }



                            for (i=0; i< (ORD*ORD); i++)
                            {
                                    om_buff[temp1_counter*OM_NOSIGNALS + INS_NO + ORD + i] = tempE3[i] + VecGammaQGammaTr[i] ;
                                   // printf("Phatupdate=%f, Part1=%f, Part2=%f\n",om_buff[temp1_counter*OM_NOSIGNALS + INS_NO + ORD + i], tempE3[i], VecGammaQGammaTr[i] );
                            }

                    } 

            // y=x, K
                    // ------
                    for (i=0; i<(ORD + ORD*ORD); i++)
                    {
                            y[i] = om_buff[om_bufcount*OM_NOSIGNALS + INS_NO + i] ; 
                           // printf("y2=%f\n", y[i]);

                    }


                    for (i=0; i<(ORD + ORD*ORD); i++)
                    {
                            x[i] = y[i] ;

                    }
                    //x[2]=-0.6;
                   // x[5]=0.13;


            }

            else /*************Else for Giant IF*******************/
            {


                    /* y=x */
                    for (i=0; i<(ORD + ORD*ORD); i++)
                    {
                            y[i] = x[i] ;
                    }


            } /*<--------------------------END OF GIANT IF  ***********/



            // Updating om_bufcount
            // --------------------
            om_bufcount = om_bufcount + 1 ;

            /* If we are at the end of the buffer start at the beginning */
            if (om_bufcount >= OM_BUFLENGTH)
            {
                    om_bufcount = 0 ;
            } 
   } 
   else {
        for (i = 0; i < ORD + ORD*ORD; i++){
            y[i] = 0.0;
        }
    }
     
}



#define MDL_UPDATE
/* Function: mdlDerivatives =================================================
 * Abstract:
 *      xdot = Ax + Bu
 */
static void mdlUpdate(SimStruct *S, int_T tid)
{
//     real_T            *dx   = ssGetdX(S);
    real_T            *x    = ssGetDiscStates(S);
    real_T              time = ssGetT(S) ;
    real_T            temp1[ORD], temp2[ORD] ;
    real_T            temp3[ORD*ORD], temp4[ORD*ORD] ;
    real_T            temp5[ORD*ORDN], temp6[ORD*ORD], temp7[ORD*ORD], temp8 ;
    real_T            Phat[ORD][ORD], AdPhat[ORD][ORD], AdPhatAd[ORD][ORD];
 
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    int_T i, j, k ;  
    real_T A[ORD][ORD], Ad[ORD][ORD], ax, ay, an, ae, heading, status;
    
    
    ax=U(0);
    ay=U(1);
    heading=U(2)*PI/180.0;
    status = U(8);
    
    if (status != NO_POS_DATA) {

        an=ax*cos(heading)-ay*sin(heading); /* Conversion to acceleration in the north and east directions */
        ae=-ax*sin(heading)-ay*cos(heading);

             /* Define the A matrix that depends on yaw rate */
        for (i=0; i<ORD; i++)
        {
            for(j=0; j<ORD; j++)
            {
                A[i][j]=0.0;
             }
         }


         A[0][1]=1.0;
         A[1][2]=-cos(heading);
         A[1][5]=sin(heading);
         A[3][4]=1.0;
         A[4][2]=sin(heading);
         A[4][5]=cos(heading);

         for (i=0; i<ORD; i++)
        {
             for(j=0; j<ORD; j++)
                {
                    if (i==j)
                    {
                        Ad[i][j]=1+Ts*A[i][j];
                    }
                    else
                    {
                       Ad[i][j]=Ts*A[i][j];
                    }
                }
        }


        /* xdot=Ax+Bu */
        for (i=0; i<ORD; i++)
        {
            temp1[i] = 0.0 ;
            for (j=0; j<ORD; j++)
            {
                temp1[i] += Ad[i][j]*x[j] ;
            }
        }

        for (i=0; i<ORD; i++)
        {
            temp2[i] = 0.0 ;
            for (j=0; j<ORDU; j++)
            {
                if(j == 0)
                {    
                temp2[i] += Bd[i][j]*an ;
                }
                else if (j == 1)
                {
                temp2[i] += Bd[i][j]*ae ;
                }
            }
        }

        for (i=0; i<ORD; i++)
        {
            x[i] = temp1[i] + temp2[i] ;
        }

        /* Pdot = AdPAd' + GammaQdGamma' */
        for (i = 0; i < ORD; i++)
        {
             for (j = 0; j < ORD; j++)
             {
                 Phat[i][j] = x[ORD + ORD*i + j];
             }
        }

        for (i=0; i<ORD ; i++)
        {
            for (j=0; j<ORD ; j++)
            {
                temp8 = 0.0 ;
                for (k=0; k<ORD ; k++)
                {
                     temp8 += Ad[i][k]*Phat[k][j]; 

                }
                AdPhat[i][j] = temp8; 
            }
        }

    //     for (i=0; i<ORD ; i++)
    //     {
    //         for (j=0; j<ORD ; j++)
    //         {
    //             temp3[j+i*ORD] = 0.0 ;
    //             for (k=0; k<ORD ; k++)
    //             {
    //                  AdPhat[j+i*ORD] += Ad[i][k]*Phat[k][j] 
    //                 
    //             }
    //         }
    //     } 

        for (i=0; i<ORD ; i++)
        {
            for (j=0; j<ORD ; j++)
            {
                temp8 = 0.0;
                for (k=0; k<ORD ; k++)
                {
                     temp8 += AdPhat[i][k]*Ad[j][k]; 

                }
                AdPhatAd[i][j] = temp8; 
                temp3[j + i*ORD] = AdPhatAd[i][j];
            }
        } 

        for (i=0; i<ORD ; i++)
        {
            for (j=0; j<ORDN ; j++)
            {
                temp5[j+i*ORDN] = 0.0 ;
                for (k=0; k<ORDN ; k++)
                {
                    temp5[j+i*ORDN] += Bw[i][k]*Qd[k][j] ;
                    /* In general for any matrix A of dimensions m by n (m rows, n columns) then A[i][j] = vec(A)[m*i + j] */
                }
            }
        }

        for (i=0; i<ORD ; i++)
        {
            for (j=0; j<ORD ; j++)
            {
                temp6[j+i*ORD] = 0.0 ;
                for (k=0; k<ORDN ; k++)
                {
                    temp6[j+i*ORD] += temp5[i*ORDN + k]*Bw[j][k] ;
                    /* In general for any matrix A of dimensions m by n (m rows, n columns) then A[i][j] = vec(A)[m*i + j] */
                }
            }
        }

        for (i=0; i<ORD ; i++)
        {
            for (j=0; j<ORD ; j++)
            {
                x[ORD + j+i*ORD] = temp3[j+i*ORD] + temp6[j+i*ORD] ;
            }
        }      
    }
    
    else {
        for (i = 0; i < ORD + ORD*ORD; i++) {
            x[i] = 0.0;
        }
    
    }
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */

// double conv_to_mod360( double angle) 
// {
// // Conversion of angle to the range of [-180,180] 
// // ----------------------------------------------
//         real_T Mfrac, Mint, angle_mod360 ;
// 
//         Mfrac = modf(angle/360.0,&Mint) ;
//         
//         if (fabs(Mfrac) == 0.0)
//         {
//                 if (angle == 0.0)
//                 {
//                         Mfrac = 0.0 ;
//                 }
//                 else if (angle > 0.0)
//                 {
//                         Mfrac = 1.0 ;
//                 }
//                 else 
//                 {
//                         Mfrac = -1.0 ;
//                 }
//         }
// 
//         if (Mfrac > 0.5)
//         {
//                 Mfrac = Mfrac - 1.0 ;
//         }
//         else if (Mfrac < -0.5)
//         {
//                 Mfrac = Mfrac + 1.0 ;
//         }
// 
//         angle_mod360 = Mfrac*360.0 ; 
//         return (angle_mod360) ;
// }

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
  //  UNUSED_ARG(S); /* unused input argument */
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
