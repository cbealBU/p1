/*
 * File : wgsxyz2enu.c
 *
 *	[east, north, up]=wgsxyz2enuV(x,y,z,refx,refy,refz)
 *
 * Abstract:
 *	Transforms x y z representing velocity in WGS84 xyz coordinates
 * into its corresponding east north and up velocities,
 * 
 *
 *
 *
 */


#define S_FUNCTION_NAME  wgsxyz2enuV
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>
/*================*
 * Build checking *
 *================*/


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 6);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S,1)) return;
    ssSetOutputPortWidth(S, 0, 3);

    ssSetNumSampleTimes(S, 1);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}


/* Function: mdlOutputs =======================================================
 * Abstract:
 *    
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
double A_EARTH, flattening, NAV_E2,rad2deg,wlon,rhosqrd,rho,templat,tempalt,rhoerror,zerror,wlat,walt;
double slat,clat,q,r_n,drdl,aa,bb,cc,dd,invdet,ang1,ang2,xrel,yrel,zrel;
double x,y,z,xref,yref,zref,E,N,U;
//3 outputs are lat,lon, h
	real_T *Y = ssGetOutputPortRealSignal(S,0);
	
//3 inputs are x,y,z in WGS 84
    InputRealPtrsType in = ssGetInputPortRealSignalPtrs(S,0);
	x		=*in[0];
	y		=*in[1];
	z		=*in[2];
	xref	=*in[3];
	yref	=*in[4];
	zref	=*in[5];



    //This dual-variable iteration seems to be 7 or 8 times faster than
	//a one-variable (in latitude only) iteration.  AKB 7/17/95 
	
	A_EARTH = 6378137;
	flattening = 1/298.257223563;
	NAV_E2 = (2-flattening)*flattening; // also e^2
    rad2deg = 180/3.14159;

	if ((xref == 0.0) & (yref == 0.0))
	{
		wlon = 0.0;
	}
	else
	{
		wlon = atan2(yref, xref)*rad2deg;
     }

//Make initial lat and alt guesses based on spherical earth.
	rhosqrd = xref*xref + yref*yref;
	rho = sqrt(rhosqrd);
	templat = atan2(zref, rho);
	tempalt = sqrt(rhosqrd + zref*zref) - A_EARTH;
	rhoerror = 1000.0;
	zerror   = 1000.0;

//Newton's method iteration on templat and tempalt makes
//	the residuals on rho and z progressively smaller.  Loop
//	is implemented as a 'while' instead of a 'do' to simplify
//	porting to MATLAB

	while ((abs(rhoerror) > 1e-6) | (abs(zerror) > 1e-6)) 
	{
		slat = sin(templat);
		clat = cos(templat);
		q = 1 - NAV_E2*slat*slat;
		r_n = A_EARTH/sqrt(q);
		drdl = r_n*NAV_E2*slat*clat/q; //d(r_n)/d(latitutde) 

		rhoerror = (r_n + tempalt)*clat - rho;
		zerror   = (r_n*(1 - NAV_E2) + tempalt)*slat - zref;

			//			  --                               -- --      --
			//			  |  drhoerror/dlat  drhoerror/dalt | |  a  b  |
            //            % Find Jacobian           |		       		    |=|        |
			//			  |   dzerror/dlat    dzerror/dalt  | |  c  d  |
			//			  --                               -- --      -- 

		aa = drdl*clat - (r_n + tempalt)*slat;
		bb = clat;
		cc = (1 - NAV_E2)*(drdl*slat + r_n*clat);
		dd = slat;

		//Apply correction = inv(Jacobian)*errorvector

		invdet = 1.0/(aa*dd - bb*cc);
		templat = templat - invdet*(+dd*rhoerror -bb*zerror);
		tempalt = tempalt - invdet*(-cc*rhoerror +aa*zerror);
		}

		wlat = templat*rad2deg;
		walt = tempalt;

		
		//R1=rot(90+wlon, 3);
		//R2=rot(90-wlat, 1);
		//R=R2*R1; 
		//xyz2enu = R;

//		N = length(xyz);
		//ENU = zeros(N,3);
		//ENU(1,:) = [xyz2enu * (xyz(1,:) - refxyz)']';
		ang1=3.14159/2+wlon*3.14159/180;
		ang2=3.14159/2-wlat*3.14159/180;
		xrel=x;
		yrel=y;
		zrel=z;
		E=xrel*cos(ang1)+yrel*sin(ang1);
		N=-xrel*sin(ang1)*cos(ang2)+yrel*cos(ang1)*cos(ang2)+zrel*sin(ang2);
		U=xrel*sin(ang1)*sin(ang2)-yrel*cos(ang1)*sin(ang2)+zrel*cos(ang2);

		
	Y[0]=E;        //Output to Y
	Y[1]=N;            //Output to Y
	Y[2]=U;             //Output to Y
}






/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
}


#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
