/*	10/13/06 JPS
 * File : lat_err_yaw_switch.c
 *THIS IS THE VERSION WITH THE SWITCH
 *	[error, seg, u, angle]=lat_err_yaw(segment, x, y, heading angle)
 *
 * Abstract:
 *       This S function calculates lateral position and heading
 *		error based on current GPS position and velocity data.
 *      It has the following basic outline:
 *			1.  Take-in (or type in) map coefficients.  Map is 
 *			polynomial segments found using mapfunclosed or similar
 *			2.For current and next segment of the map, find error 
 *			using projection theorem.
 *				a.  For current and next segment, solve polynomial 
 *				using mueller's method
 *				b.Compare resultant error, take smaller one.
 *				c.  Find sign of error by cross product
 *			3. Find angle of current spot on road, compare to
 *			GPS heading angle to get heading error
 *
 *
 */


#define S_FUNCTION_NAME  lat_err_yaw_switch
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
    ssSetInputPortWidth(S, 0, 5);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S,1)) return;
    ssSetOutputPortWidth(S, 0, 5);

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
 *    y = 2*u
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{


//4 outputs are error, segment, u, angle
	real_T *Y = ssGetOutputPortRealSignal(S,0);
	
//4 inputs are Segment number (from previous calculation, x position (East),
//y position(west), and heading angle (CCW from North)
    InputRealPtrsType in = ssGetInputPortRealSignalPtrs(S,0);
	
	int NumSegs=10;
	double ax[10];
	double bx[10];
	double cx[10];
	double dx[10];
	double ay[10]; 
	double by[10];
	double cy[10];
	double dy[10];
	int bestseg, i, ij, seg, ii, imax;
	double error, x1, y0, y1, y2, B, C, temp, y10, y20,y21,xm,ym, tol, angle, rel_angle, bestxdot, bestydot, road_angle, lkswitch;
	double besterror,u, linex, liney, linexdot, lineydot, currenterror, bestu, bestx, besty, a, b, c, d, e, f, x0, x2,x,y;
	besterror=100000; //Start with large best error to decide which segment is best;
	seg		=*in[0];
	x		=*in[1];
	y		=*in[2];
	angle	=*in[3];
    lkswitch=*in[4];

    
	Y[0]=0;        //Output to Y
	Y[1]=-1;            //Output to Y
	Y[2]=-1;             //Output to Y
	Y[3]=0;
	Y[4]=-1;    
if (lkswitch) {    
    
//Here goes the segment data from the fit
//Can be formatted using print params.m    
    
ax[0]=1.109736e+001;bx[0]=-1.377135e+001;cx[0]=1.761786e+001;dx[0]=3.629507e+000;
ay[0]=-7.907547e+000;by[0]=1.649509e+001;cy[0]=-2.584623e+001;dy[0]=-6.136883e+000;
ax[1]=1.740985e+000;bx[1]=1.200924e+000;cx[1]=2.336725e+001;dx[1]=1.857338e+001;
ay[1]=-9.710820e-001;by[1]=1.076818e+001;cy[1]=-1.657869e+001;dy[1]=-2.339557e+001;
ax[2]=1.223151e+000;bx[2]=-4.124009e+000;cx[2]=3.099205e+001;dx[2]=4.488254e+001;
ay[2]=-6.318928e+000;by[2]=1.574871e+001;cy[2]=2.044437e+000;dy[2]=-3.017715e+001;
ax[3]=3.697558e-001;bx[3]=-1.429594e+001;cx[3]=2.641348e+001;dx[3]=7.297373e+001;
ay[3]=-3.252813e+000;by[3]=1.338541e+001;cy[3]=1.458507e+001;dy[3]=-1.870293e+001;
ax[4]=-1.572546e+001;bx[4]=1.686205e+001;cx[4]=-1.069138e+000;dx[4]=8.546102e+001;
ay[4]=-1.168849e+000;by[4]=1.314544e+000;cy[4]=3.159745e+001;dy[4]=6.014734e+000;
ax[5]=1.146000e+001;bx[5]=-2.398118e+001;cx[5]=-1.452144e+001;dx[5]=8.552847e+001;
ay[5]=1.192267e+001;by[5]=-2.510597e+001;cy[5]=3.071999e+001;dy[5]=3.775788e+001;
ax[6]=2.406826e+000;bx[6]=-4.834401e+000;cx[6]=-2.810380e+001;dx[6]=5.848585e+001;
ay[6]=-2.545795e-001;by[6]=-1.659534e+001;cy[6]=1.627604e+001;dy[6]=5.529456e+001;
ax[7]=-3.918476e-001;bx[7]=2.687709e+000;cx[7]=-3.055213e+001;dx[7]=2.795447e+001;
ay[7]=1.458258e-001;by[7]=9.050296e-001;cy[7]=-1.767837e+001;dy[7]=5.472069e+001;
ax[8]=9.807598e+000;bx[8]=6.881038e+000;cx[8]=-2.635225e+001;dx[8]=-3.017983e-001;
ay[8]=1.977899e+000;by[8]=-9.239721e+000;cy[8]=-1.543084e+001;dy[8]=3.809317e+001;
ax[9]=7.260640e+000;bx[9]=-1.049834e+001;cx[9]=1.683262e+001;dx[9]=-9.965414e+000;
ay[9]=-1.074801e+001;by[9]=1.718719e+001;cy[9]=-2.797658e+001;dy[9]=1.540052e+001;

	bestseg=seg;
    
  
if ((fabs(x)<300) & (fabs(y)<300)){	
	for (ij=seg;ij<seg+2;ij++)
	{	
		i=ij;
		if (ij==NumSegs+1)    //Wraparound to first segment if on final segment 
		{
			i=1;
		}        			
	//Mueller's method for finding polynomial roots
		a=-3*pow(ax[i-1],2)-3*pow(ay[i-1],2);
		b=-5*ax[i-1]*bx[i-1]-5*ay[i-1]*by[i-1];
		c=-4*ax[i-1]*cx[i-1]-2*pow(bx[i-1],2)-4*ay[i-1]*cy[i-1]-2*pow(by[i-1],2);
		d=-3*cx[i-1]*bx[i-1]+3*x*ax[i-1]-3*dx[i-1]*ax[i-1]-3*cy[i-1]*by[i-1]+3*y*ay[i-1]-3*dy[i-1]*ay[i-1];
		e=-pow(cx[i-1],2)+2*bx[i-1]*x-2*bx[i-1]*dx[i-1]-pow(cy[i-1],2)+2*by[i-1]*y-2*by[i-1]*dy[i-1];
		f=x*cx[i-1]-dx[i-1]*cx[i-1]+y*cy[i-1]-dy[i-1]*cy[i-1];
		x0=0;
		x2=1.01;
		//tol=0.0000000001;
		tol=  0.0000000001;
		imax=100;
		y0 = a*pow(x0,5)+b*pow(x0,4)+c*pow(x0,3)+d*pow(x0,2)+e*x0+f;
		y2 = a*pow(x2,5)+b*pow(x2,4)+c*pow(x2,3)+d*pow(x2,2)+e*x2+f;
		x1 = 0.5 * (x2 + x0);
		ii=3;
		while ((fabs(x1-x0) > tol) & (ii<imax)) 
		{
			ii++;
			x1 = 0.5 * (x2 + x0);
			y1 = a*pow(x1,5)+b*pow(x1,4)+c*pow(x1,3)+d*pow(x1,2)+e*x1+f;
			if (y1 * y0 > 0.0)
			{
				temp = x0;
				x0 = x2;
				x2 = temp;
				temp = y0;
				y0 = y2;
				y2 = temp;
			}
			y10 = y1 - y0;
			y21 = y2 - y1;
			y20 = y2 - y0;
			if (y2 * y20 <2.0 * y1 * y10)
			{
				x2 = x1;
				y2 = y1;
			}
		    else
			{
				B = (x1 - x0) / y10;   
				C = (y10 -y21) / (y21 * y20); 
				xm = x0 - B * y0 * (1.0-C * y1);
				ym = a*pow(xm,5)+b*pow(xm,4)+c*pow(xm,3)+d*pow(xm,2)+e*xm+f;
				if (ym * y0 < 0.0)
				{
					x2 = xm;
					y2 = ym;
				}
			    else
				{
					x0 = xm;
					y0 = ym;
					x2 = x1;
					y2 = y1;
				}
			}
		}
	u=x1;
	
	linex=ax[i-1]*pow(u,3)+bx[i-1]*pow(u,2)+cx[i-1]*u+dx[i-1];	//  x position on line
	liney=ay[i-1]*pow(u,3)+by[i-1]*pow(u,2)+cy[i-1]*u+dy[i-1];	//  y position on line
	linexdot=3*ax[i-1]*pow(u,2)+2*bx[i-1]*u+cx[i-1];			//  dx/du
	lineydot=3*ay[i-1]*pow(u,2)+2*by[i-1]*u+cy[i-1];			//  dy/du
	currenterror=sqrt(pow((x-linex),2)+pow(y-liney,2));			//  mag of error is sqrt-sum-squares
	if (((x-linex)*lineydot-(y-liney)*linexdot)>0)				// check which side of the road is on
	{															// add neg sign if on right
		currenterror=-currenterror;
	}
	if ((fabs(currenterror)<fabs(besterror)) & (u>-0.01 & u<1.01)) // if this is the better of the
	{															  // current and next segments
		besterror=currenterror;
		bestseg=i;
		bestu=u;
		bestx=linex;
		besty=liney;
		bestxdot=linexdot;
		bestydot=lineydot;
	}
}
}
else{
besterror=0;
bestseg=seg;
bestu=0;
}
if (fabs(besterror)>1000){
	besterror=0;
	bestseg=seg;
	}

//Assumes incoming angle is measured from North/Y, with 
//Counter-Clockwise Positive
	road_angle=atan2(bestxdot,bestydot);
	rel_angle=(angle+road_angle);
	while (rel_angle>3.1415)
	{
	rel_angle=rel_angle-2*3.1415;
	}
	while(rel_angle<-3.1415)
	{
	rel_angle=rel_angle+2*3.1415;
	}
	
	
	Y[0]=besterror;        //Output to Y
	Y[1]=bestseg;            //Output to Y
	Y[2]=bestu;             //Output to Y
	Y[3]=rel_angle;
	Y[4]=ii;
}
    
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
