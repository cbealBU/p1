/*	7/09/02 JPS
 * File : lat_err_yaw.c
 *
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


#define S_FUNCTION_NAME  lat_err_yaw4_switch_map3
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
    ssSetOutputPortWidth(S, 0, 4);

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
	
	int NumSegs=6;
	double ax[6];
	double bx[6];
	double cx[6];
	double dx[6];
	double ex[6];
	double ay[6]; 
	double by[6];
	double cy[6];
	double dy[6];
	double ey[6];
	int bestseg, i, ij, seg, ii, imax;
	double error, x1, y0, y1, y2, B, C, temp, y10, y20,y21,xm,ym, tol, angle, rel_angle, bestxdot, bestydot, road_angle, lkswitch;
	double besterror,u, linex, liney, linexdot, lineydot, currenterror, bestu, bestx, besty, a, b, c, d, e, f, g, h, x0, x2,x,y;
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
//Can be formatted using print params4.m
ax[0]=17.342294;bx[0]=-35.658374;cx[0]=26.692949;dx[0]=-40.551374;ex[0]=54555.976147;
ay[0]=-7.506342;by[0]=12.084582;cy[0]=-16.912590;dy[0]=1.722701;ey[0]=34233.993440;
ax[1]=-0.966289;bx[1]=-1.169322;cx[1]=23.771588;dx[1]=-24.771425;ex[1]=54523.801640;
ay[1]=-3.017582;by[1]=17.663335;cy[1]=-25.696895;dy[1]=-25.874100;ey[1]=34223.381791;
ax[2]=-3.844370;bx[2]=3.108791;cx[2]=14.465888;dx[2]=15.398629;ex[2]=54520.666193;
ay[2]=0.874870;by[2]=1.824529;cy[2]=9.187617;dy[2]=-36.348213;ey[2]=34186.456549;
ax[3]=-8.229309;bx[3]=7.224949;cx[3]=0.726044;dx[3]=38.279300;ex[3]=54549.795132;
ay[3]=8.392590;by[3]=-11.362737;cy[3]=19.910424;dy[3]=-8.999912;ey[3]=34161.995352;
ax[4]=10.492951;bx[4]=-11.118336;cx[4]=-26.974963;dx[4]=28.488999;ex[4]=54587.796116;
ay[4]=15.105633;by[4]=-41.294895;cy[4]=36.177753;dy[4]=30.303085;ey[4]=34169.935717;
ax[5]=26.513965;bx[5]=-45.006193;cx[5]=2.627737;dx[5]=-16.844129;ex[5]=54588.684767;
ay[5]=11.744007;by[5]=-30.101167;cy[5]=2.926866;dy[5]=39.196439;ey[5]=34210.227294;
//End of parameters

	bestseg=seg;
if ((fabs(x)<300000) & (fabs(y)<300000)){	
	for (ij=seg;ij<seg+2;ij++)
	{	
		i=ij;
		if (ij==NumSegs+1)    //Wraparound to first segment if on final segment 
		{
			i=1;
		}        			
	//Mueller's method for finding polynomial roots
	a=4*pow(ax[i-1],2)+4*pow(ay[i-1],2);
    b=7*ax[i-1]*bx[i-1]+7*ay[i-1]*by[i-1];
    c=6*ax[i-1]*cx[i-1]+3*pow(bx[i-1],2)+6*ay[i-1]*cy[i-1]+3*pow(by[i-1],2);
    d=5*ax[i-1]*dx[i-1]+5*bx[i-1]*cx[i-1]+5*ay[i-1]*dy[i-1]+5*by[i-1]*cy[i-1];
    e=4*bx[i-1]*dx[i-1]+2*pow(cx[i-1],2)+4*ax[i-1]*ex[i-1]-4*ax[i-1]*x+4*by[i-1]*dy[i-1]+2*pow(cy[i-1],2)+4*ay[i-1]*ey[i-1]-4*ay[i-1]*y;
    f=-3*bx[i-1]*x+3*cx[i-1]*dx[i-1]+3*bx[i-1]*ex[i-1]+-3*by[i-1]*y+3*cy[i-1]*dy[i-1]+3*by[i-1]*ey[i-1];
    g=-2*cx[i-1]*x+dx[i-1]*dx[i-1]+2*cx[i-1]*ex[i-1]+-2*cy[i-1]*y+dy[i-1]*dy[i-1]+2*cy[i-1]*ey[i-1];
    h=-dx[i-1]*x+dx[i-1]*ex[i-1]+-dy[i-1]*y+dy[i-1]*ey[i-1];
    
		x0=-0.01;
		x2=1.01;
		//tol=0.0000000001;
		tol=  0.0000000001;
		//imax=50;
		imax=30;
		y0 = a*pow(x0,7)+b*pow(x0,6)+c*pow(x0,5)+d*pow(x0,4)+e*pow(x0,3)+f*pow(x0,2)+g*x0+h;
		y2 = a*pow(x2,7)+b*pow(x2,6)+c*pow(x2,5)+d*pow(x2,4)+e*pow(x2,3)+f*pow(x2,2)+g*x2+h;
		x1 = 0.5 * (x2 + x0);
		ii=0;
		while ((fabs(x1-x0) > tol) & (ii<imax)) 
		{
			ii++;
			x1 = 0.5 * (x2 + x0);
			y1 = a*pow(x1,7)+b*pow(x1,6)+c*pow(x1,5)+d*pow(x1,4)+e*pow(x1,3)+f*pow(x1,2)+g*x1+h;
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
				ym = a*pow(xm,7)+b*pow(xm,6)+c*pow(xm,5)+d*pow(xm,4)+e*pow(xm,3)+f*pow(xm,2)+g*xm+h;
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
	
	linex=ax[i-1]*pow(u,4)+bx[i-1]*pow(u,3)+cx[i-1]*pow(u,2)+dx[i-1]*u+ex[i-1];	//  x position on line
	liney=ay[i-1]*pow(u,4)+by[i-1]*pow(u,3)+cy[i-1]*pow(u,2)+dy[i-1]*u+ey[i-1];	//  y position on line
	linexdot=4*ax[i-1]*pow(u,3)+3*bx[i-1]*pow(u,2)+2*cx[i-1]*u+dx[i-1];			//  dx/du
	lineydot=4*ay[i-1]*pow(u,3)+3*by[i-1]*pow(u,2)+2*cy[i-1]*u+dy[i-1];			//  dy/du
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
