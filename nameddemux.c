/*
 * File: nameddemux.c
 *
 * Author: Christopher Gadda
 * (C)2004 Dynamic Design Laboratory -- Stanford University
 *
 * This file implements an s-function for use with Simulink, which is the
 * complement to namedmux.c.  Data which has been combined by a namedmux
 * block can be split apart into it's constituent signals.  See namedmux.c
 * for more details.
 *
 * This file is based (very loosely) on $MATLAB/simulink/src/sfuntmpl_basic.c
 */

#define S_FUNCTION_NAME  nameddemux
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define NUMOFPARAMS 4
#define NUMOFSTATES	0

static char msg[80];

/*====================*
 * S-function methods *
 *====================*/
static void mdlInitializeSizes(SimStruct *S)
{
    int i;
    uint_T inputcount, outputcount;
    real_T *outputwidths;
	real_T *outputindices;
	uint_T inputsize;
	const mxArray *mxiw, *mxoi;
    uint_T nod;
    const int *dims;
    
    ssSetNumSFcnParams(S, NUMOFPARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        ssSetErrorStatus(S,"Incorrect number of parameters.");
        return;
    }

    outputcount=(uint_T)(*mxGetPr(ssGetSFcnParam(S,0)));
    mxiw=ssGetSFcnParam(S,1);
    outputwidths=mxGetPr(mxiw);
    nod=mxGetNumberOfDimensions(mxiw);
    if(nod!=2)
    {
        ssSetErrorStatus(S,"The second parameter needs to be vector of port widths.");
        return;
    }
    
    dims=mxGetDimensions(mxiw);
    if(dims[0]*dims[1]!=outputcount)
    {
        ssSetErrorStatus(S,"The second parameter is not the same length as the number of outputs.");
        return;
    }
    
    mxoi=ssGetSFcnParam(S,2);
    outputindices=mxGetPr(mxoi);
    nod=mxGetNumberOfDimensions(mxoi);
    if(nod!=2)
    {
        ssSetErrorStatus(S,"The third parameter needs to be vector of indices.");
        return;
    }
    
    dims=mxGetDimensions(mxoi);
    if(dims[0]*dims[1]!=outputcount)
    {
        ssSetErrorStatus(S,"The third parameter is not the same length as the number of outputs.");
        return;
    }

    /* Time for a quick sanity check. */
    if((outputcount<1)||(outputcount>50))
    {
        sprintf(msg,"Hey, uh, this doesn't seem right.  You've asked for %d outputs\n",inputcount);
        ssSetErrorStatus(S,msg);
        inputcount=1;
        return;
    }

    inputsize=(uint_T)(*mxGetPr(ssGetSFcnParam(S,3)));
    

	ssSetNumContStates(S, NUMOFSTATES);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 1)) {
    	return;
    }

    if (!ssSetNumOutputPorts(S, outputcount)) {
    	return;
    }

	ssSetInputPortDirectFeedThrough(S, 0, 1);
    
    inputcount=0;
    for(i=0;i<outputcount;i++)
    {
		inputcount+=(int)outputwidths[i];
        ssSetOutputPortWidth(S, i, (int)outputwidths[i]);
    }
    
    ssSetInputPortWidth(S, 0, inputsize);
    if(inputsize<inputcount)
	{
        ssSetErrorStatus(S,"The input vector size is too small.");
        return;
	}
	
	
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
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#undef MDL_INITIALIZE_CONDITIONS
#undef MDL_DERIVATIVES

static void mdlOutputs(SimStruct *S, int_T tid)
{
	InputRealPtrsType uPtrs;
    int_T ny;
	int i,j,indx=0;

    real_T *y;
	int_T nOutputPorts = ssGetNumOutputPorts(S);
	real_T *outputindices=mxGetPr(ssGetSFcnParam(S,2));

	uPtrs=ssGetInputPortRealSignalPtrs(S,0);
	
	for(i=0;i<nOutputPorts;i++)
	{
	    y=	ssGetOutputPortRealSignal(S,i);
		ny = ssGetOutputPortWidth(S,i);
	    for(j=0;j<ny;j++)
		{
	        y[j]=*(uPtrs[(int)outputindices[i]-1+j]);
		}
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

#ifdef _COMPILE_USELESS_CODE_
/* This section goes in the Icon panel of the mask dialog box*/
/****************************************
port_label('output',x(1),desc(y(x(1))).name);
port_label('output',x(2),desc(y(x(2))).name);
port_label('output',x(3),desc(y(x(3))).name);
port_label('output',x(4),desc(y(x(4))).name);
port_label('output',x(5),desc(y(x(5))).name);
port_label('output',x(6),desc(y(x(6))).name);
port_label('output',x(7),desc(y(x(7))).name);
port_label('output',x(8),desc(y(x(8))).name);
port_label('output',x(9),desc(y(x(9))).name);
port_label('output',x(10),desc(y(x(10))).name);
port_label('output',x(11),desc(y(x(11))).name);
port_label('output',x(12),desc(y(x(12))).name);
port_label('output',x(13),desc(y(x(13))).name);
port_label('output',x(14),desc(y(x(14))).name);
port_label('output',x(15),desc(y(x(15))).name);
port_label('output',x(16),desc(y(x(16))).name);
port_label('output',x(17),desc(y(x(17))).name);
port_label('output',x(18),desc(y(x(18))).name);
port_label('output',x(19),desc(y(x(19))).name);
port_label('output',x(20),desc(y(x(20))).name);
port_label('output',x(21),desc(y(x(21))).name);
port_label('output',x(22),desc(y(x(22))).name);
port_label('output',x(23),desc(y(x(23))).name);
port_label('output',x(24),desc(y(x(24))).name);
port_label('output',x(25),desc(y(x(25))).name);
port_label('output',x(26),desc(y(x(26))).name);
port_label('output',x(27),desc(y(x(27))).name);
port_label('output',x(28),desc(y(x(28))).name);
port_label('output',x(29),desc(y(x(29))).name);
port_label('output',x(30),desc(y(x(30))).name);
****************************************/

/* This section goes in the Initialization panel of the mask dialog box. */
/****************************************
desc=eval(fname);

outputcount=length(signals);
inputsize=sum([desc.size]);

indices=[1 cumsum([desc.size])+1];

if((outputcount<1)|(outputcount>50))
	error('Inappropriate number of outputs.');
end

y=ones(1,outputcount);

for indx=1:outputcount,
    sig=signals{indx};
	sigi=find(strcmp(sig,{desc.name}));
    if (length(sigi)==0)
        error(['Unable to find signal named "' sig '".']);
    end
    y(indx)=sigi;
end
		
outputsizes=[desc(y).size];
outputindices=indices(y);

x=min(outputcount*ones(1,50),1:50);
****************************************/
#endif
