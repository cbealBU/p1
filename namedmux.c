/*
 * File: namedmux.c
 *
 * Author: Christopher Gadda
 * (C)2004 Dynamic Design Laboratory -- Stanford University
 *
 * This file implements an s-function for use with Simulink, which combines
 * many input signals into a single output vector, much the same way that an
 * ordinary Mux does.  When masked appropriately, this s-function creates a
 * block which take a single argument, a cell array structure which describes
 * the names and sizes of each input.  The masked block will then display an
 * input port for each input, labeled with it's name.
 *
 * What is the point of all this?  Well, if you have a bunch of signals in
 * a model that you want to save, you can give them all names, connect them
 * to a named mux, and then save the cell array structure that you used to
 * define the named mux along with the data.  Then, when you can't remember
 * which element of your data vector is what anymore, you'll have the cell
 * array structure describing it all for you.
 *
 * Better still, you can use the cell array structure when writing scripts
 * or functions that process the recorded data, so that you can refer to
 * various signals by name instead of number.  This way, when you change you
 * model, delete some signals, and add some new ones, you won't have to
 * rewrite the post-processing code.
 *
 * This file is based (very loosely) on $MATLAB/simulink/src/sfuntmpl_basic.c
 */

#define S_FUNCTION_NAME  namedmux
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define NUMOFPARAMS 2
#define NUMOFSTATES	0

static char msg[80];

/*====================*
 * S-function methods *
 *====================*/
static void mdlInitializeSizes(SimStruct *S)
{
    int i;
    uint_T inputcount,outputcount;
    const real_T *inputwidths;
    const mxArray *mxiw;
    uint_T nod;
    const int *dims;
    
    ssSetNumSFcnParams(S, NUMOFPARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        ssSetErrorStatus(S,"Incorrect number of parameters.");
        return;
    }

    inputcount=(uint_T)(*mxGetPr(ssGetSFcnParam(S,0)));
    mxiw=ssGetSFcnParam(S,1);
    inputwidths=mxGetPr(mxiw);
    nod=mxGetNumberOfDimensions(mxiw);
    if(nod!=2)
    {
        ssSetErrorStatus(S,"The second parameter needs to be vector of port widths.");
        return;
    }
    
    dims=mxGetDimensions(mxiw);
    if(dims[0]*dims[1]!=inputcount)
    {
        ssSetErrorStatus(S,"The second parameter is not the same length as the number of inputs.");
        return;
    }
    
    /* Time for a quick sanity check. */
    if((inputcount<1)||(inputcount>50))
    {
        sprintf(msg,"Hey, uh, this doesn't seem right.  You've asked for %d inputs\n",inputcount);
        ssSetErrorStatus(S,msg);
        inputcount=1;
        return;
    }
    
    ssSetNumContStates(S, NUMOFSTATES);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, inputcount)) {
    	return;
    }
    
    outputcount=0;
    for(i=0;i<inputcount;i++)
    {
        outputcount+=(int)inputwidths[i];
        ssSetInputPortWidth(S, i, (int)inputwidths[i]);
        ssSetInputPortDirectFeedThrough(S, i, 1);
    }
    
    if (!ssSetNumOutputPorts(S, 1)) {
    	return;
    }
    ssSetOutputPortWidth(S, 0, outputcount);

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
    int_T nu;
    real_T Ux;    
	int i,j,indx=0;

    real_T *y = ssGetOutputPortRealSignal(S,0);
	int_T nInputPorts = ssGetNumInputPorts(S);

	for(i=0;i<nInputPorts;i++)
	{
	    uPtrs=ssGetInputPortRealSignalPtrs(S,i);
	    nu = ssGetInputPortWidth(S,i);
	    for(j=0;j<nu;j++)
	        y[indx++]=*(uPtrs[j]);
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
port_label('input',x(1),desc{x(1)}{1});
port_label('input',x(2),desc{x(2)}{1});
port_label('input',x(3),desc{x(3)}{1});
port_label('input',x(4),desc{x(4)}{1});
port_label('input',x(5),desc{x(5)}{1});
port_label('input',x(6),desc{x(6)}{1});
port_label('input',x(7),desc{x(7)}{1});
port_label('input',x(8),desc{x(8)}{1});
port_label('input',x(9),desc{x(9)}{1});
port_label('input',x(10),desc{x(10)}{1});
port_label('input',x(11),desc{x(11)}{1});
port_label('input',x(12),desc{x(12)}{1});
port_label('input',x(13),desc{x(13)}{1});
port_label('input',x(14),desc{x(14)}{1});
port_label('input',x(15),desc{x(15)}{1});
port_label('input',x(16),desc{x(16)}{1});
port_label('input',x(17),desc{x(17)}{1});
port_label('input',x(18),desc{x(18)}{1});
port_label('input',x(19),desc{x(19)}{1});
port_label('input',x(20),desc{x(20)}{1});
port_label('input',x(21),desc{x(21)}{1});
port_label('input',x(22),desc{x(22)}{1});
port_label('input',x(23),desc{x(23)}{1});
port_label('input',x(24),desc{x(24)}{1});
port_label('input',x(25),desc{x(25)}{1});
port_label('input',x(26),desc{x(26)}{1});
port_label('input',x(27),desc{x(27)}{1});
port_label('input',x(28),desc{x(28)}{1});
port_label('input',x(29),desc{x(29)}{1});
port_label('input',x(30),desc{x(30)}{1});
****************************************/

/* This section goes in the Initialization panel of the mask dialog box. */
/****************************************
if(class(desc)~='cell')
	error('Incorrect description');
end
inputcount=length(desc);
if((inputcount<1)|(inputcount>50))
	error('Inappropriate number of inputs.');
end
inputsizes=ones(1,inputcount);
for indx=1:inputcount,
	inputsizes(indx)=desc{indx}{2};
end
x=min(inputcount*ones(1,50),1:50);
****************************************/
#endif
