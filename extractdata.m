function [data]=extractdata(y,dd,signal,startingIndex)
%function [data]=extractdata(y,dd,signal)
% extractdata.m -- Extracts a portion of XPC data by name or number.
%
% [data]=extractdata(y,dd,signal)
%
% This function takes some XPC data, y, a description of that data,
% dd, and the name of a signal in that data, signal, to be extracted.
% StartingIndex is the first index of the signal, in datadesc, this will
% equal to 1, but if read from debugger, startingIndex will =
% sum([DataDescription.size]).  Note that startingIndex=0 if labelling the
% data from datadesc
% It returns the requested subset of the data, or generates an error
% if no such named subset exists.  Optionally, signal can be an integer, in
% which case the signal returned is looked by number instead of name.
%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Version: $Revision: 107 $
% Last modified on: $Date: 2005-07-10 12:22:29 -0700 (Sun, 10 Jul 2005) $  by: $Author: cgadda $

indx=startingIndex+1;

for signo=1:length(dd),
    sigsize=dd(signo).size;
    if(isa(signal,'double'))
		if(signo==signal)
			data=y(:,indx:indx+sigsize-1);
			return;
		end
	else
		if(strcmp(dd(signo).name,signal))
			data=y(:,indx:indx+sigsize-1);
			return;
		end
	end
    indx=indx+sigsize;
   
end

error('Unable to find requested signal');
