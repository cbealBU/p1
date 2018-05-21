%POSTPROCESS postprocesses data files using the main model's Pre-recorded 
%   Data block.  This obsoletes the older offline_ssest and postprocess
%   scripts, which have little in common with this one except in spirit.  
%
%   POSTPROCESS with no parameters postprocesses the whole dataset
%       currently in the workspace and puts the result in the workspace, 
%       overwriting the original.  This method of calling is inefficient 
%       (in fact, it writes a temporary mat-file) and intended for backward
%       compatibility only.
%
%   POSTPROCESS(starttime,stoptime) does the same as above, but processes 
%       the data only between the start and stop times specified.
% 
%   POSTPROCESS('filename') postprocesses the whole dataset from the 
%       filename given and puts the result into the workspace.
%
%   POSTPROCESS('filename',starttime,stoptime) does the same as above,
%       but processes the data only between the start and stop times
%       specified.
%
%   [t,y,DataDescription]=POSTPROCESS(...) outputs the results in the
%       variable names given, and works with or without specified start and
%       stop times.
% 
%   Version 1.0 by Shad Laws, 11/13/06.
%   Version 1.1 by Shad Laws, 11/14/06, added backward compatibility modes.

function [tout,yout,DataDescriptionout]=postprocess(param1,param2,param3);

    % NOTE: there are two lines with DEBUGGING in the comments after it.
    % To facilitate debugging of this code, heed what they say :-).
    
    % parameters
    mainmodelname = 'nissan_steer';
    iolibname = 'PrerecordData';
    tempmodelname = 'temp_postprocess';
    tempdatafilename = 'temp_postprocessdata';
    ioblockname = 'Hardware I//O';
    prerecordblockname = 'Pre-recorded Data';
    prerecordsteerswitchname = 'PRS Switch';
    
    % delete old copy of the temp model if it exists (from previous
    % incomplete function run or something)
    existtempmodel = (exist([tempmodelname '.mdl'])~=0);
    if (existtempmodel==1);
        close_system(tempmodelname);
        delete([tempmodelname '.mdl']);
    end;
    
    % generate temp model and load it
    copyfile([mainmodelname '.mdl'],[tempmodelname '.mdl']);
    load_system([tempmodelname '.mdl']);
    % open_system([tempmodelname '.mdl']);  % DEBUGGING: uncomment this and comment the previous line to make the model run in the foreground

    % figure out which i/o blocks exist
    ioexist = sum(strcmp(find_system(tempmodelname),[tempmodelname '/' ioblockname]));
    prerecordexist = sum(strcmp(find_system(tempmodelname),[tempmodelname '/' prerecordblockname]));

    % delete Hardware I/O if it's there
    if (ioexist==1);
        delete_block([tempmodelname '/' ioblockname]);
    end;

    % add Pre-recorded Data if it's not there
    if (prerecordexist==0);
        load_system([iolibname '.mdl']);
        add_block([iolibname '/' prerecordblockname],[tempmodelname '/' prerecordblockname]);
        close_system([iolibname '.mdl']);
    end;
    
    % make it use pre-recorded steering commands
    set_param([tempmodelname '/' prerecordsteerswitchname],'prerecorded_flag','on');

    % set the start and stop times (if any)
    if nargin==0;
        % make the temp data file then use it with no custom start/stop times
        evalin('caller',['save ' tempdatafilename ' t y DataDescription']);
        set_param([tempmodelname '/' prerecordblockname],'filename',tempdatafilename);
        set_param([tempmodelname '/' prerecordblockname],'timeflag','off');
    elseif nargin==1;
        % no custom start/stop times
        set_param([tempmodelname '/' prerecordblockname],'filename',param1);
        set_param([tempmodelname '/' prerecordblockname],'timeflag','off');
    elseif nargin==2;
        % make the temp data file then use it with custom start/stop times
        evalin('caller',['save ' tempdatafilename ' t y DataDescription']);
        set_param([tempmodelname '/' prerecordblockname],'filename',tempdatafilename);
        set_param([tempmodelname '/' prerecordblockname],'timeflag','on');
        set_param([tempmodelname '/' prerecordblockname],'starttime',num2str(param1));
        set_param([tempmodelname '/' prerecordblockname],'stoptime',num2str(param2));
    elseif nargin==3;
        % custom start/stop times
        set_param([tempmodelname '/' prerecordblockname],'filename',param1);
        set_param([tempmodelname '/' prerecordblockname],'timeflag','on');
        set_param([tempmodelname '/' prerecordblockname],'starttime',num2str(param2));
        set_param([tempmodelname '/' prerecordblockname],'stoptime',num2str(param3));
    else;
        % incorrect function call
        disp('Incorrect number of input arguments.');
        return;
    end;
    save_system(tempmodelname);

    % run the temp model and generate DataDescription
    % return;  % DEBUGGING: uncomment this to end postprocess just before it runs the model
    sim(tempmodelname);
    DataDescription = datadesc;

    % delete the temp files
    close_system(tempmodelname);
    delete([tempmodelname '.mdl']);
    if nargin==0;
        delete([tempdatafilename '.mat']);
    end;

    % assign the variables based on the number of output arguments
    if nargout==0;
        % assign them in the base workspace
        assignin('caller','t',t);
        assignin('caller','y',y);
        assignin('caller','DataDescription',DataDescription);
    elseif nargout==2;
        % assign t and y
        tout = t;
        yout = y;
    elseif nargout==3;
        % assign t, y, and DataDescription
        tout = t;
        yout = y;
        DataDescriptionout = DataDescription;
    else;
        disp('Incorrect number of output arguments.');
        return;
    end;
        
return;