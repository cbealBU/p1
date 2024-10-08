% This script pulls data off of a Flexcase, merges the component log files,
% and creates a single mat file in a data subdirectory

% Author: Craig Beal
% Using previous work from Graham Heckert, Chris Gadda, Shad Laws


% Setup up object to communicate with raspberry pi
flexCase = raspberrypi('FlexCase01.local','pi','FazK75niXS');

% Pull data to host computer
clc
disp('Retrieving data from FlexCase...');
disp('Data files are available from the following models/runs: ');
% Grab the list of data files (just the first one in any sequence)
datafilelist = splitlines(system(flexCase,'ls *_1.mat'));

% Create a pattern to match the end of the file name to strip it off
stripPattern = "_" + digitsPattern + ".mat";
% Loop through all of the available data files
for listCount = 1:length(datafilelist)
    % Check for empty file names (usually at the very end due to the system
    % command)
    if isempty(datafilelist{listCount})
        datafilelist(listCount) = [];
        % If the data file is not empty, strip off the end of it and show the
        % root name and run number to the user
    else
        printString = extractBefore(datafilelist{listCount},stripPattern);
        fprintf(1,'%d) %s\n',listCount,printString)
    end
end
% Now request the run the user would like to import from the list just
% displayed
fileNum = 0;
while fileNum < 1 || fileNum > length(datafilelist)
    fileNum = input('Type the number of the model/run to retrieve: ');
end
retrieveFileBase = convertStringsToChars(extractBefore(datafilelist{fileNum},stripPattern));
retrieveFileName = convertStringsToChars(extractBefore(datafilelist{fileNum},stripPattern) + "_*.mat");

% Clean up
clear datafilelist fileNum listCount printString stripPattern

% Also request from the user whether or not to delete the file when
% imported
deleteFlag = lower(input('Delete the file from the Flexcase after importing (Yes/[No])? ','s'));
if isempty(deleteFlag)
    deleteFlag = 'no';
end
getFile(flexCase,retrieveFileName); % Change to filename you are looking for

% FileDataStore() gives option to choose where to read from, so use that in
% later implementation to give flexibility for user
fds = fileDatastore(retrieveFileName,'ReadFcn',@importdata);
fullFileNames = fds.Files;
% sortedFileNames = {};
for k = 1:length(fullFileNames)
     fprintf('Now reading file %s\n',fullFileNames{k});
end
clear k

fprintf('Creating info structure for data file...\n');
% Grab the date
info.date=datestr(now,29);
info.time=erase(datestr(now,13),":");
% Building info structure from user input
info.driver=deblank(input('Driver: ','s'));
info.testloc=deblank(input('Test Location: ','s'));
info.tyPress=deblank(input('Tire Pressures: ','s'));
info.ambT=deblank(input('Ambient Temp: ','s'));
% Enter a description for the data file/test
description='';
des=input('Description:  (End with a . on a line by itself.)\n','s');
while(~strcmp(des,'.'))
    description=[description '\n' des];
    des=input('','s');
end
info.description=description;

if length(fds.Files) > 1
    % Give option to run matlab stitcher
    stitchFlag = lower(input("Run file stitcher? ([Yes]/No): ",'s'));
    if isempty(stitchFlag)
        stitchFlag = 'yes';
    end
    if strcmp(stitchFlag,'yes')
        fprintf(1,'Stitching files... ');
        Raspberrypi_MAT_stitcher(dir([retrieveFileBase '*']));
        fprintf(1,' complete.\n')
    end
    oldFileString = [retrieveFileBase '__stitched.mat'];
    newFileString = ['./data/' retrieveFileBase '_' datestr(now,'yyyy-mm-dd_HH-MM-SS') '.mat'];
    if ~exist('./data','dir')
        mkdir('data');
    end
else
    stitchFlag = 'no';
    oldFileString = [retrieveFileBase '_1.mat'];
end

if strcmp(stitchFlag,'no') && length(fds.Files) > 1
    disp('Files not stitched, leaving them in current directory. No data loaded.');
else
    newFileString = ['./data/' retrieveFileBase '_' datestr(now,'yyyy-mm-dd_HH-MM-SS') '.mat'];
    movefile(oldFileString,newFileString);
    fprintf(1,'Files stitched (or was only a single file) and moved to the data subdirectory as %s.\n',...
        [retrieveFileBase '_' datestr(now,'yyyy-mm-dd_HH-MM-SS') '.mat'])

    save(newFileString,"info",'-append')
    clear rt_* rmCmd
    load(newFileString)
    
    % Do an error check on the stitching function
    if any(diff(rt_tout) <= 0)
        warning('Time vector is not continuously increasing. Data collection or stitching may have encountered errors.')
    end
    
    disp('Final data file reloaded into the workspace.')
    clear newFileString
    delete(retrieveFileBase + "*.mat")

end

if strcmp(deleteFlag,'yes')
    rmCmd = ['rm -r ' retrieveFileBase '*.mat'];
    system(flexCase,rmCmd);
    clear rmCmd
end
clear oldFileString retrieveFileBase retrieveFileName
clear stitchFlag deleteFlag fds fullFileNames