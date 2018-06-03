%function varargout = savedata(varargin)
% savedata
%
%     This function displays a dialog which prompts for information about
%     the data being saved, then writes the data and the information into
%     a uniquely-named file.  The data to be saved should be in global
%     variables y & t, as will be the case if the command getdata is used
%     to retrieve the data from the xpc target.


% Last Modified by GUIDE v2.5 13-Sep-2004 17:44:51

% if(nargin==2)
%     y=varargin{2};
%     t=varargin{1};
% else
%     
% end

% First check for the existence of tg.
if(~exist('tg'))
    error('Cannot find a variable named "tg".  Unable to control xPC target.');
end

% Stop the model (if not already stopped) and retrieve the data...
getdata;

% Build the info structure...
info.time=datestr(now,31);
info.modelname=tg.Application;
info.userblockname=UserBlockName;

% Ok, we want to get some info, but we need the model files to be loaded into
% memory for this to work.  So first we see if they're already available.
if(isempty(find_system('name',info.modelname)))
    load_system(info.modelname);
	info.modelversion=get_param([info.modelname '/Model Info'],'MaskDisplayString');
    close_system(info.modelname);
else
	info.modelversion=get_param([info.modelname '/Model Info'],'MaskDisplayString');
end

% This part is hard-coded to look for a library named HardwareIO.  That's
% sort of unfortunate, but hopefully I'll think of a better way to do this
% at some point.
if isempty(find_system('name','HardwareIO'))
	load_system('HardwareIO');
	info.libversion=get_param('HardwareIO/Model Info','MaskDisplayString');
	close_system('HardwareIO');
else
	info.libversion=get_param('HardwareIO/Model Info','MaskDisplayString');
end

% Now we'd like to get Subversion's svnversion string, which will tell us
% the latest revision number of any file in the whole directory tree, as
% well as whether there are uncommitted modifications to any files in this
% directory tree.  Note that the Model Info blocks above only record the
% revision numbers of HardwareIO.mdl and nissan_steer.mdl, which doesn't
% let you know if there were changes to an s-function or an initialization
% .m file.  That's why we need to record the global revision number, too.
[svnversion_status,svnversion_result]=system('svnversion');
if svnversion_status
    warning('Subversion does not appear to be correctly installed on your system.');
    disp('You should correct this problem as soon as possible.');
    info.svnversion='-0-';
else
    info.svnversion=svnversion_result;
end

info.driver=lower(deblank(input('Driver: ','s')));
info.testnum=str2num(input('Test number: ','s'));
info.testloc=lower(deblank(input('Test Location: ','s')));

description='';
des=input('Description:  (End with a . on a line by itself.)\n','s');
while(~strcmp(des,'.'))
    description=[description '\n' des];
    des=input('','s');
end
info.description=description;


dirlist=dir([info.driver '_' datestr(now,29) '_*.mat']);

if(size(dirlist,1))
    lastone=dirlist(end);
    letters=lastone.name(end-(5:-1:4));
	if(letters(2)<'z')
		letters(2)=char(letters(2)+1);
	else
		letters(2)='a';
		letters(1)=char(letters(1)+1);
	end
else
    letters='aa';
end

info.run=letters;

% Create a unique filename, putting the data in a "data" subdirectory.
fname=['data/' info.driver '_' datestr(now,29) '_' info.run ];

% This part shouldn't be necessary.  But just to be safe...
dirlist=dir([fname '.mat']);
if(size(dirlist,1))
	error(['Internal savedata error.  Tried to save data as: ' fname ...
			'.mat but that file already exists!']);
end
        
% Run the sanity checker
sanity;

% Now save the file...
if exist('DataDescriptionUser')
    save(fname,'y','t','TET','info','DataDescription','DataDescriptionUser');
else
    save(fname,'y','t','TET','info','DataDescription');
end

% Summarize things
disp(['Model name: ' info.modelname]);
disp(['Version: ' info.modelversion]);
disp(['Time & Date: ' info.time]);
disp(['Driver: ' info.driver]);
disp(sprintf('Data dimensions: %d signals at %d data points',size(y,2),size(y,1)));
disp(['Max TET: ' num2str(max(TET)) ' with Ts= ' num2str(Ts)]);            
disp(['Data successfully saved to file: ' fname '.mat']);
