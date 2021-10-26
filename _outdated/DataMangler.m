function DataMangler(directory)
% This function opens older datasets which do not have speed or lateral
% acceleration entries in their info variable.  For each dataset the
% appropriate values are computed by examining DataDescription & y and
% then the info variable is changed to include this info.  The dataset is
% then resaved.  It might be a good idea to make copies of the original
% unmodified data before running this script.

	[HAL,TestNumbers,hints]=GetHALData();

	% First, we'll look up the path separator for our particular OS.
	% Unix uses /, Windows uses \.  
	slash=PathSeparator();

	% We need a low-pass filter so we can smooth the data out a bit.
	[num,den]=butter(5,.05,'low');  % Good for ay data.
	[num2,den2]=butter(5,.02,'low');  % Good for speed data.
	
	% Now, we iterate through all subdirectories, looking for
	% datasets that need upgrading.  If we find a .mat file that doesn't
	% contain an "info" variable, we skip it.
	path=directory;
	while(path)
		% Find data files in the directory.
		files=dir([path slash '*.mat']);

		% Examine each file.
		for ii=1:length(files)
			info=ReadInfo([path slash files(ii).name]);

			if(~isfield(info,'description'))
				disp([files(ii).name ' is not a valid dataset.']);
				continue;
			end
			%if(isfield(info,'speed')&isfield(info,'maxlat')&isfield(info,'HAL'))
			%	disp([files(ii).name ' already has speed and lateral acceleration info.']);
			%	continue;
			%end

			% Ok, this dataset needs some fixin' up.
			% First, load in the data.  This is the slow part.
			data=load([path slash files(ii).name],'DataDescription','y','t');
			DataDescription=data.DataDescription;
			y=data.y;
			t=data.t;
			
			% Now, find the particular signals we need.
			INS=[];
			Wheelspeeds=[];
			Vehicle_State=[];
			for signo=1:length(DataDescription),
				if(strcmp(DataDescription(signo).name,'INS'))
					INS=extractdata(y,DataDescription,signo);
				end
				if(strcmp(DataDescription(signo).name,'Wheelspeeds'))
					Wheelspeeds=extractdata(y,DataDescription,signo);
				end
				if(strcmp(DataDescription(signo).name,'Vehicle_State'))
					Vehicle_State=extractdata(y,DataDescription,signo);
				end
			end
			% Compute any missing fields, but don't replace existing ones.
			%if(~isfield(info,'speed'))
				info.speed=0;
				if(~isempty(Wheelspeeds))
					filteredSpeeds=filtfilt(num2,den2,500/96*.3085*2*pi*sum(Wheelspeeds(:,1:2),2)/2);
					info.speed=max(filteredSpeeds);
				else
					allSpeeds=Vehicle_State(:,3);
					legitSpeeds=allSpeeds(find((allSpeeds<40)&(allSpeeds>0)));
					filteredSpeeds=filtfilt(num2,den2,legitSpeeds);
					info.speed=max(filteredSpeeds);
				end
			%end
			if(~isfield(info,'maxlat'))
				info.maxlat=max(abs(filtfilt(num,den,INS(:,4))));
				if(info.maxlat>15)
					info.maxlat=0;  % If there's something screwy, toss it.
				end
			end
			if(~isfield(info,'HAL'))
				if(isfield(info,'testnum'))
					info.HAL=HAL(find(info.testnum==TestNumbers));
				end
			end
			if(~isfield(info,'hints'))
				if(isfield(info,'testnum'))
					info.hints=hints(find(info.testnum==TestNumbers));
				end
			end
			save([path slash files(ii).name],'DataDescription','y','t','info');
		end

		path=walk(path,directory);
	end
	
end % -- end of DataIndexer function --

function info=ReadInfo(file)
	warningSettings=warning('off');
	try
		info=load(file,'info');
	catch
		warning(warningSettings);
	end
	warning(warningSettings);
	if(isfield(info,'info'))
		info=info.info;
	end
end % -- end of ReadInfo function --







% -- walk function -- %

% This function "walks" a directory tree.  Pass in any path as the first
% argument, and a different path will be returned.  If you always pass in
% the path returned by a previous call to walk, you will eventually visit
% every subdirectory of the original path passed to walk.
% The second argument (which is optional) specifies an effective "root"
% path, which establishes the stopping condition.  The returned path
% will always be a subpath of the second argument.  When all subpaths of
% the second argument have been visited, the empty string '' will be
% returned.  If the second argument is omitted, it assumes the same value
% as the first argument.
function path=walk(oldPath,varargin)
	
	if(nargin>1)
		startingPath=varargin{1};
	else
		startingPath=oldPath;
	end
	
	slash=PathSeparator();

	% First, we'll look for any subdirectories.
	subdirs=dir(oldPath);
	subdirs=subdirs(find([subdirs.isdir]));
	
	% If there are any subdirectories, return the first of them.
	if(length(subdirs)>2) % We have to skip the first two entries, "." and ".."
		path=[oldPath slash subdirs(3).name];
	else
		while(true)
			% If there are no subdirectories, we must be at a "leaf".  Time to
			% start working our way back up.
			% First, we'll split up oldPath.
			lastSlash=max(find(slash==oldPath));
			parentDir=oldPath(1:lastSlash-1);
			tail=oldPath(lastSlash+1:end);

			% Make sure we don't go too high up.
			if(isempty(strfind(parentDir,startingPath)))
				path='';  % Indicate that we're done.
				break;
			end
				
			% Find our "sibling" directories.
			sibs=dir(parentDir);
			sibs=sibs(find([sibs.isdir]));
			
			nextSib=find(strcmp(tail,{sibs.name}))+1;
			if(nextSib>length(sibs))
				% We've already visited the last subdirectory here,
				% so we'll keep going up.
				oldPath=parentDir;
			else
				% Ok, move into the next sibling directory.
				path=[parentDir slash sibs(nextSib).name];
				break;
			end
		end
	end
end % -- end of walk function --

% -- PathSeparator -- %
function ps=PathSeparator()
	% The path separater is OS-specific.  (stupid microsoft)
	if(ispc)
		ps='\';
	else
		ps='/';
	end
end % -- end of PathSeparator function --
