% Automatic flipper.
% Since the P1 code uses UNIX-style line endings, it is necessary to run
% this before each revision to SVN.  This file automatically runs flip.exe
% to first check whether or not the file needs flipping, then flip
% each that is necessary.  Runs for each mdl file in the current directory.
% Note that this ensures that mdl files that weren't updated are left alone
% and therefore not dumped into the repository as new when no actual
% changes took place.
%
% By Shad "I need to graduate" Laws, 6/2/09.  Version 1.1

close all;
clear all;

mdlfiles = dir('*.mdl');

for ii = 1:length(mdlfiles);
    % run flip to check the ending type
    [out_a,out_b] = system(['flip -t ' mdlfiles(ii).name]);
    % check the result... it should be UNIX
    if out_b(end-4:end) ~= 'UNIX ';
        % it isn't unix, so flip it.
        disp(['Flipping ' mdlfiles(ii).name]);
        system(['flip -u ' mdlfiles(ii).name]);
    else;
        % it's fine - leave it alone.
        disp([mdlfiles(ii).name ' appears to be up-to-date.']);
    end;
end;

return;


