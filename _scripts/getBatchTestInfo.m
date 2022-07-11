% Script to extract test info from a directory of P1 data files

files = dir('*.mat');
fid = fopen('test_log.txt','w');
fprintf(fid,'Test ID\t Descrip\t Max Speed\t Max Lat Acc\t Test Time\n');
for i = 1:length(files)
    load(files(i).name);
    names;
    topSpeed = max(GPS(:,10));
    maxLatAcc = max(abs(INS(:,4)));
    endTime = t(end);
    
    % Hack the newline characters out of the description field
    description = info.description;
    % Drop the prefix \n
    description = description(3:end);
    % Replace other \n's with ;'s
    description(findstr(description,'\n'))=';';
    % Write out the log
    fprintf(fid,'%s\t %s\t %2.1f\t %2.1f\t %2.1f\n',files(i).name, description, topSpeed, maxLatAcc, endTime);
end
fclose(fid);