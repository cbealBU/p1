% Script to call all plots or 

% Ask user for file name
query = input('Enter P1 file name with .mat extension or leave empty to use current workspace variables: ','s');
% If user hits return without entering file name, then it's assumed that 
% the file is already loaded into the workspace

if ~isempty(query)
    load(query)
end

plotsToDisp = input(['Enter the number corresponding to the plot you want to display.\nSeparate multiple plot inputs by a comma.\n' ...
    '1: Control Panels\n2: Driver Input\n3: Steering\n4: Motor\n5: IMU\n6: Wheel Force Transducers\n7: GPS\n'],'s');

cleanedInput = erase(plotsToDisp," ");
c = strsplit(cleanedInput,',');

if isempty(plotsToDisp)
    plotControlPanel;
    plotDriverInput;
    plotSteering;
    plotMotors
    plotIMU;
    plotWFTs;
    plotGPS;
else
    for k = 1:length(c)
        if c{k} == '1'
            plotControlPanel;
        elseif c{k} == '2'
            plotDriverInput;
        elseif c{k} == '3'
            plotSteering;
        elseif c{k} == '4'
            plotMotors;
        elseif c{k} == '5'
            plotIMU;
        elseif c{k} == '6'
            plotWFTs;
        elseif c{k} == '7'
            plotGPS;
        else
            disp(['NOTE: The number ' c{k} ' does not correspond to any value on the list.'])
        end  
    end
end

% Clean up
clear query plotsToDisp cleanedInput c