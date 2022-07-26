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

% controlPanelStatus = false; driverInputStatus = false; steeringStatus = false; 
% motorStatus = false; IMUStatus = false; WFTStatus = false; GPSStatus = false;

c = strsplit(plotsToDisp,',');

if isempty(plotsToDisp)
    controlPanelPlots;
    driverInputPlots;
    steeringPlots;
    motorPlots
    IMUPlots;
    WFTPlots;
    GPSPlots;
else
    for k = 1:length(c)
        if c{k} == '1'
            controlPanelPlots;
        elseif c{k} == '2'
            driverInputPlots;
        elseif c{k} == '3'
            steeringPlots;
        elseif c{k} == '4'
            motorPlots;
        elseif c{k} == '5'
            IMUPlots;
        elseif c{k} == '6'
            WFTPlots;
        elseif c{k} == '7'
            GPSPlots;
        else
            disp(['NOTE: The number ' c{k} ' does not correspond to any value on the list.'])
        end  
    end
end
