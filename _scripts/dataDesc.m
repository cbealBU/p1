% Adding descriptions to data file

data = struct([]);

%% Only for testing with RP4
if ~exist('y','var')
    error("Variable 'y' does not exist")
end

% t = y(1,:);
% 
% ii=1;
% data(ii).name='01 Gyroscope';
% data(ii).y=y(2:4,:);
% data(ii).size=3;
% data(ii).units={'dps','dps','dps'};
% data(ii).desc='Contains in order: Roll rate(1), pitch rate(2), yaw rate(3).';
% 
% ii=2;
% data(ii).name='02 Accelerometer';
% data(ii).y=y(5:7,:);
% data(ii).size=3;
% data(ii).units={'g','g','g'};
% data(ii).desc='Contains in order: ax(1), ay(2), az(3).';
% 
% ii=3;
% data(ii).name='03 Atmosphere';
% data(ii).y=y(8:10,:);
% data(ii).size=3;
% data(ii).units={'hPa','deg C','%'};
% data(ii).desc='Contains in order: pressure, temperature, humidity.';

%% For use with FlexCase and P1 data
ii = 1;
data(ii).name = '01 Control Panel';
data(ii).y = rt_ControlPanel;
data(ii).size = length(rt_ControlPanel(1,:));
data(ii).units = {'rad' 'rad' 'amps' 'amps' 'volts' 'volts' 'rad' 'rad' 'ticks' 'ticks' 'volts' 'volts'};
data(ii).desc = 'Contains in order: ';

ii = 2;
data(ii).name = '02 Driver Input';
data(ii).y = rt_DriverInput;
data(ii).size = length(rt_DriverInput(1,:));
data(ii).units = {};
data(ii).desc = 'Contains in oder: ';

ii = 3;
data(ii).name = '03 Ignition';
data(ii).y = rt_Ignition;
data(ii).size = length(rt_Ignition(1,:));
data(ii).units = {};
data(ii).desc = 'Contains in oder: ';

ii = 4;
data(ii).name = '04 IMU';
data(ii).y = rt_IMU;
data(ii).size = length(rt_IMU(1,:));
data(ii).units = {};
data(ii).desc = 'Contains in oder: ';

ii = 5;
data(ii).name = '05 ';
data(ii).y = rt_DriverInput;
data(ii).size = length(rt_DriverInput(1,:));
data(ii).units = {};
data(ii).desc = 'Contains in oder: ';










% 
% data(ii).name = '01 Steering';
% data(ii).y = y(2:13,:);
% data(ii).size = 12;
% data(ii).units = {'rad' 'rad' 'amps' 'amps' 'volts' 'volts' 'rad' 'rad' 'ticks' 'ticks' 'volts' 'volts'};
% data(ii).desc = 'Contains in order: L&R motor shaft angles, L&R motor currents, L&R motor voltages, L&R gearbox shaft angles, raw L&R encoder values, and raw L&R pot values';
% 
% ii = 2;
% data(ii).name = '02 INS';
% data(ii).y = y(14:19,:);
% data(ii).size = 6;
% data(ii).units = {'rad/s' 'm/s^ii' 'deg/s' 'm/s^ii' 'deg/s' 'm/s^ii'};
% data(ii).desc = 'Contains in order: Yaw rate, ax, roll rate, ay, pitch rate, & az';
% 
% ii = 3;
% data(ii).name = '03 GPS';
% data(ii).y = y(20:47,:);
% data(ii).size = 28;
% data(ii).units = {'na','s','na','deg','deg','m','m','m','m','m/s','m/s','deg','deg','deg','na','na','na','s','deg','deg','m','m','m^2','m^2','m^2','m^2','m^2','m^2'};
% data(ii).desc = 'PPS, time of week, week num, Latitude, Longitude, Altitude, ECEF X, ECEF Y, ECEF Z, horizontal speed, vert velocity, course over ground, heading, roll angle, nav mode, attitude status, sats used, differential age, yaw stddev, roll stddev, horizontal rms, vertical rms, covariance N-N, covariance N-E, covariance N-U, covariance E-E, covariance E-U, covariance U-U';
% 
% ii = 4;
% data(ii).name = '04 Handwheel';
% data(ii).y = y(49:51,:);
% data(ii).size = 3;
% data(ii).units = {'rad' 'rad' 'rad'};
% data(ii).desc = 'Contains three separate handwheel angle measurements.';

clear ii