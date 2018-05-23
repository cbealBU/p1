function DataDescription=datadesc;
% datadesc.m
%
% This file creates a description of the data that will get logged by
% the nissan_steer.mdl model.  This file is automatically run when the model is
% opened.  The variable DataDescription should be stored along with the
% data when it is saved, as is done automatically by the script savedata.m.
%
% This makes it possible for post-processing scripts to automatically know
% what signals are in a particular data file and how to use them.  It also
% helps humans remember what's what after a couple of months go by.
%


DataDescription = struct([]);

ii=1;
DataDescription(ii).name='Steering';
DataDescription(ii).size=12;
DataDescription(ii).units={'rad' 'rad' 'amps' 'amps' 'volts' 'volts' 'rad' 'rad' 'ticks' 'ticks' 'volts' 'volts'};
DataDescription(ii).desc='Contains in order: L&R motor shaft angles, L&R motor currents, L&R motor voltages, L&R gearbox shaft angles, raw L&R encoder values, and raw L&R pot values';

ii=2;
DataDescription(ii).name='INS';
DataDescription(ii).size=6;
DataDescription(ii).units={'rad/s' 'm/s^ii' 'deg/s' 'm/s^ii' 'deg/s' 'm/s^ii'};
DataDescription(ii).desc='Contains in order: Yaw rate, ax, roll rate, ay, pitch rate, & az';

ii=3;
DataDescription(ii).name='GPS';
DataDescription(ii).size=25;
DataDescription(ii).units={'na','s','na','deg','deg','m','m/s','m/s','deg','deg','deg','na','na','na','s','deg','deg','m','m','m^2','m^2','m^2','m^2','m^2','m^2'};
DataDescription(ii).desc='PPS, time of week, week num, latitude, longitude, altitude, horizontal speed, vert velocity, course over ground, heading, roll angle, nav mode, attitude status, sats used, differential age, yaw stddev, roll stddev, horizontal rms, vertical rms, covariance N-N, covariance N-E, covariance N-U, covariance E-E, covariance E-U, covariance U-U';

ii=4;
DataDescription(ii).name='Handwheel';
DataDescription(ii).size=3;
DataDescription(ii).units={ 'rad' 'rad' 'rad'};
DataDescription(ii).desc='Contains three separate handwheel angle measurements.';

ii=5;
DataDescription(ii).name='Accelerator';
DataDescription(ii).size=1;
DataDescription(ii).units={ 'V' };
DataDescription(ii).desc='Contains the raw accelerator pedal signal.';

ii=6;
DataDescription(ii).name='Wheelspeeds';
DataDescription(ii).size=4;
DataDescription(ii).units={ 'ticks/sample' 'ticks/sample' 'ticks/sample' 'ticks/sample'};
DataDescription(ii).desc='Contains wheelspeed in order data for FR, FL, RR, and RL wheels.';

ii=7;
DataDescription(ii).name='Commands';
DataDescription(ii).size=6;
DataDescription(ii).units={ 'A' 'A' 'A' 'A' 'A' 'rad'};
DataDescription(ii).desc='Contains L&R steering motor current commands, L&R traction motor commands, & handwheel motor command, HAL command';

ii=8;
DataDescription(ii).name='SSest';
DataDescription(ii).size=21;
DataDescription(ii).units={'rad' 'unitless' 'rad/s' 'rad/s' 'rad' 'unitless' 'rad/s' 'rad/s' 'm/s' 'm/s^2' 'm/s^2' 'm/s' 'm/s^2' 'm/s^2' 'rad' 'm' 'm' 'time steps' 'time steps' 'unitless' 'unitless'};
DataDescription(ii).desc= 'Contains in order: 1) YAW angle, gyro sf, gyro bias, 4) rate; 5) ROLL angle, gyro sf, gyro bias, 8) rate; 9) LONGITUDINAL velocity, accel bias, 11) accel at CG; LATERAL velocity at CG, accel bias, 14) accel at CG; 15) SIDESLIP ANGLE; POSITION east (x), north (y); DELAY vel/pos, attitude; UPDATE vel/pos, attitude';

ii=9;
DataDescription(ii).name='Switches';
DataDescription(ii).size=4;
DataDescription(ii).units={'bool' 'bool' 'bool' 'bool'};
DataDescription(ii).desc='Contains status of fault switch, HAL switch, cruise control switch, and brake pedal switch.';

ii=10;
DataDescription(ii).name='Load Cells';
DataDescription(ii).size=4;
DataDescription(ii).units={ 'N' 'N' 'N' 'N' };
DataDescription(ii).desc='Contains front L&R tie rod load cell readings, rear L&R tie rod load cell readings.';

ii=11;
DataDescription(ii).name='Vehicle State';
DataDescription(ii).size=7;
DataDescription(ii).units={'rad/s' 'rad' 'm/s' 'rad' 'rad/s' 'rad/s'};
DataDescription(ii).desc='Contains NON-GPS-BASED vehicle yaw rate, sideslip, horizontal speed, rear wheel speeds';

ii=12;
DataDescription(ii).name='PostProc';
DataDescription(ii).size=5;
DataDescription(ii).units={'rad' 'rad' 'rad' 'N-m' 'N-m'};
DataDescription(ii).desc='Contains in order: left & right steer angles, ackermann angle, left & right steer torques measured by load cells';

ii=13;
DataDescription(ii).name='Tire Estimation';
DataDescription(ii).size=12;
DataDescription(ii).units={'rad' 'rad' 'unitless' 'na' 'na' 'na' 'na' 'na' 'm' 'm' 'N' 'N'};
DataDescription(ii).desc='Contains in order: front slip angle, rear slip angle, tire-road friction coefficient, estimation flags, left & right pneumatic trail, left & right peak forces';

ii=14;
DataDescription(ii).name='Steering Torques';
DataDescription(ii).size=10;
DataDescription(ii).units={'Nm' 'Nm' 'Nm' 'Nm' 'Nm' 'Nm' 'Nm' 'Nm' 'Nm' 'Nm'};
DataDescription(ii).desc='Contains in order: left & right aligning, Dalembert, Coulomb friction, damping, and jacking torques';

% % uncomment for SSC controller (hack -- fix this soon) CGB
% ii = 14;
% DataDescription(ii).name='Controller Debug';
% DataDescription(ii).size=1;
% DataDescription(ii).units={'unitless'};
% DataDescription(ii).desc='Contains in order: empty';
% % DataDescription(ii).name='Controller Debug';
% % DataDescription(ii).size=9;
% % DataDescription(ii).units={'rad' 'rad/s' 'rad/s' 'rad' 'rad/s/s' 'rad/s' 'N' 'N' 'unitless'};
% % DataDescription(ii).desc='Contains in order: desired steer angle, S, nominal yaw rate, nominal sideslip, rnom_dot, Bnom_dot, Fyf desired, Fyr, Enable Switch';
% 
