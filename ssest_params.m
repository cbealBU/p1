 % Attitude Filter

%   Offsets/biases ADDED to raw Beeline signals (deg)
    gpsins.roll_angle_offset = -0.4925;
    gpsins.yaw_angle_offset = 90.6;    % nominal is 90, not zero!
%     gpsins.yaw_angle_offset = 91.2;    % nominal is 90, not zero!
%   Offsets/biases ADDED to raw INS signals (deg/s)
    gpsins.roll_rate_bias = -0.2179;
    gpsins.yaw_rate_bias = -0.2222;
%   Scale factors that multiply unbiased INS signals
    gpsins.roll_rate_sf = 1;   % this shouldn't be changed
%   Cross-couplings SUBTRACTED from unbiased INS signals
    gpsins.roll_to_yaw = -0.05;
    gpsins.yaw_to_roll = -0.0085;

% Velocity Filter
%   Biases ADDED to raw INS signals (deg/s)
    gpsins.long_acc_bias = 0.3492;
    gpsins.lat_acc_bias = 0.3240;
%   Scale factors that multiply unbiased INS signals
    gpsins.long_acc_sf = 0.97;
    gpsins.lat_acc_sf = 1.06;
%   Cross-couplings SUBTRACTED from unbiased INS signals
    gpsins.long_to_lat = 0.00;
    gpsins.lat_to_long = 0.00;

% Kalman filter initialization parameters
%   Velocity filter initialization
    gpsins.init_state_vel = [0, 0]; % P42
    gpsins.init_cov_vel = zeros(1,4*4); % P43
%   Heading filter initialization
    gpsins.init_state_hdg = [1, 0]; % P42
    gpsins.init_cov_hdg = zeros(1,3*3); % P43
%   Roll filter initialization
    gpsins.init_state_roll = [0]; % P66
    gpsins.init_cov_roll = zeros(1,2*2); % P67
%Position filter initialization. ENTER INITIAL POSITIONS HERE
%     gpsins.initPosE = 54552; %Initial East Position (meters from reference)
%     gpsins.initPosN = 34233; %Initial North Position (meters from reference)
    gpsins.initPosE = 9378; %Initial East Position (meters from reference)
    gpsins.initPosN = -474; %Initial North Position (meters from reference)
        
%GPS Delay Parameters
    gpsins.splitbeelinedelay = 0.200/Ts; %Beeline delays above this value trigger split integration
    % online values
    gpsins.maxoem4delay = 0.080/Ts; %Cutoff delay for OEM4 GPS. Delays above this value flagged with -2
    gpsins.maxbeelinedelay = 0.800/Ts; %Cutoff delay for Beeline GPS. Delays above this value flagged with -2
    % offline values
    gpsins.maxoem4delayoffline = 0.150/Ts; %Cutoff delay for OEM4 GPS. Delays above this value flagged with -2
    gpsins.maxbeelinedelayoffline = 0.800/Ts; %Cutoff delay for Beeline GPS. Delays above this value flagged with -2

%Sensor Variances and Time Constants
    %Velocity Filter
    gpsins.velfilter.sens1_var = 7.1249e-4; %ax measurements variance
    gpsins.velfilter.sens2_var = 4.4450e-4; %ay measurements variance
    gpsins.velfilter.timeconst1 = 1.0e-1; %filter time constant for ax bias   
    gpsins.velfilter.timeconst2 = 1.0e-1; %filter time constant for ay bias
    gpsins.velfilter.msr1_var = 0.05; %GPS Measurements variance
    gpsins.velfilter.msr2_var = 0.05; %GPS Measurements variance
    %Roll Filter
    gpsins.rollfilter.sens_var = 0.0130; %gyro measurements variance
    gpsins.rollfilter.timeconst = 1.0e-4; %filter time constant for bias
    gpsins.rollfilter.msr_var = 0.2; %GPS measurement variance
    %Heading/Yaw Filter
    gpsins.hdgfilter.sens_var = 0.015; %gyro measurements variance
    gpsins.hdgfilter.timeconst1 = 1.0e-6; %filter time constant for sensitivity
    gpsins.hdgfilter.timeconst2 = 1.0e-4; %filter time cosntant for bias / sensitivity
    gpsins.hdgfilter.msr_var = 0.2; %GPS measurements variance
    gpsins.hdgfilter.init_sf_var = 0.01; %initial sensitivity (scale factor) variance
        
%GPS Differential Cutoffs - Maximum allowable differentials between GPS updates and integrated results
    gpsins.diff_roll = 3; %between GPS roll angle and integrated roll angle
    gpsins.diff_yaw = 3; %between GPS yaw angle and integrated yaw angle
    gpsins.diff_vel_x = 2; %between GPS Vx and integrated Vx
    gpsins.diff_vel_y = 2; %between GPS Vy and integrated Vy
   
%Buffer length Paramters
%    gpsins.buff_length = 5.0/Ts; %Maximum possible time (in sample times) to be stored in any given buffer
    
%GPS Sensor Position Information (for velfilter_acc.c)
    gpsins.dist_sensor = 0.01; %distance from CG to sensors
    gpsins.height_sensor = 0.1; %height from roll center to sensors
    gpsins.h_gps_ant = 1.04; %height from sensors to GPS antenna
    gpsins.l_gps_ant = -0.37; %distance from sensors to GPS antenna
    
%Other filter specific parameters
    %velfilter_acc.c
    gpsins.num_grade_avg = 0.5/Ts; %how long average grade when slow
    
    
