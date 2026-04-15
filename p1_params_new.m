% Script to define a number of constants needed in various places in the P1
% control models.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                       %
%              BASIC MODEL INFO/CONSTANTS               %
%                                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Fundamental time step of the model
Ts_MCU = 0.01;
Ts_MPU = 0.002;

p1params.ToWorkspaceSize = 500;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                       %
%            P1 SIMULINK MODEL PARAMETERS               %
%                                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Accelerator pedal mapping (pedal volts to motor torque)
p1params.input.VhighGuard =4.8;    % guard voltage to protect against a short
p1params.input.VlowGuard = 0.4;     % guard voltage to protect against an open circuit
p1params.input.VmaxAccel = 4.0;    % high saturation voltage for accelerator potentiometer
p1params.input.VmidAccel = 3.6;    % design voltage at the transition from regen to drive
p1params.input.VminAccel = 1.6;    % low saturation voltage measured from accelerator potentiometer
p1params.input.Vdeadband = 0.4;
p1params.input.TmaxAccel = 350;       % motor torque desired at max accelerator travel (in Nm)
p1params.input.TminAccel = -20;       % regen torque desired at min accelerator travel (negative, in Nm)
p1params.input.TregenBrake = -1000;    % regen torque desired on brake pedal press (negative, in Nm)
p1params.input.TmaxRev = 100;         % motor torque desired at max accelerator travel in reverse (in Nm)
p1params.input.TminRev = -320;         % regen torque desired at min accelerator travel in reverse (in Nm)
p1params.input.Tmax = 3212;            % maximum spec motor torque (in Nm) not to be exceeded
p1params.input.Tmin = -3212;           % maximum spec regen torque (in Nm) not to be exceeded
p1params.input.handwheelMidpoint = 222; % Midpoint of the handwheel potentiometer (cts)
p1params.input.handwheelScaling = 250; % scale factor for handwheel potentiometer (rev/ct)
p1params.input.handwheelOffset = 0; % scale factor for handwheel potentiometer (rev/ct)

p1params.input.VAccelFc = 1;        % Accelerator pedal filter cut off frequency (in Hz)
% The following two lines are the result of using the MATLAB butterworth and IIR filter
% design tools with the cutoff above:
% N = 6 (for a 6th order Butterworth)
% wc = p1params.input.VAccelFc*(2*Ts); (set up the normalized filter frequency)
% [b,a] = butter(N,wc); (design using the specified normalized cutoff frequency)
p1params.input.VAccelFiltNum = [8.5316e-10
    5.119e-09
   1.2797e-08
   1.7063e-08
   1.2797e-08
    5.119e-09
   8.5316e-10]';
p1params.input.VAccelFiltDen = [      1
      -5.7572
       13.816
      -17.687
       12.742
      -4.8969
      0.78442]';

% Drivetrain control parameters
p1params.drivetrain.left.CANTimeOut = 1.2; % CAN time out threshold (in s)
p1params.drivetrain.left.zeroSpeedThresh = 6; % threshold for considering the vehicle to be moving slowly (m/s)
p1params.drivetrain.left.velLimREV = -900; % maximum motor speed in reverse (RPM)
p1params.drivetrain.left.velLimFWD = 5400; % maximum motor speed going forward (RPM)
p1params.drivetrain.left.torqueLimLow = -350; % maximum motor torque in regen (Nm)
p1params.drivetrain.left.torqueLimHigh = 350; % maximum motor torque in drive (Nm)
p1params.drivetrain.left.minTorqueStartDrive = -120; % minimum torque in regen allowed to start limp home drive mode (Nm)
p1params.drivetrain.left.maxTorqueStartDrive = 10; % minimum torque in motoring allowed to start limp home drive mode (Nm)
p1params.drivetrain.left.minTorqueFullDrive = -120; % minimum torque in regen allowed to start full drive mode (Nm)
p1params.drivetrain.left.maxTorqueFullDrive = 50; % minimum torque in motoring allowed to start full drive mode (Nm)
p1params.drivetrain.left.npulleys = 5.6;

p1params.drivetrain.right.CANTimeOut = 1.2; % CAN time out threshold (in s)
p1params.drivetrain.right.zeroSpeedThresh = 2; % threshold for considering the vehicle to be moving slowly (m/s)
p1params.drivetrain.right.velLimREV = -900; % maximum motor speed in reverse (RPM)
p1params.drivetrain.right.velLimFWD = 5400; % maximum motor speed going forward (RPM)
p1params.drivetrain.right.torqueLimLow = -350; % maximum motor torque in regen (Nm)
p1params.drivetrain.right.torqueLimHigh = 350; % maximum motor torque in drive (Nm)
p1params.drivetrain.right.minTorqueStartDrive = -25; % minimum torque in regen allowed to start an active drive mode (Nm)
p1params.drivetrain.right.maxTorqueStartDrive = 10; % minimum torque in regen allowed to start an active drive mode (Nm)
p1params.drivetrain.right.minTorqueFullDrive = -25; % minimum torque in regen allowed to start an active drive mode (Nm)
p1params.drivetrain.right.maxTorqueFullDrive = 50; % minimum torque in regen allowed to start an active drive mode (Nm)
p1params.drivetrain.right.npulleys = 5.6;

% Steering control parameters
p1params.steering.sleepTimeout = 20;
p1params.steering.sleepRateThresh = 5*pi/180;
p1params.steering.sleepSpeedThresh = 0.2;

p1params.steering.left.CANTimeOut = 0.2; % CAN time out threshold (in s)
p1params.steering.left.startupTimeOut = 30; % Time to wait for proper startup before faulting (s)
p1params.steering.left.initNumPotReadings = 50; % Number of potentiometer readings to average for startup position (#)
p1params.steering.left.maxPotSTD = 500; % Maximum standard deviation of potentiometer readings (mV)
p1params.steering.left.minPotAvgVal = 200; % Minimum average potentiometer value (mV)
p1params.steering.left.maxPotAvgVal = 4800; % Maximum average potentiometer value (mV)
p1params.steering.left.potEncSlope = -73.299; % Slope of potentiometer to encoder calibration curve (cts/mV)
p1params.steering.left.potEncInt = 212718; % Intercept of potentiometer to encoder calibration curve (cts)
p1params.steering.left.finalOffsetLowThresh = -6000;
p1params.steering.left.finalOffsetHighThresh = 24000;
p1params.steering.left.hallWidthLowThresh = 1000;
p1params.steering.left.hallWidthHighThresh = 7000;
p1params.steering.left.homeOffsetLowThresh = -8000;
p1params.steering.left.homeOffsetHighThresh = 8000;
p1params.steering.left.intOffsetThresh = 3;
p1params.steering.left.alignmentSlewRate = 50; %
p1params.steering.left.finalOffsetAlignment = -22917;

p1params.steering.right.CANTimeOut = 0.2; % CAN time out threshold (in s)
p1params.steering.right.startupTimeOut = 30; % Time to wait for proper startup before faulting (s)
p1params.steering.right.initNumPotReadings = 50; % Number of potentiometer readings to average for startup position (#)
p1params.steering.right.maxPotSTD = 500; % Maximum standard deviation of potentiometer readings (mV)
p1params.steering.right.minPotAvgVal = 200; % Minimum average potentiometer value (mV)
p1params.steering.right.maxPotAvgVal = 4800; % Maximum average potentiometer value (mV)
p1params.steering.right.potEncSlope = 130.6; % Slope of potentiometer to encoder calibration curve (cts/mV)
p1params.steering.right.potEncInt = -157771; % Intercept of potentiometer to encoder calibration curve (cts)
p1params.steering.right.finalOffsetLowThresh = -10000;
p1params.steering.right.finalOffsetHighThresh = 6000;
p1params.steering.right.hallWidthLowThresh = 1000;
p1params.steering.right.hallWidthHighThresh = 7000;
p1params.steering.right.homeOffsetLowThresh = -8000;
p1params.steering.right.homeOffsetHighThresh = 8000;
p1params.steering.right.intOffsetThresh = 3;
p1params.steering.right.alignmentSlewRate = 50; %
p1params.steering.right.finalOffsetAlignment = 3407;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                       %
%      DATA ACQUISITION/COMMUNICATION PARAMETERS        %
%                                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p1params.wft.left.FxScale = 1.220703125;
p1params.wft.left.FyScale = 0.6103515625;
p1params.wft.left.FzScale = 1.220703125;
p1params.wft.left.MxScale = 0.18310546875;
p1params.wft.left.MyScale = 0.18310546875;
p1params.wft.left.MzScale = 0.18310546875;
p1params.wft.left.VelScale = 0.06103515625;
p1params.wft.left.PosScale = 0.010986328125;
p1params.wft.left.AxScale = 0.0030517578125;
p1params.wft.left.AzScale = 0.0030517578125;
p1params.wft.right.FxScale = 1.220703125;
p1params.wft.right.FyScale = 0.6103515625;
p1params.wft.right.FzScale = 1.220703125;
p1params.wft.right.MxScale = 0.18310546875;
p1params.wft.right.MyScale = 0.18310546875;
p1params.wft.right.MzScale = 0.18310546875;
p1params.wft.right.VelScale = 0.06103515625;
p1params.wft.right.PosScale = 0.010986328125;
p1params.wft.right.AxScale = 0.0030517578125;
p1params.wft.right.AzScale = 0.0030517578125;

% Noise covariances for the IMU sensors. Estimated from two (relatively
% short) stationary data sets.
p1params.imu.AxVar = (100*0.0125)^2;
p1params.imu.AyVar = (100*0.0148)^2;
p1params.imu.AzVar = (1*0.0160)^2;
p1params.imu.GxVar = 0.093^2; %0.111^2;
p1params.imu.GyVar = 0.093^2; %0.105^2;
p1params.imu.GzVar = 0.093^2;

% GPS message reception time step
p1params.gps.timeStep = 0.01;
% Noise covariances for the GPS data.
p1params.gps.YawStd = 0.25*pi/180;  % rad
p1params.gps.RollStd = 0.5*pi/180;  % rad
p1params.gps.VhVar = 0.004;       % m (approximate horizontal RMS)

% These can be found by augmenting the Kalman filter temporarily with a
% sensitivity term and determining the converged result. Note that this
% requires substantial vehicle excitation. Default value is 1.0.
p1params.imu.AxGain = 1.0;
p1params.imu.AyGain = 1.0;
p1params.imu.AzGain = 1.0;
p1params.imu.GxGain = 1.0;
p1params.imu.GyGain = 1.0;
p1params.imu.GzGain = 1.0;
% The cross-coupling terms between Ax and Ay is found by a least-squares
% estimation process. This is detailed in Jihan Ryu's thesis.
p1params.imu.Cxy = 0;
% The vector location of the IMU relative to the GPS primary (driver side)
% antenna
p1params.imu.r_GPS = [-0.1; 0.5; 0.7];

% Covariance matrix for the yaw Kalman Filter
p1params.KF.Yaw.Rw = zeros(2,2);
p1params.KF.Yaw.Rw(1,1) = p1params.imu.GzVar;
p1params.KF.Yaw.Rv = p1params.gps.YawStd;
% Covariance matrix for the roll Kalman Filter
p1params.KF.Roll.Rw = zeros(2,2);
p1params.KF.Roll.Rw(1,1) = p1params.imu.GxVar;
p1params.KF.Roll.Rv = p1params.gps.RollStd;
% Covariance matrix for the velocity Kalman Filter
p1params.KF.Vel.Rw = zeros(4,4); 
p1params.KF.Vel.Rw(1,1) = p1params.imu.AxVar; 
p1params.KF.Vel.Rw(3,3) = p1params.imu.AyVar;
p1params.KF.Vel.Rv = zeros(2,2);
p1params.KF.Vel.Rv(1,1) = p1params.gps.VhVar;
p1params.KF.Vel.Rv(2,2) = p1params.gps.VhVar;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                       %
%             SPI DATA TRANSMISSION SETUP               %
%                                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Declare the number of bytes in each array
p1params.spi.headerBytes = 10;            % SPI header ("start" plus 3 zeros plus 2 byte counter)
p1params.spi.flexCaseBytes = 1;           % FlexCase info (ignition state, etc.)
p1params.spi.controlPanelBytes = 3;       % Switch and indicator states
p1params.spi.driverInputBytes = 21;       % Accelerator, brake, handwheel
p1params.spi.steeringBytes = 25;          % Roadwheel steering commands/states
p1params.spi.drivetrainBytes = 44;        % Drivetrain commands/states
p1params.spi.imuBytes = 12;               % Inertial measurement data
p1params.spi.wftBytes = 20;               % Wheel force transducer data
p1params.spi.gpsBytes = 128;              % GPS packet bytes
p1params.spi.mpuBytes = 21;               % MPU I/O bytes
p1params.spi.footerBytes = 5;

% Determine the number of bytes being transmitted in each direction
mcu2mpuBytes = p1params.spi.headerBytes+ p1params.spi.flexCaseBytes + ...
    p1params.spi.controlPanelBytes + p1params.spi.driverInputBytes + ...
    2*p1params.spi.steeringBytes + 2*p1params.spi.drivetrainBytes + ...
    p1params.spi.imuBytes + 2*p1params.spi.wftBytes + p1params.spi.footerBytes;
mpu2mcuBytes = p1params.spi.gpsBytes + p1params.spi.mpuBytes;

% Determine the smallest multiple of 4 bytes that can be transmitted
p1params.spi.spiBytes = 4*ceil(max(mpu2mcuBytes,mcu2mpuBytes)/4);

% Check for an overrun of the MPU time step
spiXmitTime = p1params.spi.spiBytes*8/2000000; % Update this data rate to match model
if spiXmitTime > Ts_MPU
    error('SPI message will overrun MPU time step. Adjust settings.')
end

% Determine the number of bytes transmitted over SPI each cycle, assuming
% that the MCU->MPU message will have more data than the MPU->MCU message
% (if this assumption breaks, the padding will have to be moved from the
% MPU model to the MCU model)
%p1data.spiBytes = mcu2mpuBytes;
% Determine the padding needed in the MPU->MCU message (same caveat as
% above)
%p1data.paddingBytes = mcu2mpuBytes-mpu2mcuBytes;
p1params.spi.mcuPadding = p1params.spi.spiBytes - mcu2mpuBytes;
p1params.spi.mpuPadding = p1params.spi.spiBytes - mpu2mcuBytes;

% Clean up the intermediate variables
clear mcu2mpuBytes mpu2mcuBytes

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                       %
%        MEASURED/ESTIMATED VEHICLE PARAMETERS          %
%                                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Gear ratios
%n_drive = 5.6;      % transmission drive ratio (unitless)
p1params.steering.n_handwheel = 50;   % handwheel drive ratio (unitless)
p1params.steering.n_steering = 160;   % steering drive ratio (unitless)

% Encoder CPRs (counts per revolution)
p1params.steering.cpr_handwheel = 500*4;  % 1000 CPR quadrature
p1params.steering.cpr_steering = 1000*4;   % 500 CPR quadrature

% Dimensions and mass properties
p1params.model.m = 1724.0;                 % mass (kg)
p1params.model.a = 1.35;                   % cg to front axle distance (m)
p1params.model.b = 1.15;                   % cg to rear axle distance (m)
p1params.model.d = 1.62;                   % track width (m)
p1params.model.Iz = 1300;                  % Hand-fit estimate. (kg-m^2)

% Roll properties
p1params.model.Ix = 800;                   % roll inertia (kg-m^2)
p1params.model.h_roll = 0.39;              % effective roll height, i.e. cg height minus roll center height (m)
p1params.model.b_roll = 4800;              % roll damping (N-m-s/rad)
p1params.model.k_roll = 160000;            % roll stiffness (N-m/rad)

% Cornering stiffness
p1params.model.tire.fl.Ca = 75000/2;                % front left cornering stiffness (N/rad)
p1params.model.tire.fr.Ca = 75000/2;                % front right cornering stiffness (N/rad)
p1params.model.tire.rl.Ca = 135000/2;               % rear left cornering stiffness (N/rad)
p1params.model.tire.rr.Ca = 135000/2;               % rear right cornering stiffness (N/rad)

% Tire effective rolling radii
p1params.model.tire.fl.re = 0.161*2;               % tire effective rolling radius (m)
p1params.model.tire.fr.re = 0.161*2;               % tire effective rolling radius (m)
p1params.model.tire.rl.re = 0.3085;                % tire effective rolling radius (m)
p1params.model.tire.rr.re = 0.3085;                % tire effective rolling radius (m)

% Tire relaxation length estimates
p1params.model.tire.fl.rl = 0.3;                   % tire relaxation length (m)
p1params.model.tire.fr.rl= 0.3;                    % tire relaxation length (m)
p1params.model.tire.rl.rl = 0.55;                  % tire relaxation length (m)
p1params.model.tire.rr.rl = 0.55;                  % tire relaxation length (m)

% Tire pneumatic trail estimates
p1params.model.tire.fl.tp = 0.023;                 % pneumatic trail (m)
p1params.model.tire.fr.tp = 0.023;                 % pneumatic trail (m)
p1params.model.tire.rl.tp = 0.023;                 % pneumatic trail (m)
p1params.model.tire.rr.tp = 0.023;                 % pneumatic trail (m)


% % Create Data Bus for Simulink Model
% p1paramsInfo = Simulink.Bus.createObject(p1params);
% p1paramsBus = evalin('base', p1paramsInfo.busName);