% Script to define the data storage and transmission arrays

% Declare the number of bytes in each array
p1data.flexCaseBytes = 1;           % FlexCase info (ignition state, etc.)
p1data.controlPanelBytes = 3;       % Switch and indicator states
p1data.driverInputBytes = 21;       % Accelerator, brake, handwheel
p1data.steeringBytes = 25;          % Roadwheel steering commands/states
p1data.drivetrainBytes = 44;        % Drivetrain commands/states
p1data.imuBytes = 12;               % Inertial measurement data
p1data.wftBytes = 20;               % Wheel force transducer data
p1data.gpsBytes = 128;              % GPS packet bytes

% Determine the number of bytes being transmitted in each direction
mcu2mpuBytes = p1data.flexCaseBytes + p1data.controlPanelBytes + ...
    p1data.driverInputBytes + 2*p1data.steeringBytes + ...
    2*p1data.drivetrainBytes + p1data.imuBytes + 2*p1data.wftBytes;
mpu2mcuBytes = p1data.gpsBytes;

% Determine the number of bytes transmitted over SPI each cycle, assuming
% that the MCU->MPU message will have more data than the MPU->MCU message
% (if this assumption breaks, the padding will have to be moved from the
% MPU model to the MCU model)
p1data.spiBytes = mcu2mpuBytes;
% Determine the padding needed in the MPU->MCU message (same caveat as
% above)
p1data.paddingBytes = mcu2mpuBytes-mpu2mcuBytes;

% Clean up the intermediate variables
clear mcu2mpuBytes mpu2mcuBytes