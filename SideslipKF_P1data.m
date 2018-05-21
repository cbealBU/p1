% Sideslip Estimation using one antenna GPS using P1 Data
% JH 8/16/2007

clear all; clc; close all;

% ************ First, choose which data set to look at (comment one out): *******************
% load ramp steer maneuver data
postprocess('C:\MATLAB71\work\TestData\judy_2005-08-30_ab.mat'); tstart = 15; tend = 42;   
% load slalom maneuver data
% postprocess('C:\MATLAB71\work\TestData\shad_2005-08-30_af.mat'); tstart = 23; tend = 46;
names                                   % our data file that defines the names our signal data structures
p1_params                               % our vehicle parameter file for P1

% *********************** Get data ******************************
% All of the data streams in this section are needed for the estimation method.  To implement this on another
% vehicle, this is the section you would change to get this information from your data structure:
T = tstart*500+1:tend*500+1;
t = t(T) - tstart;
N = length(T);
ts = Ts;                                % [s] sample time of XPC box (500 Hz)
tsGPS = 0.1;                            % [s] sample time of GPS Novatel OEM4 Unit (10 Hz)
deltaL = PostProc(T,1);                 % [rad] Left steer angle, at road wheels 
deltaR = PostProc(T,2);                 % [rad] Right steer angle, at road wheels 
delta = (deltaL + deltaR)/2;            % [rad] Average steer angle, at road wheels 
beta = SSest(T,15);                     % [rad] Sideslip measurement from 2 antenna GPS
r = INS(T,1)/.984;                      % [rad/s] Yaw rate, corrected by scale factor
Vx = SSest(T,9);                        % [m/s] Longitudinal velocity using wheelspeeds & GPS
alpha = atan(beta + a.*r./Vx) - delta;  % [rad] Front tire slip angle from 2 antenna GPS
ay = SSest(T,14);                       % [m/s^2] Lateral acceleration, compensated for roll and accelerometer bias from GPS
ax = SSest(T,11);                       % [m/s^2] Longitudinal acceleration, compensated for accelerometer bias from GPS
ax_raw = INS(T,2);                      % [m/s^2] Longitudinal accleration from INS
ay_raw = INS(T,4);                      % [m/s^2] Lateral acceleration from INS
phi = SSest(T,5);                       % [rad] Roll Angle measured from 2 antenna GPS
% Raw OEM4 GPS Data
UTC = GPS(T,11);                        % [s] Universal Time Coodinate (i.e. GPS time) from OEM4 receiver
VGPS = GPS(T,13);                       % [m/s] Planar velocity vector from OEM4 receiver
headingGPS = unwrap(GPS(T,14)*pi/180);  % [rad] Planar heading angle w.r.t. North from OEM4 receiver
VzGPS = GPS(T,15);                      % [m/s] Vertical velocity from OEM4 receiver
VGPSsol = GPS(T,16);                    % [] Velocity Solution Type (3 is good) from OEM4 receiver
PPS = GPS(T,17);                        % [s] Pulse per second square wave from OEM4 receiver, used to sychronize GPS data with INS data
diffUTC = [0; diff(UTC)];               % [1/s] Difference vector of UTC signal, used to tell when new GPS data has come in

%*************************** Plot Maneuver *************************************************
% If you want to look at your experimental maneuver & GPS data, you can uncomment this section out:           
% figure
% subplot(4,1,1); plot(t,delta*180/pi); title('steer angle (deg)'); axis tight;
% subplot(4,1,2); plot(t,r*180/pi); title('yaw rate (deg/s)'); axis tight;
% subplot(4,1,3); plot(t,ay); title('lat acceleration (m/s^2)'); axis tight;
% subplot(4,1,4); plot(t,Vx); title('SSest Velocity (m/s)'); axis tight;
% figure
% subplot(2,1,1); plot(t,[VGPS]); title('OEM4 velocity (m/s)'); axis tight; grid on;
% subplot(2,1,2); plot(t,headingGPS*180/pi); title('OEM4 heading (deg)'); axis tight; grid on;
%****************************Sychronize GPS Data *******************************************
% Find delay vector (in sample times) by comparing UTC and PPS signals, for details see GPSdelay.m
delayPPS = GPSdelay(PPS, UTC, t, ts);
% Postprocess all GPS data in order to synchronize them given delayPPS
firstdelay = 0;
halfSampleDelayGPS = tsGPS/ts/2;
for k = 1:N
    % find first meaningful delay value
    if ((delayPPS(k)>0) && (firstdelay==0))
        firstdelay = delayPPS(k);
    end
    % start after firstdelay is identified
    if ((firstdelay>0) && (k>firstdelay) && (k+delayPPS(k)+halfSampleDelayGPS<N))
        delayIndex = round(delayPPS(k)) + halfSampleDelayGPS;   % record delay in sample times, take into account 1/2 sample time latency 
                                                                % in GPS velocity measurement 
    else
        delayIndex = 0;                                         % there is no delay (this part of data the delay is unknown)
    end
    delayIndex = round(delayIndex);
    % assign GPS data value back to when it was valid
    VGPS(k) = VGPS(k+delayIndex);
    headingGPS(k) = headingGPS(k+delayIndex);
    VGPSsol(k) = VGPSsol(k+delayIndex);
    UTC(k) = UTC(k+delayIndex);
end   
%******************* Kalman filter to estimate sideslip angle *****************************
Sgyro = 0.01;               % [rad/s] standard deviation of gyro, based on physical readings from Carlson's disseration p50
Sheadinggps = 0.2;          % [rad] standard deviation of GPS heading, based on Novatel data sheet, from Carlson's disseration p50
Svgps = 0.02;               % [m/s] standard deviation of GPS velocity, based on Novatel data sheet, from Carlson's disseration p50
Sgbias = 0.001;             % [rad/s] standard deviation of gyro bias, tuned to adjust gyro bias estimate rate of convergence
SVy = Svgps;                % [m/s] standard deviation of GPS lateral velocity, set to be equal to Svgps
Sabias = 0.03;              % [m/s^2] standard deviation of accelerometer bias, tuned to adjust accelerometer bias estimate rate of convergence
Sacc = 0.2;                 % [m/s^2] standard deviation of accelerometer, empirically determined from straight driving maneuver  
R_THRES = 1*pi/180;         % [rad] threshold yaw rate to determine whether estimate gyro bias or integrate yaw rate 
V_THRES = 2;                % [m/s] threshold longitudinal velocity to estimate gyro bias 

% define xg = [heading gyro_bias]' and initial covariance Pg
x0 = [headingGPS(1) 0]';
xg = zeros(2,N); xg(:,1) = x0;
Rg = Sgyro^2;
Qg = ts*eye(2)*[ts*Sgyro^2 0; 0 Sgbias^2]*eye(2);
Pg = zeros(2,2,N);
Pg(:,:,1) = [Sheadinggps^2 0; 0 Sgbias^2];
Lg = zeros(2,N);

% define xa = [lateral_velocity accel_bias]' and initial covariance Pa
xa = zeros(2,N);
La = zeros(2,N);
Ra = Sacc^2;
Qa = ts*eye(2)*[ts*Sacc^2 0; 0 Sabias^2]*eye(2);
Pa = zeros(2,2,N);
Pa(:,:,1) = [SVy^2 0; 0 Sabias^2];

% define system dynamics matrices A, B, C1, C2 (they are the same for the accelerometer & gyro state
% equations)
Acts = [0 -1; 0 0];
Bcts = [1; 0];
C1 = [1 0];
C2 = [0 0];
syscts = ss(Acts,Bcts,C1,0);
% discretize matrices
sys = c2d(syscts,ts);
[A,B,C1,D] = ssdata(sys);
% initialize values
GPSvalid = zeros(N,1);
Bhat_ant = zeros(N,1);
B_ant = zeros(N,1);
Vy_GPS = zeros(N,1);
Vx_GPS = zeros(N,1);
yKF = zeros(N,1);
nGPS_age = zeros(N,1);
for k = 1:N-1
    % Check if GPS measurement is valid 
    GPSvalid(k) = (VGPSsol(k) == 3);
    
    % find number of samples since a new GPS update was received
    if k > 50
        if (diffUTC(k) ~= 0)
            nGPS_age(k) = 0;
        else
            nGPS_age(k) = nGPS_age(k-1)+1;
        end
    end
    % calculate sideslip angle at GPS antenna
    B_ant(k) = xg(1,k) - headingGPS(k);
    % find lateral/longitudinal velocity components of GPS measured velocity
    Vy_GPS(k) = VGPS(k)*sin(B_ant(k));
    Vx_GPS(k) = VGPS(k)*cos(B_ant(k));
    
    % LATERAL VELOCITY MEASUREMENT UPDATE
    % if GPS Velocity measurement is available, GPS is valid, and vehicle is moving, use GPS to estimate accelerometer bias (use C1);
    if ((abs(VGPS(k) > V_THRES)) & (GPSvalid(k) == 1) & (diffUTC(k) ~= 0))
        C = C1;
    else     % otherwise, just integrate estimate forward   
        C = C2;
    end
    La(:,k) = Pa(:,:,k)*C'*inv(C*Pa(:,:,k)*C' + Ra);
    xa(:,k) = xa(:,k) + La(:,k)*(Vy_GPS(k) - C*xa(:,k));
    Pa(:,:,k) = (eye(2) - La(:,k)*C)*Pa(:,:,k);
    
    % LATERAL VELOCITY TIME UPDATE
    % calculate derivative of estimate from state equation
    if k == 1
        xadot_k = [0 0]';
    else
        xadot_k = Acts*xa(:,k-1) + Bcts*(ay_raw(k-1) - r(k-1)*Vx_GPS(k-1));     
    end
    xadot_k1 = Acts*xa(:,k) + Bcts*(ay_raw(k) - r(k)*Vx_GPS(k));             
    xa(:,k+1) = xa(:,k) + ts/2*(xadot_k + xadot_k1); %ts*(xdot_k1); 
    Pa(:,:,k+1) = A*Pa(:,:,k)*A' + Qa;
    Bhat_ant(k+1) = asin(xa(1,k+1)/VGPS(k+1));
    
    % HEADING MEASUREMENT UPDATE
    % if going straight & moving, heading is valid, use GPS vel to estimate gyro bias (use C1)
    if ((abs(r(k)) < R_THRES) & (abs(VGPS(k) > V_THRES)) & (GPSvalid(k) == 1));
        C = C1;
        % headingadd(k+1) = headingadd(k);
    else           
        C = C2;
        % headingadd(k+1) = headingadd(k) - ts*x(2,k) + -ts*r(k) ;
    end
    Lg(:,k) = Pg(:,:,k)*C'*inv(C*Pg(:,:,k)*C' + Rg);
    xg(:,k) = xg(:,k) + Lg(:,k)*(headingGPS(k) - C*xg(:,k-nGPS_age(k)));     
    Pg(:,:,k) = (eye(2) - Lg(:,k)*C)*Pg(:,:,k);
    
    % HEADING TIME UPDATE
    % calculate derivative of estimate from state equation
    if k == 1
        xgdot_k = [0 0]';
    else
        xgdot_k = Acts*xg(:,k-1) + Bcts*-r(k-1);      % neg sign on r is to correct for GPS sign convention that clockwise (RH) is positive
    end
    xgdot_k1 = Acts*xg(:,k) + Bcts*-r(k);             % neg sign on r is to correct for GPS sign convention that clockwise (RH) is positive
    xg(:,k+1) = xg(:,k) + ts/2*(xgdot_k + xgdot_k1); %ts*(xdot_k1); 
    Pg(:,:,k+1) = A*Pg(:,:,k)*A' + Qg;
end
% calculate the front tire slip angle, assuming beta @ the antenna = beta @ the CG
% Note: we need steer angle measurements for this
alphahat = atan(Bhat_ant + a.*r./VGPS) - delta;
%******************* Plot the estimation results *****************************
% plot heading estimate
figure
plot(t, [xg(1,:)' headingGPS]*180/pi); title('Heading Estimate'); legend('estimated \psi','OEM4 heading') %  
grid on
axis tight
% plot gyro bias estimate
figure
plot(t, [xg(2,:)']); title('Gyro Bias Estimate');  %SSest(T,3) legend('estimated bias','SSest bias')
% hold on; plot(t,phi*.1,'r')
grid on
axis tight
% plot sideslip estimate
figure
plot(t, [Bhat_ant beta]*180/pi ); legend('Single Antenna GPS','Double Antenna GPS'); xlabel('t(s)'); ylabel('Sideslip Angle \beta (deg)')
grid on
axis tight
% plot tire slip estimate
figure
plot(t, [alphahat alpha]*180/pi ); legend('Single Antenna GPS','Double Antenna GPS'); xlabel('t(s)'); ylabel('Front Tire Slip Angle \alpha (deg)')
title('Front Slip Angle Comparison')
grid on
axis tight
% plot accelerometer bias estimate
figure
plot(t, [xa(2,:)']); title('Accel Bias Estimate');  %SSest(T,3) legend('estimated bias','SSest bias')
grid on
axis tight
% plot GPS status
figure
plot(t, GPSvalid); title('GPS status, 1=invalid, 0=valid')



