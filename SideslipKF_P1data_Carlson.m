% Sideslip Estimation using one antenna GPS using P1 Data
% JH 8/16/2007

clear all; clc;
close all;
% load data
postprocess('C:\MATLAB71\work\TestData\judy_2005-08-30_ab.mat'); tstart = 15; tend = 42;
% postprocess('C:\MATLAB71\work\TestData\shad_2005-08-30_af.mat'); tstart = 23; tend = 46;
names                                   % our data file that defines the names our signal data structures
p1_params                               % load vehicle parameter file for P1

% get data
T = tstart*500+1:tend*500+1;
t = t(T) - tstart;
N = length(T);
ts = Ts;                                % [s] sample time of XPC box (500 Hz)
tsGPS = 0.1;                            % [s] sample time of GPS Novatel OEM4 Unit (10 Hz)
deltaL = PostProc(T,1);                 % [rad] Left steer angle, at road wheels
deltaR = PostProc(T,2);                 % [rad] Right steer angle, at road wheels
delta = (deltaL + deltaR)/2;            % [rad] Average steer angle, at road wheels
beta = SSest(T,15);                     % [rad] Sideslip measurement from 2 antenna GPS
r = INS(T,1)/.984; %./SSest(T,2); - SSest(T,3)./SSest(T,2); %SSest(T,4);             % [rad/s] Yaw rate, corrected by scale factor & bias compensated for roll angle from 2 antenna GPS
Vx = SSest(T,9);                        % [m/s] Longitudinal velocity using wheelspeeds & GPS
alpha = atan(beta + a.*r./Vx) - delta; % [rad] Front tire slip angle from 2 antenna GPS
ay = SSest(T,14);                       % [m/s^2] Lateral acceleration, compensated for roll and accelerometer bias from GPS
ax = SSest(T,11);                       % [m/s^2] Longitudinal acceleration, compensated for accelerometer bias from GPS
ax_raw = INS(T,2);                      % [m/s^2] Longitudinal accleration from INS
ay_raw = INS(T,4);                      % [m/s^2] Lateral acceleration from INS
phi = SSest(T,5);                       % [rad] Roll Angle measured from 2 antenna GPS
% Raw OEM4 GPS Data
UTC = GPS(T,11);                        % [s] Universal Time Coodinate (i.e. GPS time) from OEM4 receiver
VGPS = GPS(T,13);                       % [m/s] Planar velocity vector from OEM4 receiver
headingGPS = unwrap(GPS(T,14)*pi/180);          % [rad] Planar heading angle w.r.t. North from OEM4 receiver
VzGPS = GPS(T,15);                      % [m/s] Vertical velocity from OEM4 receiver
VGPSsol = GPS(T,16);                    % [] Velocity Solution Type (3 is good) from OEM4 receiver
PPS = GPS(T,17);                        % [s] Pulse per second square wave from OEM4 receiver, used to sychronize GPS data with INS data
diffUTC = [0; diff(UTC)];               % [1/s] Difference vector of UTC signal, used to tell when new GPS data has come in

% plot maneuver
figure
subplot(4,1,1); plot(t,delta*180/pi); title('steer angle (deg)'); axis tight;
subplot(4,1,2); plot(t,r*180/pi); title('yaw rate (deg/s)'); axis tight;
subplot(4,1,3); plot(t,ay); title('lat acceleration (m/s^2)'); axis tight;
subplot(4,1,4); plot(t,Vx); title('SSest Velocity (m/s)'); axis tight;

% plot GPS data
figure
subplot(2,1,1); plot(t,[VGPS]); title('OEM4 velocity (m/s)'); axis tight; grid on;
subplot(2,1,2); plot(t,headingGPS*180/pi); title('OEM4 heading (deg)'); axis tight; grid on;

% find delay vector (in sample times) by comparing UTC and PPS signals, for details see GPSdelay.m
delayPPS = GPSdelay(PPS, UTC, t, ts);

% resync GPS data streams given delayPPS
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
%******************* Least Squares Estimator for Front Average Wheel Radius (Ru) using GPS ********************
% This approach was taken from Chris Carlson's Dissertation p11.  It uses least squares (LS) method to
% estimate average radius of the undriven wheels (on P1, they are the front wheels). As a by product
% of this method, front axle longitudinal stiffness is also estimated, but as Carlson discusses in
% his dissertation, using LS produces a biased estimate of longitudinal stiffness and a nonlinear
% method must be used for accurate Cx estimation.
%
% omega_u = zeros(N,1);               % angular velocity of the undriven wheels
% angle_per_tick = 2*pi/48;           % only a guess!!
% num_ticksL = Wheelspeeds(T,1);      % num of encoder ticks per Ts on front left wheel
% num_ticksR = Wheelspeeds(T,2);      % num of encoder ticks per Ts on front right wheel
% A_wheel = zeros(N,2);               % A matrix
% for k = 1:N
%     % calculate average angular velocity
%     if (k == 1 || k == N)
%         omega_u(k) = 0;
%     else
%         omega_uL = angle_per_tick*(num_ticksL(k+1) - num_ticksL(k-1))/(2*ts);
%         omega_uR = angle_per_tick*(num_ticksR(k+1) - num_ticksR(k-1))/(2*ts);
%         % take the average wheel speeds
%         omega_u(k) = mean([omega_uL omega_uR]);
%         %********** NEED A WAY OF SMOOTHING OUT OMEGA_U HERE!! **************
%     end
%     % construct A matrix
%     A_wheel(k,:) = [-1/m omega_u(k)/m/VGPS(k)];   % assume VGPS = Vx
% end
% % Apply least squares method to estimate Ru
% x_wheel = A_wheel \ ax;
% % solve for Ru
% Ru = x_wheel(2)/x_wheel(1)

% assume we know effective wheel radii of front tires (can be estimated beforehand
% at constant speed, straight ahead driving with GPS *OR* with the online method outlined above).
Rr = param.fr.r;
Rl = param.fl.r;
% get smooth estimate of angular wheel speeds from encoder ticks. Code written by Shengbo Li
[LwhlSpd,RwhlSpd]=self_CalWhlSpd(t,Wheelspeeds(T,:),Rl,ts);
omegal = LwhlSpd/Rl;
omegar = RwhlSpd/Rl;

%******************* Kalman filter to estimate sideslip angle ********************
Sgyro = 0.01;               % [rad/s] standard deviation of gyro, based on physical readings from Carlson's disseration p50
Sheadinggps = 0.2;          % [rad] standard deviation of GPS heading, based on Novatel data sheet, from Carlson's disseration p50
Svgps = 0.02;               % [m/s] standard deviation of GPS velocity, based on Novatel data sheet, from Carlson's disseration p50
Sgbias = 0.001;             % [rad/s] standard deviation of gyro bias (hand tuned), tuned to adjust rate of convergence
SVy = Svgps;                % [m/s] standard deviation of GPS lateral velocity, set to be equal to Svgps
Sabias = 0.03;              % [m/s^2] standard deviation of accelerometer bias (hand tuned), tuned to adjust rate of convergence
Sacc = 0.2;                 % [m/s^2] standard deviation of accelerometer, empirically determined from straight driving maneuver
SRadius = 1e-6;             % [m] standard deviation of wheel radius (hand tuned), taken from Carlson's dissertation p50
Sahat = 0.0005;              % [.] standard deviation of 1/(scaling factor) (hand tuned), taken from Carlson's dissertation p50: 5e-4
Sbhat = 0.005;               % [rad/s] standard deviation of (gyro bias)/(scaling factor) (hand tuned), taken from Carlson's dissertation p50: 5e-5
R_THRES = 1*pi/180;         % [rad] threshold yaw rate to determine whether estimate gyro bias or integrate yaw rate
V_THRES = 2;                % [m/s] threshold longitudinal velocity to estimate gyro bias

%%% define xg = [heading, front_right_wheel_radius, front_left_wheel_radius, 1/gyro_sf, gyro_bias/gyro_sf]' and initial covariance Pg
x0 = [headingGPS(1) Rr Rl 1 0]';
xg = zeros(5,N); xg(:,1) = x0;
Lg = zeros(5,3,N);
Rg = diag([Sheadinggps^2 Svgps^2 Sgyro^2]);
Qg = diag([Sheadinggps^2 SRadius^2 SRadius^2 Sahat^2 Sbhat^2])*ts;      % discretize process noice matrix
Pg = zeros(5,5,N);
Pg(:,:,1) = Qg;
%%% define xa = [lateral_velocity accel_bias]' and initial covariance Pa
xa = zeros(2,N);
La = zeros(2,N);
Ra = Sacc^2;
Qa = ts*eye(2)*[ts*Sacc^2 0; 0 Sabias^2]*eye(2);
Pa = zeros(2,2,N);
Pa(:,:,1) = [SVy^2 0; 0 Sabias^2];
% For accelerometer integration, define system dynamics matrices A, B, C1, C2
Actsa = [0 -1; 0 0];
Bctsa = [1; 0];
C1a = [1 0];
C2a = [0 0];
sysctsa = ss(Actsa,Bctsa,C1a,0);
% discretize matrices
sysa = c2d(sysctsa,ts);
[Aa,Ba,C1a,Da] = ssdata(sysa);
% initialize values
GPSvalid = zeros(N,1);
Bhat_ant = zeros(N,1);
B_ant = zeros(N,1);
Vy_GPS = zeros(N,1);
Vx_GPS = zeros(N,1);
yKF = zeros(N,1);
nGPS_age = zeros(N,1);
headingadd = zeros(1,N); headingadd(1) = headingGPS(1);

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
        Ca = C1a;
    else     % otherwise, just integrate estimate forward
        Ca = C2a;
    end
    La(:,k) = Pa(:,:,k)*Ca'*inv(Ca*Pa(:,:,k)*Ca' + Ra);
    xa(:,k) = xa(:,k) + La(:,k)*(Vy_GPS(k) - Ca*xa(:,k));
    Pa(:,:,k) = (eye(2) - La(:,k)*Ca)*Pa(:,:,k);

    % LATERAL VELOCITY TIME UPDATE
    % calculate derivative of estimate from state equation
    if k == 1
        xadot_k = [0 0]';
    else
        xadot_k = Actsa*xa(:,k-1) + Bctsa*(ay_raw(k-1) - r(k-1)*Vx_GPS(k-1));
    end
    xadot_k1 = Actsa*xa(:,k) + Bctsa*(ay_raw(k) - r(k)*Vx_GPS(k));
    xa(:,k+1) = xa(:,k) + ts/2*(xadot_k + xadot_k1); %ts*(xdot_k1);
    Pa(:,:,k+1) = Aa*Pa(:,:,k)*Aa' + Qa;
    Bhat_ant(k+1) = asin(xa(1,k+1)/VGPS(k+1));

    % HEADING MEASUREMENT UPDATE
    % if going straight & moving, heading is valid, use GPS heading & wheelspeed sensors to estimate
    % gyro bias & heading
    if ((abs(VGPS(k) > V_THRES)) && (GPSvalid(k) == 1)) % && (diffUTC(k) ~= 0)); % (abs(r(k)) < R_THRES) &
        Cg = [1 0 0 0 0;
            0 0.5*omegar(k) 0.5*omegal(k) 0 0;
            0 omegar(k)/(tw_m*cos(deltaR(k))) -omegal(k)/(tw_m*cos(deltaL(k))) 0 0];
        Cgcheck(k) = 1;
    % if GPS is unavailable or speed is too low, only use yaw rate as a measurement
    else %(abs(VGPS(k) < V_THRES)) || (GPSvalid(k) ~= 1)
        Cg = [zeros(2,5);
            0 omegar(k)/(tw_m*cos(deltaR(k))) -omegal(k)/(tw_m*cos(deltaL(k))) 0 0];
        Cgcheck(k) = 2;
%     % if turning
%     else
%         Cg = [1 0 0 0 0;
%             0 0.5*omegar(k) 0.5*omegal(k) 0 0;
%             zeros(1,5)];
%         Cgcheck(k) = 3;
    end
    yg = [headingGPS(k) VGPS(k) r(k)]';            % neg sign on r is to correct for GPS sign convention that clockwise (RH) is positive
    Lg(:,:,k) = Pg(:,:,k)*Cg'*inv(Cg*Pg(:,:,k)*Cg' + Rg);
    xg(:,k) = xg(:,k) + Lg(:,:,k)*(yg - Cg*xg(:,k)); %-nGPS_age(k)));
    Pg(:,:,k) = (eye(5) - Lg(:,:,k)*Cg)*Pg(:,:,k);

    % HEADING TIME UPDATE
    % define system dynamics matrices Ag, Bg
    Actsg = [0 0 0 -r(k) -1; zeros(4,5)];
    Ag = [0 0 0 -r(k)*ts -ts; zeros(4,5)] + eye(5);
    % calculate derivative of estimate from state equation
    if k == 1
        xgdot_k = zeros(5,1);
    else
        xgdot_k = Actsg*xg(:,k-1);
    end
    xgdot_k1 = Actsg*xg(:,k);
    xg(:,k+1) = xg(:,k) + ts/2*(xgdot_k + xgdot_k1); %ts*(xgdot_k1);
    Pg(:,:,k+1) = Ag*Pg(:,:,k)*Ag' + Qg;
    % try adding heading directly
    headingadd(:,k+1) = headingadd(:,k) + ts*(-r(k)*xg(4,k)+xg(5,k));
end
% plot heading estimate
figure
plot(t, [xg(1,:)' headingGPS headingadd']*180/pi); title('Heading Estimate'); legend('estimated \psi','OEM4 heading') %
grid on
axis tight

% calculate the front tire slip angle, assuming beta @ the antenna = beta @ the CG
alphahat = atan(Bhat_ant + a.*r./VGPS) - delta;
% find gyro bias & sf
gyro_sf = 1./xg(4,:)';
gyro_bias = xg(5,:)'.*gyro_sf;

% plot gyro bias & sf estimate
figure
subplot(2,1,1)
plot(t, [gyro_bias SSest(T,3)]); title('Gyro Bias Estimate');   legend('estimated bias','SSest bias')
grid on
axis tight
subplot(2,1,2)
plot(t, gyro_sf); title('Gyro Scale Factor Estimate');
grid on
axis tight
% plot sideslip estimate
figure
plot(t, [Bhat_ant beta]*180/pi); legend('OEM4 Sideslip at GPS Antenna (deg)','Beeline Sideslip at CG (deg)','OEM4 Sideslip at new GPS update')
grid on
axis tight
% % plot tire slip estimate
% figure
% plot(t, [alphahat alpha]*180/pi ); legend('OEM4 front slip angle (deg)','Beeline front slip angle (deg)')
% title('Front Slip Angle Comparison')
% grid on
% axis tight
% % plot wheel radii estimates
% figure
% plot(t, [xg(2,:); xg(3,:)]'); title('Front Wheel Radii Estimates'); legend('right','left');
% grid on
% axis tight
% % plot accelerometer bias estimate
% figure
% plot(t, [xa(2,:)']); title('Accel Bias Estimate');  %SSest(T,3) legend('estimated bias','SSest bias')
% grid on
% axis tight

% % plot GPS status
% figure
% plot(t, GPSvalid); title('GPS status, 1=invalid, 0=valid')



