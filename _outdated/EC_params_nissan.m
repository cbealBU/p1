% Parameters for Envelope control with differential drive
% Created by Shinichiroh in Jun 23, 2010

Ts = 0.002;             % sampling time
g=9.81;

% Air drug
AfCd=1.4406*0;
rho=1.29;

% estimator parameter
alphathres= 1;          %slip angle threshold to begin estimation (deg)
Fnf = m*g*b/(a+b);  %nominal front tire total normal force (N)
Cfls=Cfl;
Cfrs=Cfr;
Crls=Crl;
Crrs=Crr;
Pfthres = 2;
% Pfthres = 0;
Kob = 0.001;

[SATfiln,SATfild]=butter(5,40/(1/Ts/2));
% [filterb,filtera]=butter(5,3/(1/Ts/2));Ns=40;
% [filterb,filtera]=butter(5,5/(1/Ts/2));Ns=1; % current
% [filterb,filtera]=butter(2,3/(1/Ts/2));Ns=20; % best, same delay
% [filterb,filtera]=cheby1(2,0.5,3/(1/Ts/2));Ns=1; % best, half delay
% [filterb,filtera]=cheby1(2,0.5,6/(1/Ts/2));Ns=1; % best, quater delay

frontslopex=[   0    0.024299    0.1944    0.6561    1.5551    3.0374    5.2486    8.3345   12.4411   17.7140   24.2990   32.3419   41.9886   53.3848   66.6764   82.0090   99.5286  119.3809  141.7116  166.6659]/1000;
frontslopey=[  300.0000  284.2105  268.4211  252.6316  236.8421  221.0526  205.2632  189.4737  173.6842  157.8947  142.1053  126.3158 110.5263   94.7368   78.9474   63.1579   47.3684   31.5789   15.7895    0.00045]/100;
frontslopexmax=max(frontslopex);
frontslopexmin=min(frontslopey);

rearslopex=[ 1.0000    1.0525    1.1100    1.1725    1.2400    1.3125    1.3900    1.4725    1.5600    1.6525    1.7500    1.8525    1.9600 2.0725    2.1900    2.3125    2.4400    2.5725    2.7100    2.8525    3.0000];
rearslopey=[  1.0000    0.9999    0.9990    0.9966    0.9920    0.9844    0.9730    0.9571    0.9360    0.9089    0.8750    0.8336    0.7840 0.7254    0.6570    0.5781    0.4880    0.3859    0.2710    0.1426         1e-8];
rearslopexmax=max(rearslopex);
rearslopexmin=min(rearslopex);


% Below parameters are temporarily defined.
% They need to be identified based on the experimental vehicle.
hcg=h_roll;               %Height of COG
Iw=1.2;                   %Wheel intertia
Ns=10;                  %Steering Gear Ratio
Is=0.1;                 %Steering Inertia

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% for Envelope control

% Vehicle envelope control
q=1;
K=10;   % error dynamics gain
Ks=-10;   % servo gain (1/time constant of target first order delay response)
Ku=-40000; % switching gain

% Slip control
Ksl=-10;    % servo gain (1/time constant of target first order delay response)
Kt=40000;   % switching gain


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% for sim

Vxini=60/3.6;                %Initial Velocity

% mu
mu_p=0.6;
mu_s=mu_p;
l=6*tp;


