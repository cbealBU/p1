%bicycle_model_dxdt gives the state derivatives of a bicycle model of 
%a vehicle.  In this model, steer angle and sidelsip angle are assumed to
%be small so that small-angle approximations may be used in the derivation

function [xdot] = bicycle_model_dx_dt(t,x)

%Access global variables for vehicle physical parameters
global m Iz L a b 

%Access global variables for steering input parameters
global input_type init_angle final_angle steptime steering_data time_data 
global rate ramptime
%Access global variable for tire model parameters
global tiretype Cf Cr mu_f mu_s_f Fzf Fzr Fxf mu_r mu_s_r

%If we are doing simple coupling solution for Ux, declare global variable for Fxr.
%Otherwise, we don't need it or are solving for it
if strcmp(tiretype, 'fiala - simple coupling - Ux solution')
  global Fxr
end

%Access global variables for longitudinal velocity parameter
global Ux_const Ux_data

%"Unpack" state variables
%Uy and r if using lateral force fiala model
if (strcmp(tiretype, 'linear') || strcmp(tiretype, 'fiala'))
    Uy = x(1);
    r = x(2);
%Uy, r, Ux if using fiala model with simple coupling and solving for Ux
elseif  strcmp(tiretype, 'fiala - simple coupling - Ux solution')
    Uy = x(1);
    r = x(2);
    Ux = x(3);
%Uy, r, Ux if using fiala model with simple coupling and solving for Ux
elseif strcmp(tiretype, 'fiala - simple coupling - Fx solution') 
%   Uy = x(1);
    beta = x(1);
    Uy = Ux_const*tan(beta);
    r = x(2);
    Fxr = x(3);
end

%Compute steering input angle based upon 
if strcmp(input_type, 'step') %step steer
    delta = steering('step', init_angle, final_angle, steptime, t);
elseif strcmp(input_type, 'none') %no steering input
    delta = 0;
elseif (strcmp(input_type, 'from data')) || (strcmp(input_type, 'from data-constant velocity')) %steering angle from data
    delta = steering('from data', steering_data, time_data, t);
elseif strcmp(input_type, 'ramp') %ramp steer
    delta = steering('ramp', init_angle, rate, ramptime, t);
else
    delta = 0;
end

%Tire slip angle computation assuming Ux is an input value 
if (strcmp(tiretype, 'linear') || strcmp(tiretype, 'fiala') || strcmp(tiretype, 'fiala - simple coupling - Fx solution'))
    %If simulating from steering and velocity data
    if strcmp(input_type, 'from data')
        Ux_input = interp1(time_data, Ux_data, t); %Interpolate between velocity data points
    else %Otherwise, use constant input value
        Ux_input = Ux_const;
    end

    %Use states to compute front and rear slip angles

    %If vehicle velocity is above some critical value, compute
    %sideslip...otherwise, set sideslip to zero to avoid division by zero or
    %very small values
    
    if strcmp(tiretype, 'linear')
      if Ux_input > 0.1
        alpha_f = (Uy+a*r)/Ux_input - delta; %front sideslip angle
        alpha_r = (Uy-b*r)/Ux_input;         %rear sideslip angle
      else
        alpha_f = 0;
        alpha_r = 0;
      end
    end
    
    if (strcmp(tiretype, 'fiala') || strcmp(tiretype, 'fiala - simple coupling - Fx solution'))
      if Ux_input > 0.1
        alpha_f = atan2(Uy+a*r,Ux_input) - delta; %front sideslip angle
        alpha_r = atan2(Uy-b*r,Ux_input);         %rear sideslip angle
      else
        alpha_f = 0;
        alpha_r = 0;
      end
    end
%Tire slip angle computation assuming Ux is a state
elseif strcmp(tiretype, 'fiala - simple coupling - Ux solution')
    if Ux > 0.1
        alpha_f = atan2(Uy+a*r,Ux) - delta; %front sideslip angle
        alpha_r = atan2(Uy-b*r,Ux);         %rear sideslip angle
    else
        alpha_f = 0;
        alpha_r = 0;
    end
end
    
%Compute front and rear tire side forces using specified tire model
if strcmp(tiretype, 'linear') %Linear tire model
    Fyf = tireforce('linear', Cf, alpha_f);
    Fyr = tireforce('linear', Cr, alpha_r);
elseif strcmp(tiretype, 'fiala') %Fiala tire model
    Fyf = tireforce('fiala', Cf, Fzf, mu_f, mu_s_f, alpha_f);
    Fyr = tireforce('fiala', Cr, Fzr, mu_r, mu_s_r, alpha_r);
elseif (strcmp(tiretype, 'fiala - simple coupling - Ux solution') || strcmp(tiretype, 'fiala - simple coupling - Fx solution'))  
    Fyf = tireforce('fiala - simple coupling', Cf, Fzf, mu_f, mu_s_f, alpha_f, Fxf);
    Fyr = tireforce('fiala - simple coupling', Cr, Fzr, mu_r, mu_s_r, alpha_r, Fxr);    
end

%Computation of state derivatives/nonlinear function values

if strcmp(tiretype, 'linear')
    %Compute state derivatives Uydot rdot
    Uydot = (Fyf + Fyr)/m - r*Ux_input;
    rdot = (a*Fyf - b*Fyr)/Iz;

    %Concatenate Uydot and rdot into state derivative vector xdot
    xdot = [Uydot; rdot];
elseif strcmp(tiretype, 'fiala')
    %Compute state derivatives Uydot rdot
    Uydot = (Fyf*cos(delta) + Fyr)/m - r*Ux_input;
    rdot = (a*Fyf*cos(delta) - b*Fyr)/Iz;

    %Concatenate Uydot and rdot into state derivative vector xdot
    xdot = [Uydot; rdot];
elseif strcmp(tiretype, 'fiala - simple coupling - Ux solution')
    %Compute state derivatives Uydot rdot Uxdot
    Uydot = (Fyf*cos(delta) + Fyr)/m - r*Ux;
    rdot = (a*Fyf*cos(delta) - b*Fyr)/Iz;
    Uxdot = (Fxf + Fxr - Fyf*sin(delta))/m + r*Uy;
    
    %Concatenate into state derivative vector xdot
    xdot = [Uydot; rdot; Uxdot];
elseif strcmp(tiretype, 'fiala - simple coupling - Fx solution')
    %Compute state derivatives Uydot rdot Uxdot
%     Uydot = (Fyf*cos(delta) + Fyr)/m - r*Ux_input;
%     rdot = (a*Fyf*cos(delta) - b*Fyr)/Iz;
%     Uxdot = (Fxf + Fxr - Fyf*sin(delta))/m + r*Uy;

%     Uydot = (Fyf + Fyr)/m - r*Ux_input;
    betadot = (Fyf + Fyr)/(m*Ux_input) - r;
    rdot = (a*Fyf - b*Fyr)/Iz;
    Uxdot = (Fxf + Fxr - Fyf*sin(delta))/m + r*Uy;
    
    %Concatenate into state derivative vector xdot
%     xdot = [Uydot; rdot; Uxdot];
    xdot = [betadot; rdot; Uxdot];

end
  
end
    