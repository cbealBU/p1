% JH 4/17/08
% GetTorques  Returns torque commanded to each tire (in Nm) determined by HW3 torque schedule
% Takes in 
%  s - current distance down path (m)
%  spower - point at which to press on the throttle (m)

function torque = GetTorques(s,spower)

if s >= 20 && s <= 55
    % hard braking
    Tfr = -1000;
    Tfl = -1000;
    Trl = -500;
    Trr = -500;
elseif s > 55 && s < 100
    % braking entering a curve
    rate_f = 1000/(100-55); 
    rate_r = 500/(100-55);
    % ramp up from negative torque to 0 linearly
    Tfr = -1000 + rate_f*(s-55);
    Tfl = -1000 + rate_f*(s-55);
    Trl = -500 + rate_r*(s-55);
    Trr = -500 + rate_r*(s-55);
elseif s >= spower && s <= 500 
    % powering out of the turn 
    Trl = 450;
    Trr = 450;
    Tfr = 0;
    Tfl = 0;
else
    Trl = 10;
    Trr = 10;
    Tfr = 0;
    Tfl = 0;
end

% repack my torques into a tidy matrix
torque = [Trl Tfl; 
          Trr Tfr];