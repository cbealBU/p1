% CURRENTFIXUP   Modifies commanded currents to account for voltage saturation.
%    This makes use of the param structure to get the maximum voltage, the
%    motor constant, and the motor resistance.
%
%    [I,w,V]=CURRENTFIXUP(Commands,Steering,Ts,param)
%
%    Commands - the matrix of commanded currents (only the first two, the 
%       current commands, are used)
%    Steering - the matrix of steering data (only the first two, the motor 
%       encoders, are used)
%    Ts - the sample time (used to get the motor speed)
%    param - the param structure
%
%    Version 1.1 by Shad Laws, 11/14/06

function [I,w,V] = currentfixup(Commands,Steering,Ts,param);
    
    % factor to change the motor constant (put 1 for no correction)
    m = 0.123/0.113*0 + 1;

    % speeds
    wl = gradient(Steering(:,1),Ts);
    wr = gradient(Steering(:,2),Ts);
    
    % saturated voltages
    Vl = min(max((Commands(:,1)*param.fl.Rm + wl*param.fl.km *m),-param.fl.Vmax),param.fr.Vmax);
    Vr = min(max((Commands(:,2)*param.fr.Rm + wr*param.fr.km *m),-param.fl.Vmax),param.fr.Vmax);
    
    % saturated currents
    Il = 1/param.fl.Rm * (Vl - wl*param.fl.km *m);
    Ir = 1/param.fr.Rm * (Vr - wr*param.fr.km *m);
    
    I = [Il Ir];
    w = [wl wr];
    V = [Vl Vr];

return;    