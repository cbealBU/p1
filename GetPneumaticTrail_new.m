function [tp] = GetPneumaticTrail_new(tp_window,i,tp0,tpprev,Fyhatprev,Mzprev,tmprev)

% JH 5/19/08
%
% GetPneumaticTrail returns the window-averaged pneumatic trail calculated from total aligning
% moment, lateral force estimate and mechanical trail.  If the index i of the simulation is too
% early to have enough previous tp's to average over (or if lateral force estimate is zero), 
% the function defaults to nominal tp0.

if i>tp_window+1 && abs(Fyhatprev)>0
    if abs(Fyhatprev) > 0
        % if Fyhatprev is nonzero, then calculate new tp value
        tpnew = -(Mzprev/(Fyhatprev)+tmprev);
    else
        % otherwise, use last tp value as current value
        tpnew = tpprev(end);
    end
    % average over a window of length tp_window
    tp = 0;
    for jj=1:tp_window-1
       tp = tp + tpprev(jj)/tp_window;
    end
    tp = tp + tpnew/tp_window;
else
    tp = tp0;
end