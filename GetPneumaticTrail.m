function [tp] = GetPneumaticTrail(Fyhat,Mz,tm,tp_window,i,tp0,tpprev)

% JH 5/19/08
%
% GetPneumaticTrail returns the window-averaged pneumatic trail calculated from total aligning
% moment, lateral force estimate and mechanical trail.  If the index i of the simulation is too
% early to have enough previous tp's to average over (or if lateral force estimate is zero), 
% the function defaults to nominal tp0.

% if i>tp_window+1 && abs(Fyhat(i-1))>0
%     % average over a window of length tp_window
%     tp_count=1;
%     for jj=i-tp_window:i-1
%         % algebraically solve for tp based on previously estimated Fyf
%         if abs(Fyhat(jj)) > 0
%             tp_portion(tp_count) = -(Mz(jj)/(Fyhat(jj))+tm(jj));
%         else
%             tp_portion(tp_count) = tpprev(jj);
%         end
%         tp_count=tp_count+1;
%     end
%     tp = mean(tp_portion);
% else
%     tp = tp0;
% end

if i>tp_window+1 && abs(Fyhat(i-1))>0
    if abs(Fyhat(i-1)) > 0
        tpnew = -(Mz(i-1)/(Fyhat(i-1))+tm(i-1));
    else
        tpnew = tpprev(i-2);
    end
    % average over a window of length tp_window
    tp = 0;
    for jj=i-tp_window:i-2
       tp = tp + tpprev(jj)/tp_window;
    end
    tp = tp + tpnew/tp_window;
else
    tp = tp0;
end