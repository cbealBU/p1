function [Iforce] = PeakForceEstimator(tp,tpthres,alphahat,alphathres,If_nom,Ca,Iforce_prev,tp0,tm,Mz,mu_ratio,ECstatus,Iforceprev)

% JH 5/19/08
%
% PeakForceEstimator returns inverted peak lateral force estimate.  The
% estimator is purely algebraic, using a simple linearly modeled relationship between inverted peak force and
% pneumatic trail.  If tires are saturated (alphahat > alpha_sl), then peak force can be determined
% purely from total aligning moment and mechanical trail.
% Inputs:
%  tp - pneumatic trail estimate outputted from GetPneumaticTrail.m (m)
%  tpthres - pneumatic trail threshold (m)
%  alphahat - estimated slip angle (rad)
%  alphathres - slip angle threshold for when Iforce estimation begins (rad)
%  If_nom - nominal inverted peak force (1/N)
%  Ca - cornering stiffness (N/rad)
%  Iforce_prev - previous time step's inverted peak force (1/N)
%  tp0 - nominal pneumatic trail (m)
%  tm - mechanical trail (m)
%  Mz - total aligning moment (Nm)
%  mu_ratio - ratio of mu_slide/mu_peak (.)
%  ECstatus - status flag of envelope controller: -1=off, 0=no_saturation, 1=front saturation, 2=rear saturation
%  Iforceprev 
% Outputs:
%  Iforce - inverted peak lateral force estimate (1/N)

PURESLIPCONST = 3;
if ECstatus > 0.5
    Iforce = Iforceprev;
else
    if tp >= tpthres || abs(alphahat) < alphathres
        Iforce = If_nom;
    else
        % are we before full slide (based on ESTIMATED If values)?
        %%%%%%%%%%%%%%%%%%%%%% TRY VARYING CONSTANT FOR FULL SLIP: %%%%%%%%%%%%%%%%%%%%%%
        if abs(alphahat) < atan(PURESLIPCONST/(Ca*Iforce_prev))
            %%%%%%%%%%%%%%%%%%%%%%% TRY VARYING CONSTANT FOR FULL SLIP: %%%%%%%%%%%%%%%%%%%%%%
            Iforce = -PURESLIPCONST*(tp - tp0)/(tp0*Ca*abs(tan(alphahat)));
        else
            % since tp = 0 at full slide, use Mz info to solve for If
            Iforce = sign(alphahat)*tm*mu_ratio/Mz;
        end
    end
end
% Check that Iforce is reasonable (cap peak force estimate at mu = 1 & Fnf, which assumes that each tire will not have a 
% greater peak force than when on a high-mu surface and ALL of the weight of the front axle has
% transferred to it.)
Iforce = max(Iforce,If_nom);