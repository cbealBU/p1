% Judy Hsu
function joodie(x,Fyf,alpha,Fnf,nonlinear)

mu = x(1);
Cy = x(2);

Fy = zeros(1,length(alpha));
for j = 1:length(alpha)
    if alpha(j) == 0
        Fy(j) = 0;
    else
        lambda = mu*Fnf/(2*Cy*abs(tan(alpha(j))));
        if lambda < 1
            f = (2-lambda)*lambda;
        else
            f = 1;
        end
        Fy(j) = -Cy*tan(alpha(j))*f;
    end
end

% simulate & plot

hold on
if nonlinear == 0
    plot(alpha*180/pi,-Fy,'b.');
else
    plot(alpha*180/pi,-Fy,'k.');
end

% ylabel('lateral force Fy (N)');
% xlabel('slip angle \alpha (deg)');
% title('Dugoff Tire Model, \mu = 0.8');

    
