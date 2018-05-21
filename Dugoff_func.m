% Judy Hsu
function F = joodie(x,data)

global Fnf

Fyf = data(:,1);
alpha = data(:,2);
mu = x(1);
Cy = x(2); %Caf;

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

F = Fyf - Fy';

% % simulate & plot
% hold on
% plot(alpha*180/pi,-Fy,'r');

% ylabel('lateral force Fy (N)');
% xlabel('slip angle \alpha (deg)');
% title('Dugoff Tire Model, \mu = 0.8');

    
