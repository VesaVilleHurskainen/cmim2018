% Forward Euler method
function [t,y ]= fwdEuler(fun, tspan, y0)

t = tspan;
y = zeros(length(y0),length(tspan));
y(:,1) = y0;

for i = 1:length(tspan)-1
    dt = tspan(i+1)-tspan(i);
    y(:,i+1) = y(:,i) + dt*fun(tspan(i), y(:,i));
end