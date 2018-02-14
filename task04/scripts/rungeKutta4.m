% Runge-Kutta 4th order method
function [t,y] = rungeKutta4(fun, tspan, y0)

t = tspan;
y = zeros(length(y0),length(tspan));
y(:,1) = y0;

for i = 1:length(tspan)-1
    dt = tspan(i+1)-tspan(i);
    k1 = dt*fun(tspan(i), y(:,i));
    k2 = dt*fun(tspan(i) + (1/2)*dt, y(:,i) + (1/2)*k1);
    k3 = dt*fun(tspan(i) + (1/2)*dt, y(:,i) + (1/2)*k2);
    k4 = dt*fun(tspan(i) + dt, y(:,i) + k3);
    y(:,i+1) = y(:,i) + (1/6)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;
end