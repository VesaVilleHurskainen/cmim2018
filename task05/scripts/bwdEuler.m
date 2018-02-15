% Backward Euler method
function [t,y] = bwdEuler(fun, tspan, y0, jac, tol, maxiter)

t = tspan;
y = zeros(length(y0),length(tspan));
y(:,1) = y0;

for i = 1:length(tspan)-1
    t2 = tspan(i+1);
    dt = t2-tspan(i);
    y1 = y(:,i);
    y(:,i+1) = newtonRaphson(@(y2) y2-y1-dt*fun(t2,y2), y1, @(y) eye(length(y0))-dt*jac(y), tol, maxiter);
end