% Semi-implicit Euler method
function y = simEuler(fun, y0, tspan)

y = zeros(length(y0),length(tspan));
y(:,1) = y0;

for i = 1:length(tspan)-1
    dt = tspan(i+1)-tspan(i);
    y(:,i+1) = y(:,i) + dt*fun(y(:,i),tspan(i));
end