% Semi-implicit Euler method
function [t,u,v] = simEuler(fun_f, fun_g, tspan, u0, v0)

t = tspan;
u = zeros(length(u0),length(tspan));
v = zeros(length(v0),length(tspan));
u(:,1) = u0;
v(:,1) = v0;

for i = 1:length(tspan)-1
    dt = tspan(i+1)-tspan(i);
    v(:,i+1) = v(:,i) + dt*fun_g(tspan(i), u(:,i));
    u(:,i+1) = u(:,i) + dt*fun_f(tspan(i), v(:,i+1));
end