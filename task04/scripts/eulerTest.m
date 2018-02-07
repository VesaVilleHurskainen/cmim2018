% Parameters
r = 0.5;
n0 = 1;
t0 = 0;
t1 = 10;
tstep = 0.1;

% Simulation timespan
tspan = t0:tstep:t1;

% Define differential function
fun = @(n,t) r*n;

% Compute solution using forward Euler
n_euler = fwdEuler(fun,n0,tspan);

% Compute analytical solution for comparison
analytic_fun = @(t) n0*exp(r*t);
n_analytic = analytic_fun(tspan);

% Plot results
plot(tspan,n_euler)
hold on
plot(tspan,n_analytic)
hold off
grid on
xlabel('Time t')
ylabel('Population n')
legend('Forward Euler','Analytic solution')