% Test script for forward Euler method, CMIM
% By VVH, Feb 2018

clear
close all

% Select test type:
Case = 2;

if Case == 1    % Population
    
    % Parameters
    r = 0.5;
    n0 = 1;
    
    t0 = 0;
    t1 = 10;
    tstep = 0.1;

    % Simulation timespan
    tspan = t0:tstep:t1;

    % Define differential equation as function
    fun = @(n,t) r*n;

    % Compute solution using forward Euler
    n_euler = fwdEuler(fun,n0,tspan);

    % Compute analytical solution for comparison
    analytic_fun = @(t) n0*exp(r*t);
    n_analytic = analytic_fun(tspan);

    % Plot results
    figure
    plot(tspan,n_euler)
    hold on
    plot(tspan,n_analytic)
    hold off
    grid on
    xlabel('Time t')
    ylabel('Population n')
    legend('Forward Euler','Analytic solution')
    
    % Plot error
    figure
    plot(tspan,n_euler-n_analytic)
    hold off
    grid on
    xlabel('Time t')
    ylabel('Error')
    
elseif Case == 2    % Mass-spring-damper
    m = 1;
    k = 100;
    c = 0;  % Undamped system for energy balance checking
    A = 1;
    
    t0 = 0;
    t1 = 1;
    tstep = 0.001;
    
    % Define timespan
    tspan = t0:tstep:t1;
    
    % Define differential equation as function
    fun = @(y,t) [y(2); (-c*y(2)-k*y(1))/m];
    y0 = [A,0];
    
    % Solve response using forward Euler
    n_euler = fwdEuler(fun,y0,tspan);
    
    % Plot position and velocity
    figure
    plot(tspan,n_euler)
    grid on
    xlabel('Time t')
    ylabel('Response')
    legend('x','v')
    
    % Compute and plot system energy
    T_potential = 0.5*k*n_euler(1,:).^2;
    T_kinetic = 0.5*m*n_euler(2,:).^2;
    T_total = T_potential+T_kinetic;
    
    figure
    plot(tspan,T_total)
    grid on
    xlabel('Time t')
    ylabel('System energy')
    
else
    disp('Select a case!')
end