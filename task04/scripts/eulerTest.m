% Test script for CMIM task 4
% By VVH, Feb 2018

clear
close all

% Select test type:
Case = 2;
integrator = 'rungeKutta4';

% Define time settings
t0 = 0;
t1 = 1;
tstep = 0.001;

% Define timespan
tspan = t0:tstep:t1;

if Case == 1    % Population with forward Euler
    % System parameters
    r = 0.5;
    n0 = 1;
    
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
    
elseif Case == 2    % Mass-spring-damper, several integrators
    % System parameters
    m = 1;
    k = 100;
    A = 1;
    
    if strcmp(integrator,'fwdEuler')
        % Define differential equation as function
        fun = @(t,y) [y(2); (-k/m)*y(1)];
        y0 = [A,0];

        % Solve response using forward Euler
        [t,y] = fwdEuler(fun,tspan,y0);
        
    elseif strcmp(integrator,'simEuler')
        % Define differential equation as functions
        fun_f = @(t,v) v;
        fun_g = @(t,u) (-k/m)*u;
        u0 = A;
        v0 = 0;

        % Solve response using semi-implicit Euler
        [t,u,v] = simEuler(fun_f,fun_g,tspan,u0,v0);
        y = [u;v];
        
    elseif strcmp(integrator,'rungeKutta4')
        % Define differential equation as function
        fun = @(t,y) [y(2); (-k/m)*y(1)];
        y0 = [A,0];

        % Solve response using Runge-Kutta 4th order method
        [t,y] = rungeKutta4(fun,tspan,y0);
        
    end
    
    % Plot position and velocity
    figure
    plot(t,y)
    grid on
    xlabel('Time t')
    ylabel('Response')
    legend('x','v')
    
    % Compute and plot system energy
    T_potential = 0.5*k*y(1,:).^2;
    T_kinetic = 0.5*m*y(2,:).^2;
    T_total = T_potential+T_kinetic;
    
    figure
    plot(t,T_total)
    grid on
    xlabel('Time t')
    ylabel('System energy')
    
else
    disp('Select a case!')
end