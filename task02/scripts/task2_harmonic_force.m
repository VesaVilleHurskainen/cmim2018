clear
close all


% System parameters
m = 1;
k = 100;
c = 0.1;
A = 10;

% Computation parameters
y0 = [0;0];           % Starting position and veloctiy
tspan = 0 : 0.01 : 10;  % Solved timespan

% Tolerances (set to 0 to use default parameters)
rtol = 1e-6;
atol = 1e-9;

% Define function for force frequency
omega = @(t) 2*t;


% Compute natural frequencies
dr = c/(2*sqrt(k*m));   % Damping ratio
wn = sqrt(k/m);         % Undamped natural frequency
wd = wn*sqrt(1-dr^2);   % Damped natural frequency

% Compute time when force frequency passes damped natural frequency
wdfun = @(t) omega(t)-wd;
t_wd = fzero(wdfun,0);

% Equation of motion for spring-mass-damper: mx'' + cx' + kx = F
% Reduced to 1st order ODE system (y1 = x, y2 = x') :
odefun = @(t,y) [y(2) ; (- k*y(1) - c*y(2) + A*sin(omega(t)*t))/m];

% Choose solver and solver options
% opts = odeset('RelTol',rtol,'AbsTol',atol);
[~,y] = ode45(odefun,tspan,y0);


% Plot response
figure
plot(tspan,y(:,1));
hold on
plot([t_wd,t_wd],ylim);
grid on
title('System response')
ylabel('Displacement [m]')
xlabel('Time [s]')