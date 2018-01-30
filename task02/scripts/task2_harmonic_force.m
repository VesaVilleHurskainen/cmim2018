clear
close all

% System parameters
m = 1;
k = 100;
c = 0.1;
A = 0;

% Computation parameters
y0 = [0.1;0];           % Starting position and veloctiy
tspan = 0 : 0.01 : 10;  % Solved timespan

% Equation of motion for spring-mass-damper: mx'' + cx' + kx = F
% Reduced to 1st order ODE system (y1 = x, y2 = x') :
omega = @(t) 0;
odefun = @(t,y) [y(2) ; - k/m * y(1) - c/m * y(2) + A/m * sin(omega(t)*t)];

% Compute analytical results for comparison
dr = c/(2*sqrt(k*m));   % Damping ratio
wn = sqrt(k/m);         % Undamped natural frequency
wd = wn*sqrt(1-dr^2);   % Damped natural frequency

% Analytical response function (underdamped case, F = 0)
anfun = @(t,x0,xd0) exp(-dr*wn*t)*((xd0 + dr*wn*x0)/wd*sin(wd*t) + x0*cos(wd*t));

% Compute values using analytical function
y_an = zeros(length(tspan),1);
for i=1:length(tspan)
    y_an(i) = anfun(tspan(i),y0(1),y0(2));
end

% Choose solver and solver options
opts = odeset('RelTol',
[~,y_45] = ode45(odefun,tspan,y0);
[~,y_23] = ode23(odefun,tspan,y0);
[~,y_113] = ode113(odefun,tspan,y0);

% Plot analytically and numerically solved response
figure
plot(tspan,y_an);
hold on
plot(tspan,y_45(:,1))
plot(tspan,y_23(:,1))
plot(tspan,y_113(:,1))
hold off
grid on
title('System response')
ylabel('Displacement [m]')
xlabel('Time [s]')
legend('Analytical','ode45','ode23','ode113')

% Compute and plot position error
err_45 = y_an-y_45(:,1);
err_23 = y_an-y_23(:,1);
err_113 = y_an-y_113(:,1);
figure
plot(tspan,err_45);
hold on
plot(tspan,err_23);
plot(tspan,err_113);
grid on
title('Position error, default settings')
ylabel('Error [m]')
xlabel('Time [s]')
legend('ode45','ode23','ode113')
