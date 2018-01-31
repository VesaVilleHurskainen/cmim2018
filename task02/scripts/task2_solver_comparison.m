clear
close all


% System parameters
m = 1;
k = 100;
c = 0.1;

% Computation parameters
y0 = [0.1;0];           % Starting position and veloctiy
tspan = 0 : 0.01 : 10;  % Solved timespan

% Tolerances (set to 0 to use default parameters)
rtol = 0;
atol = 0;


% Compute natural frequencies
dr = c/(2*sqrt(k*m));   % Damping ratio
wn = sqrt(k/m);         % Undamped natural frequency
wd = wn*sqrt(1-dr^2);   % Damped natural frequency

% Equation of motion for spring-mass-damper: mx'' + cx' + kx = 0
% Reduced to 1st order ODE system (y1 = x, y2 = x') :
odefun = @(t,y) [y(2) ; (- k*y(1) - c*y(2))/m];

% Analytical response function (underdamped case, F = 0)
anfun = @(t,x0,xd0) exp(-dr*wn*t)*((xd0 + dr*wn*x0)/wd*sin(wd*t) + x0*cos(wd*t));

% Compute response using analytical function
y_an = zeros(length(tspan),1);
for i=1:length(tspan)
    y_an(i) = anfun(tspan(i),y0(1),y0(2));
end

% Set solvers and solver options
if (rtol > 0) && (atol > 0)
    opts = odeset('RelTol',rtol,'AbsTol',atol);
    [~,y_45] = ode45(odefun,tspan,y0,opts);
    [~,y_23] = ode23(odefun,tspan,y0,opts);
    [~,y_113] = ode113(odefun,tspan,y0,opts);
else
    [~,y_45] = ode45(odefun,tspan,y0);
    [~,y_23] = ode23(odefun,tspan,y0);
    [~,y_113] = ode113(odefun,tspan,y0);
end


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
err_45 = y_45(:,1)-y_an;
err_23 = y_23(:,1)-y_an;
err_113 = y_113(:,1)-y_an;
figure
plot(tspan,err_45);
hold on
plot(tspan,err_23);
plot(tspan,err_113);
hold off
grid on
title('Position error')
ylabel('Error [m]')
xlabel('Time [s]')
legend('ode45','ode23','ode113')
