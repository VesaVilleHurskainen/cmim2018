% Routine for dynamics solution
function [x,xd,lambda,t] = run_dynamics(data)

bodies = data.bodies;
ncoords = data.coords;

% Define initial position vector
x0 = zeros(ncoords,1);
for i = 1:numel(bodies)
    x0(3*(i-1)+1:3*i) = bodies(i).position;
end

% Define initial condition for solution
y0 = [x0;zeros(size(x0));zeros(data.consts,1)];

timespan = data.timespan;
solver = data.solver;
opts = data.options;

% Solve system
if strcmp(solver,'ode45')
    [t,y] = ode45(@dynamicsFunction,timespan,y0,opts,data);
end

% Separate solution into vectors
x = y(:,1:ncoords)';
xd = y(:,ncoords+1:ncoords*2)';
lambda = y(:,ncoords*2+1:end)';