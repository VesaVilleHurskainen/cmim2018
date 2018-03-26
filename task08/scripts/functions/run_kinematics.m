% Routine for kinematics solution.
function [x,xd,xdd,t] = run_kinematics(data)

% Define initial position vector
x0 = zeros(3*numel(data.bodies),1);
for i = 1:numel(data.bodies)
    x0(3*(i-1)+1:3*i) = data.bodies(i).position;
end

t = data.timespan;
dt = data.dt;

x = zeros(length(x0),length(t));
xd = zeros(length(x0),length(t));
xdd = zeros(length(x0),length(t));


% Loop over time steps
for i = 1:length(t)
          
    if i == 1
        % At initial time, x = x0
        x(:,i) = x0;
        
    else
        % Use Taylor expansion to determine initial guess
        y0 = x(:,i-1) + xd(:,i-1)*dt + 0.5*xdd(:,i-1)*dt^2;
        
        % Solve position for current time step using Newton-Raphson
        const_eq = @(y) C(y,t(i),data);
        const_jac = @(y) Cq(y,data);
        x(:,i) = newtonRaphson(const_eq, y0, const_jac, data.abstol, data.maxiter);
    end
    
    % Solve velocity for current time step
    xd(:,i) = Cq(x(:,i),data)\(-Ct(t(i),data));
    
    % Solve acceleration for current time step
    xdd(:,i) = Cq(x(:,i),data)\Cgamma(x(:,i),xd(:,i),t(i),data);
    
end