function [x,xd,xdd,t] = run_kinematics(data)

addpath('functions')

% Define starting position
x0 = zeros(3*numel(data.bodies),1);
for i = 1:numel(data.bodies)
    x0(3*(i-1)+1:3*i) = data.bodies(i).position;
end

t = data.timespan;
x = zeros(length(x0),length(t));
xd = zeros(length(x0),length(t));
xdd = zeros(length(x0),length(t));


% Loop over time steps
for i = 1:length(t)
       
    h = 1e-4;
    tol = 1e-6;
    maxiter = 1000;
    
    if i == 1
        % No need to solve position at initial time.
        x(:,i) = x0;
        
        y0d = zeros(3*numel(data.bodies),1);
        y0dd = zeros(3*numel(data.bodies),1);
        
    else
        y0 = x(:,i-1);
        y0d = xd(:,i-1);
        y0dd = xdd(:,i-1);
        
        % Solve position for current time step
        const_eq = @(y) C(y,t(i),data);
        const_jac = @(y) jacobian_fdiff(const_eq,y,h);
        x(:,i) = newtonRaphson(const_eq, y0, const_jac, tol, maxiter);
    end
    
    % Solve velocity for current time step
    constd_eq = @(yd) Cq(x(:,i),data)*yd + Ct(x(:,i),yd,t(i),data);
    constd_jac = @(yd) jacobian_fdiff(constd_eq,yd,h);
    xd(:,i) = newtonRaphson(constd_eq, y0d, constd_jac, tol, maxiter);
    
    % Solve acceleration for current time step
    constdd_eq = @(ydd) Cq(x(:,i),data)*ydd - Cgamma(x(:,i),xd(:,i),t(i),data);
    constdd_jac = @(ydd) jacobian_fdiff(constdd_eq,ydd,h);
    xdd(:,i) = newtonRaphson(constdd_eq, y0dd, constdd_jac, tol, maxiter);
    
end