% MBD study code input file, four-bar linkage dynamics
% March 2018, Vesa-Ville Hurskainen

% Initialize struct
clear
data = struct();


% BEGIN PARAMETER DEFINITIONS

% Define solution parameters
data.dt = 0.01;
t_end = 5;
data.timespan = 0:data.dt:t_end;
data.solver = 'ode45';
data.options = odeset;

% Define parameters for Newton-Raphson method
data.abstol = 1e-6;
data.maxiter = 1000;

% Define parameters for Baumgarte stabilization
data.alpha = 10;
data.beta = 10;

% Define numerical parameters
data.g = [0,-9.81]';
L = 1;
m = 1;

% Define bodies and their initial positions
body1.type = 'slenderRod';        % Body type
body1.position = [0 L/2 pi/2]';   % Body initial position in global coordinates
body1.length = L;                 % Rod length
body1.mass = m;                   % Rod mass

body2.type = 'slenderRod';
body2.position = [L L 0]';
body2.length = 2*L;
body2.mass = 2*m;

body3.type = 'slenderRod';
body3.position = [3*L/2 L/2 -3*pi/4]';
body3.length = L*sqrt(2);
body3.mass = m*sqrt(2);

data.bodies = [body1,body2,body3];
    
% Define joint constraints
joint1.type = 'revolute';             % Joint type
joint1.bodies = [0,1];                % Joint bodies (numbering by order in bodies, 0 = ground)
joint1.positions = [[0;0],[-L/2;0]];  % Joint (local) positions

joint2.type = 'revolute';
joint2.bodies = [1,2];
joint2.positions = [[L/2;0],[-L;0]];

joint3.type = 'revolute';
joint3.bodies = [2,3];
joint3.positions = [[L;0],[-L*sqrt(2)/2;0]];

joint4.type = 'revolute';
joint4.bodies = [3,0];
joint4.positions = [[L*sqrt(2)/2;0],[L;0]];

data.joints = [joint1,joint2,joint3,joint4];

% % Define time dependent constraints and their derivatives as functions
% % At the moment, only functions of type q_i = f(t) are implemented,
% const_body = 1;                     % Affected body
% const_dof = 3;                      % Affected degree of freedom of the body
% const_expr = @(t) (pi/2)+omega*t;   % Constraint function
% const_diff = @(t) omega;            % Constraint function time derivative
% const_ddiff = @(t) 0;               % Function second time derivative
% const1 = struct('body',const_body,'dof',const_dof,'expression',const_expr,'diff',const_diff,'ddiff',const_ddiff);
% 
data.constraints = [];

% END PARAMETER DEFINITIONS


% BEGIN SOLUTION

% Solve kinematics
[x,xd,t] = analyse(data);

% Visualize results
% pproc_animate(x,t,data);

% Plot constraint drift
for i = 1:length(t)
    Rc(:,i) = sum(abs(C(x(:,i),t(i),data)));
end
% figure
semilogy(t,abs(Rc),'k-.','LineWidth',1);
grid on