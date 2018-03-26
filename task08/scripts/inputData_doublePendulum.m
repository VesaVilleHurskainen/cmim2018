% MBD study code input file, double pendulum dynamic computation
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
data.alpha = 1;
data.beta = 1;

% Define numerical parameters
data.g = [0,-9.81]';
L = 1;
m = 1;

% Define bodies and their initial positions
type = 'slenderRod';        % Body type
position = [L/2 0 0]';   % Body initial position in global coordinates
length = L;                 % Rod length
mass = m;                   % Rod mass
body1 = struct('type',type,'position',position,'length',length,'mass',mass);

type = 'slenderRod';
position = [3*L/2 0 0]';
length = L;
mass = m;
body2 = struct('type',type,'position',position,'length',length,'mass',mass);

data.bodies = [body1,body2];
    
% Define joint constraints
joint_type = 'revolute';             % Joint type
joint_bodies = [0,1];                % Joint bodies (numbering by order in bodies, 0 = ground)
joint_positions = [[0;0],[-L/2;0]];  % Joint (local) positions
joint1 = struct('type',joint_type,'bodies',joint_bodies,'positions',joint_positions);

joint_type = 'revolute';
joint_bodies = [1,2];
joint_positions = [[L/2;0],[-L/2;0]];
joint2 = struct('type',joint_type,'bodies',joint_bodies,'positions',joint_positions);

data.joints = [joint1,joint2];

% % Define time dependent constraints and their derivatives as functions
% % At the moment, only functions of type q_i = f(t) are implemented
% const_body = 1;                     % Affected body
% const_dof = 3;                      % Affected degree of freedom of the body
% const_expr = @(t) (pi/2)+omega*t;   % Constraint function
% const_diff = @(t) omega;            % Constraint function time derivative
% const_ddiff = @(t) 0;               % Function second time derivative
% const1 = struct('body',const_body,'dof',const_dof,'expression',const_expr,'diff',const_diff,'ddiff',const_ddiff);
% 
% data.constraints = [const1];

% END PARAMETER DEFINITIONS


% BEGIN SOLUTION

% Solve kinematics
[x,xd,t] = analyse(data);

% Visualize results
pproc_animate(x,t,data);