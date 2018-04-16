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
body1.position = [L/2 0 0]';      % Body initial position in global coordinates
body1.length = L;                 % Rod length
body1.mass = m;                   % Rod mass

body2.type = 'slenderRod';
body2.position = [L L/2 pi/2]';
body2.length = L;
body2.mass = m;

data.bodies = {body1,body2};
    
% Define joint constraints
joint1.type = 'revolute';             % Joint type
joint1.bodies = [0,1];                % Joint bodies (numbering by order in bodies, 0 = ground)
joint1.positions = [[0;0],[-L/2;0]];  % Joint position in 1st and 2nd body reference frame

joint2.type = 'translational';
joint2.bodies = [1,2];
joint2.positions = [[L/2;0],[-L/2;0]];
joint2.normal = [0;1];                % Normal vector in 1st body reference frame (to-do: implement normal calculation from points if not specified)

data.joints = {joint1,joint2};

% % Define time dependent constraints and their derivatives as functions
% % At the moment, only functions of type q_i = f(t) are implemented,
% const_body = 1;                     % Affected body
% const_dof = 3;                      % Affected degree of freedom of the body
% const_expr = @(t) (pi/2)+omega*t;   % Constraint function
% const_diff = @(t) omega;            % Constraint function time derivative
% const_ddiff = @(t) 0;               % Function second time derivative
% const1 = struct('body',const_body,'dof',const_dof,'expression',const_expr,'diff',const_diff,'ddiff',const_ddiff);

% Define point forces
force1.body = 2;
force1.forcevector = [0;20];
force1.location = [0;0];

data.forces = {force1};

data.constraints = [];

% END PARAMETER DEFINITIONS


% BEGIN SOLUTION

% Solve kinematics
[x,xd,t] = analyse(data);

% Visualize results
pproc_animate(x,t,data);