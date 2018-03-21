% Initialize struct
clear
data = struct();


% BEGIN PARAMETER DEFINITIONS

% Define solution parameters
t_step = 0.01;
t_end = 1;
data.timespan = 0:t_step:t_end;

% Define numerical parameters
L = 1;
omega = -1;

% Define bodies and their initial positions
type = 'slenderRod';        % Body type
position = [0 L/2 pi/2]';   % Body initial position in global coordinates
length = L;                 % Rod length
body1 = struct('type',type,'position',position,'length',length);

type = 'slenderRod';
position = [L/2 L 0]';
length = L;
body2 = struct('type',type,'position',position,'length',length);

type = 'slenderRod';
position = [L L/2 -pi/2]';
length = L;
body3 = struct('type',type,'position',position,'length',length);

data.bodies = [body1,body2,body3];
    
% Define joint constraints
joint_type = 'revolute';             % Joint type
joint_bodies = [0,1];                % Joint bodies (numbering by order in bodies, 0 = ground)
joint_positions = [[0;0],[-L/2;0]];  % Joint (local) positions
joint1 = struct('type',joint_type,'bodies',joint_bodies,'positions',joint_positions);

joint_type = 'revolute';
joint_bodies = [1,2];
joint_positions = [[L/2;0],[-L/2;0]];
joint2 = struct('type',joint_type,'bodies',joint_bodies,'positions',joint_positions);

joint_type = 'revolute';
joint_bodies = [2,3];
joint_positions = [[L/2;0],[-L/2;0]];
joint3 = struct('type',joint_type,'bodies',joint_bodies,'positions',joint_positions);

joint_type = 'revolute';
joint_bodies = [3,0];
joint_positions = [[L/2;0],[L;0]];
joint4 = struct('type',joint_type,'bodies',joint_bodies,'positions',joint_positions);

data.joints = [joint1,joint2,joint3,joint4];

% Define time dependent constraints and their derivatives as functions
% At the moment, only functions of type q_i = f(t) are implemented
const_body = 1;     % Affected body
const_dof = 3;      % Affected degree of freedom of the body
const_expr = @(t) (pi/2)+omega*t;   % Constraint function
const_diff = @(t) omega;            % Constraint function time derivative
const_ddiff = @(t) 0;               % Function second time derivative
const1 = struct('body',const_body,'dof',const_dof,'expression',const_expr,'diff',const_diff,'ddiff',const_ddiff);

data.constraints = [const1];

% END PARAMETER DEFINITIONS


% BEGIN SOLUTION

% Solve kinematics
[x,xd,xdd,t] = run_kinematics(data);

% Visualize results
pproc_animate(x,t,data);