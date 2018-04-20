% MBD study code input file, grabber dynamics, optimized full model
% April 2018, Vesa-Ville Hurskainen

% Initialize struct
clear
data = struct();


% BEGIN PARAMETER DEFINITIONS

% Define solution parameters
data.dt = 0.01;
t_end = 1;
data.timespan = 0:data.dt:t_end;
data.solver = 'ode45';
data.options = odeset;

% Define parameters for Newton-Raphson method
data.abstol = 1e-6;
data.maxiter = 1000;

% Define parameters for Baumgarte stabilization
data.alpha = 100;
data.beta = 100;

% Define numerical parameters
data.g = [0;0];
L1 = 5;
m = 1;
F = -400;
rhoL = 2;

% Define points
pA = [0;0];
pK = [9;6];

% Unoptimized
pB = [2;4];
pC = [-3;4];

% Optimized
% pB = [0.0687;6.6047];
% pC = [-3.0713;1.2103];

pBl = [pB(1);-pB(2)];
pCl = [pC(1);-pC(2)];
pKl = [pK(1);-pK(2)];

% Define bodies and their initial positions
body1.type = 'slenderRod';        % Body type
body1.points = [pA-[L1;0],pA];    % Body initial position in global coordinates (by endpoints)

body2.type = 'slenderRod';
body2.points = [pB,pK];

body3.type = 'slenderRod';
body3.points = [pA,pB];

body4.type = 'slenderRod';
body4.points = [pB,pC];

body5.type = 'slenderRod';
body5.points = [pBl,pKl];

body6.type = 'slenderRod';
body6.points = [pA,pBl];

body7.type = 'slenderRod';
body7.points = [pBl,pCl];

% Calculate lenghts
L2a = norm(pB-pK);
L2b = norm(pA-pB);
L3 = norm(pB-pC);

% Define masses
body1.mass = rhoL*L1;
body2.mass = rhoL*L2a;
body3.mass = rhoL*L2b;
body4.mass = rhoL*L3;
body5.mass = rhoL*L2a;
body6.mass = rhoL*L2b;
body7.mass = rhoL*L3;


data.bodies = {body1,body2,body3,body4,body5,body6,body7};
   
% Define joint constraints
joint1.type = 'translational';  % Joint type
joint1.bodies = [0,1];          % Joint bodies (numbering by order in bodies, 0 = ground)
joint1.positions = [pA, pA-[L1/2;0]];
joint1.normal = [0;1];          % Normal vector in 1st body reference frame (to-do: implement normal calculation from points if not specified)

joint2.type = 'revolute';
joint2.bodies = [1,3];
joint2.positions = [[L1/2;0],[-L2b/2;0]];

joint3.type = 'revolute';
joint3.bodies = [3,2];
joint3.positions = [[L2b/2;0],[-L2a/2;0]];

joint4.type = 'revolute';
joint4.bodies = [3,4];
joint4.positions = [[L2b/2;0],[-L3/2;0]];

joint5.type = 'revolute';
joint5.bodies = [4,0];
joint5.positions = [[L3/2;0],pC];

joint6.type = '1DOF';
joint6.bodies = [2,3];
joint6.dof = 3;

joint7.type = 'revolute';
joint7.bodies = [1,6];
joint7.positions = [[L1/2;0],[-L2b/2;0]];

joint8.type = 'revolute';
joint8.bodies = [6,5];
joint8.positions = [[L2b/2;0],[-L2a/2;0]];

joint9.type = 'revolute';
joint9.bodies = [6,7];
joint9.positions = [[L2b/2;0],[-L3/2;0]];

joint10.type = 'revolute';
joint10.bodies = [7,0];
joint10.positions = [[L3/2;0],pCl];

joint11.type = '1DOF';
joint11.bodies = [5,6];
joint11.dof = 3;

data.joints = {joint1,joint2,joint3,joint4,joint5,joint6,joint7,joint8,joint9,joint10,joint11};

% Define point forces
force1.body = 1;                    % Affected body
force1.forcevector = @(t) [F;0];    % Force vector (function of time)
force1.location = [0;0];            % Effect point coordinates (body relative)

data.forces = {force1};

% END PARAMETER DEFINITIONS


% BEGIN SOLUTION

% Solve kinematics
[x,xd,t,data] = analyse(data);

% Visualize results
pproc_animate(x,t,data);