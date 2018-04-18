% Function for grabber optimization
function f = optFun(xx)

% Clear old data and initialize struct
data = struct();


% BEGIN PARAMETER DEFINITIONS

% Define solution parameters
data.dt = 0.01;
t_end = 3;
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
data.g = [0;0];
L1 = 5;
m = 1;
F = -10;

% Define points from arguments
pA = [0;0];
pB = xx(1:2);
pC = xx(3:4);
pK = [9;6];

% Define bodies and their initial positions
body1.type = 'slenderRod';        % Body type
body1.points = [pA-[L1;0],pA];    % Body initial position in global coordinates (by endpoints)
body1.mass = m;                   % Rod mass

body2.type = 'slenderRod';
body2.points = [pB,pK];
body2.mass = m;

body3.type = 'slenderRod';
body3.points = [pA,pB];
body3.mass = m;

body4.type = 'slenderRod';
body4.points = [pB,pC];
body4.mass = m;

data.bodies = {body1,body2,body3,body4};

% Calculate lenghts
L2a = norm(pB-pK);
L2b = norm(pA-pB);
L3 = norm(pB-pC);
    
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

data.joints = {joint1,joint2,joint3,joint4,joint5,joint6};

% Define point force
force1.body = 1;
force1.forcevector = [F;0];
force1.location = [0;0];

data.forces = {force1};

% END PARAMETER DEFINITIONS


% BEGIN SOLUTION

% Solve
[xdata,t] = analyse(data);

% Calculate f = xK_max - xK_min
x2 = xdata(bcoords(2),:);
xK = zeros(2,length(t));
for i = 1:length(t)
    xK(:,i) = x2(1:2,i) + [cos(x2(3,i)) -sin(x2(3,i)); sin(x2(3,i)) cos(x2(3,i))]*[L3/2;0];
    if xK(2,i) < 0
        xK(:,i:end) = [];
        break
    end
end

f = max(xK(1,:))-min(xK(1,:));