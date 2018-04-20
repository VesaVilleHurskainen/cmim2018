% Fitness function for grabber optimization
% April 2018, Vesa-Ville Hurskainen

function f = optFun(xx)

% Clear old data and initialize struct
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
F = -200;
rhoL = 2;

% Define points from arguments
pA = [0;0];
pB = xx(1:2);
pC = xx(3:4);
pK = [9;6];

% Define bodies and their initial positions
body1.type = 'slenderRod';        % Body type
body1.points = [pA-[L1;0],pA];    % Body initial position in global coordinates (by endpoints)

body2.type = 'slenderRod';
body2.points = [pB,pK];

body3.type = 'slenderRod';
body3.points = [pA,pB];

body4.type = 'slenderRod';
body4.points = [pB,pC];

% Calculate lenghts
L2a = norm(pB-pK);
L2b = norm(pA-pB);
L3 = norm(pB-pC);

% Define masses
body1.mass = rhoL*norm(body1.points(:,1)-body1.points(:,2));
body2.mass = rhoL*norm(body2.points(:,1)-body2.points(:,2));
body3.mass = rhoL*norm(body3.points(:,1)-body3.points(:,2));
body4.mass = rhoL*norm(body4.points(:,1)-body4.points(:,2));

data.bodies = {body1,body2,body3,body4};
    
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

% Define point forces
force1.body = 1;                    % Affected body
force1.forcevector = @(t) [F;0];    % Force vector (function of time)
force1.location = [0;0];            % Effect point coordinates (body relative)

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
    if xK(2,i) < 2
        xK(:,i:end) = [];
        break
    end
end

f = max(xK(1,:))-min(xK(1,:));