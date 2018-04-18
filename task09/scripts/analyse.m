% Routine for automatically determining the type of analysis to run
function [x,xd,t,data] = analyse(data)

addpath('functions')

% Check input and fill if needed
if ~isfield(data,'bodies')
    error('ERROR: No bodies defined.')
end

if ~isfield(data,'joints')
    data.joints = {};
end

if ~isfield(data,'constraints')
    data.constraints = {};
end

if ~isfield(data,'forces')
    data.forces = {};
end

% If rods defined by points, calculate position and length
bodies = data.bodies;
for i = 1:numel(bodies)
    if strcmp(bodies{i}.type,'slenderRod')
        if isfield(bodies{i},'points')
            bpoints = bodies{i}.points;
            bodies{i}.position = [mean(bpoints,2); atan2((bpoints(2,2)-bpoints(2,1)),(bpoints(1,2)-bpoints(1,1)))];
            bodies{i}.length = norm(bpoints(:,1)-bpoints(:,2));
        end
    end
end
data.bodies = bodies;     

% Calculate system's degrees of freedom and initial position
coord = numel(data.bodies)*3;
data.coords = coord;

bodies = data.bodies;
x0 = zeros(coord,1);
for i = 1:numel(bodies)
    x0(3*(i-1)+1:3*i) = bodies{i}.position;
end
data.x0 = x0;

const = length(Ct(0,data));
dof = coord-const;

data.consts = const;
data.dof = dof;

% Prealculate initial value data for certain joint types
joints = data.joints;
for j = 1:numel(joints)
    if strcmp(joints{j}.type,'translational') || strcmp(joints{j}.type,'1DOF')
        bposi0 = bodypos(joints{j}.bodies,x0);
        joints{j}.init = bposi0;
    end
end
data.joints = joints;

% Choose analysis type according to DOF count
if dof > 0
    [x,xd,~,t] = run_dynamics(data);
    
elseif dof == 0
    [x,xd,~,t] = run_kinematics(data);
    
else
    disp('ERROR: Overconstrained system.')
    x = 0;
    xd = 0;
    t = 0;
end

end