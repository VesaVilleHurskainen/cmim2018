% Routine for automatically determining the type of analysis to run
function [x,xd,t] = analyse(data)

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

% Calculate system's degrees of freedom and initial position
coord = numel(data.bodies)*3;
data.coords = coord;

bodies = data.bodies;
x0 = zeros(coord,1);
for i = 1:numel(bodies)
    x0(3*(i-1)+1:3*i) = bodies{i}.position;
end
data.x0 = x0;

const = length(C(zeros(coord,1),0,data));
dof = coord-const;

data.consts = const;
data.dof = dof;

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