% Routine for automatically determining the type of analysis to run
function [x,xd,t] = analyse(data)

addpath('functions')

% Check input and fill if needed
if ~isfield(data,'bodies')
    disp('ERROR: No bodies defined.')
    return
end

if ~isfield(data,'joints')
    data.joints = [];
end

if ~isfield(data,'constraints')
    data.constraints = [];
end

% Calculate system's degrees of freedom
coord = numel(data.bodies)*3;
const = length(C(zeros(coord,1),0,data));
dof = coord-const;

data.coords = coord;
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