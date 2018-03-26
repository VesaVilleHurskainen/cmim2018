function [x,xd,xdd,t] = analyse(data)

% Calculate system's degrees of freedom
coord = numel(data.bodies)*3;

const = 0;
for i = 1:numel(data.joints)
    if strcmp(data.joints(i).type,'revolute')
        const = const+2;
    end
end

const = const + numel(data.constraints);

dof = coord-const;

% Choose analysis type according to DOF count
if dof > 0
    [x,xd,xdd,t] = run_dynamics(data);
    
elseif dof == 0
    [x,xd,xdd,t] = run_kinematics(data);
    
else
    disp('Overconstrained system: negative degrees of freedom.')
    x = 0;
    xd = 0;
    xdd = 0;
    t = 0;
end

end