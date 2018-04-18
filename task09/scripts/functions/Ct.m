% Constraint vector time derivative Ct
function Ct = Ct(t,data)

Ct = [];

joints = data.joints;
constraints = data.constraints;

% Non-rheonomic constraints yield zeros
for j = 1:numel(joints)
    jtype = joints{j}.type;
    
    if strcmp(jtype,'revolute')
        Ct = [Ct; zeros(2,1)];
    elseif strcmp(jtype,'translational')
        Ct = [Ct; zeros(2,1)];
    elseif strcmp(jtype,'1DOF')
        Ct = [Ct; 0];
    end
end

% Time-dependent constraints
for j = 1:numel(constraints)
    bdof = bcoords(data.constraints(j).body);
    cdof = bdof(data.constraints(j).dof);
    cdiff = data.constraints(j).diff;
    
    Ct = [Ct; cdiff(t)];
end

end