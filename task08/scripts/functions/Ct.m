% Constraint vector time derivative Ct
function Ct = Ct(x,xd,t,data)

Ct = [];

joints = data.joints;
constraints = data.constraints;

% Non-rheonomic constraints yield zeros
for j = 1:numel(joints)
    jtype = joints(j).type;
    jbody = joints(j).bodies;
    jposi = joints(j).positions;
    
    if jbody(1) == 0
        bpos = [[0;0;0], x(bcoords(jbody(2)))];
        bvel = [[0;0;0], xd(bcoords(jbody(2)))];
    elseif jbody(2) == 0
        bpos = [x(bcoords(jbody(1))), [0;0;0]];
        bvel = [xd(bcoords(jbody(1))), [0;0;0]];
    else
        bpos = [x(bcoords(jbody(1))), x(bcoords(jbody(2)))];
        bvel = [xd(bcoords(jbody(1))), xd(bcoords(jbody(2)))];
    end

    if strcmp(jtype,'revolute')
        Ct = [Ct; zeros(2,1)];
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