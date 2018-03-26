% Constraint vector Jacobian Cq
function Cq = Cq(x,data)

Cq = [];

joints = data.joints;
constraints = data.constraints;

% Joints
for j = 1:numel(joints)
    jtype = joints(j).type;
    jbody = joints(j).bodies;
    jposi = joints(j).positions;
    
    if jbody(1) == 0
        bpos = [[0;0;0], x(bcoords(jbody(2)))];
    elseif jbody(2) == 0
        bpos = [x(bcoords(jbody(1))), [0;0;0]];
    else
        bpos = [x(bcoords(jbody(1))), x(bcoords(jbody(2)))];
    end
    
    ta = bpos(3,1);
    tb = bpos(3,2);

    if strcmp(jtype,'revolute')
        Cqj = ...
            [ 1, 0, - jposi(1,1)*sin(ta) - jposi(2,1)*cos(ta), -1,  0,   jposi(1,2)*sin(tb) + jposi(2,2)*cos(tb)
              0, 1,   jposi(1,1)*cos(ta) - jposi(2,1)*sin(ta),  0, -1, - jposi(1,2)*cos(tb) + jposi(2,2)*sin(tb)];
        
        Cqt = zeros(2,length(x));
        
        if jbody(1) == 0
            Cqt(:,bcoords(jbody(2))) = Cqt(:,bcoords(jbody(2))) + Cqj(:,4:6);
        elseif jbody(2) == 0
            Cqt(:,bcoords(jbody(1))) = Cqt(:,bcoords(jbody(1))) + Cqj(:,1:3);
        else
            Cqt(:,[bcoords(jbody(1)),bcoords(jbody(2))]) = Cqt(:,[bcoords(jbody(1)),bcoords(jbody(2))]) + Cqj;
        end
        
        Cq = [Cq; Cqt];
    end
end

% Time-dependent constraints
for j = 1:numel(constraints)
    bdof = bcoords(data.constraints(j).body);
    cdof = bdof(data.constraints(j).dof);
    
    Cqb = zeros(1,length(x));
    Cqb(cdof) = -1;
        
    Cq = [Cq; Cqb];
end

end