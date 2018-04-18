% Constraint vector Jacobian Cq
function Cq = Cq(x,data)

Cq = [];

joints = data.joints;
constraints = data.constraints;

% Joints
for j = 1:numel(joints)
    jtype = joints{j}.type;
    jbody = joints{j}.bodies;
    
    bpos = bodypos(jbody,x);
    
	xa = bpos(1,1);
	ya = bpos(2,1);
    ta = bpos(3,1);
	xb = bpos(1,2);
	yb = bpos(2,2);
    tb = bpos(3,2);

    if strcmp(jtype,'revolute')
        jposi = joints{j}.positions;
        Cqj = ...
            [ 1, 0, - jposi(1,1)*sin(ta) - jposi(2,1)*cos(ta), -1,  0,   jposi(1,2)*sin(tb) + jposi(2,2)*cos(tb)
              0, 1,   jposi(1,1)*cos(ta) - jposi(2,1)*sin(ta),  0, -1, - jposi(1,2)*cos(tb) + jposi(2,2)*sin(tb)];
        Cqt = zeros(2,length(x));
        
    elseif strcmp(jtype,'translational')
        jposi = joints{j}.positions;
        jnorm = joints{j}.normal;
        Cqj = ...
            [ jnorm(1)*cos(ta) - jnorm(2)*sin(ta), jnorm(2)*cos(ta) + jnorm(1)*sin(ta), jnorm(1)*ya*cos(ta) - xa*jnorm(2)*cos(ta) + xb*jnorm(2)*cos(ta) - jnorm(1)*yb*cos(ta) - xa*jnorm(1)*sin(ta) + xb*jnorm(1)*sin(ta) - ya*jnorm(2)*sin(ta) + yb*jnorm(2)*sin(ta) + jposi(1,2)*jnorm(2)*cos(ta - tb) - jnorm(1)*jposi(2,2)*cos(ta - tb) + jposi(1,2)*jnorm(1)*sin(ta - tb) + jposi(2,2)*jnorm(2)*sin(ta - tb), jnorm(2)*sin(ta) - jnorm(1)*cos(ta), - jnorm(2)*cos(ta) - jnorm(1)*sin(ta), (jposi(2,2)*cos(tb) + jposi(1,2)*sin(tb))*(jnorm(1)*cos(ta) - jnorm(2)*sin(ta)) - (jposi(1,2)*cos(tb) - jposi(2,2)*sin(tb))*(jnorm(2)*cos(ta) + jnorm(1)*sin(ta))
			0, 0, 1, 0 0 -1];
        Cqt = zeros(2,length(x));
    
    elseif strcmp(jtype,'1DOF')
        Cqj = zeros(1,6);
        Cqj(joints{j}.dof) = 1;
        Cqj(3+joints{j}.dof) = -1;
        Cqt = zeros(1,length(x));
    end
    
    if jbody(1) == 0
        Cqt(:,bcoords(jbody(2))) = Cqt(:,bcoords(jbody(2))) + Cqj(:,4:6);
    elseif jbody(2) == 0
        Cqt(:,bcoords(jbody(1))) = Cqt(:,bcoords(jbody(1))) + Cqj(:,1:3);
    else
        Cqt(:,[bcoords(jbody(1)),bcoords(jbody(2))]) = Cqt(:,[bcoords(jbody(1)),bcoords(jbody(2))]) + Cqj;
    end

    Cq = [Cq; Cqt];
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