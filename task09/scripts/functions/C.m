% Routine to compute constraint vector C
function C = C(x,t,data)

C = [];

joints = data.joints;
constraints = data.constraints;

% Joints
for j = 1:numel(joints)
    jtype = joints{j}.type;
    jbody = joints{j}.bodies;
    
    bposi = bodypos(jbody,x);

    if strcmp(jtype,'revolute')
        jposi = joints{j}.positions;
        C = [C
            bposi(1,1) - bposi(1,2) + jposi(1,1)*cos(bposi(3,1)) - jposi(1,2)*cos(bposi(3,2)) - jposi(2,1)*sin(bposi(3,1)) + jposi(2,2)*sin(bposi(3,2))
            bposi(2,1) - bposi(2,2) + jposi(1,1)*sin(bposi(3,1)) - jposi(1,2)*sin(bposi(3,2)) + jposi(2,1)*cos(bposi(3,1)) - jposi(2,2)*cos(bposi(3,2))];
    
    elseif strcmp(jtype,'translational')
        jposi = joints{j}.positions;
        jnorm = joints{j}.normal;
        bposi0 = joints{j}.init;
        C = [C
            (jnorm(1)*cos(bposi(3,1)) - jnorm(2)*sin(bposi(3,1)))*(bposi(1,1) - bposi(1,2) + jposi(1,1)*cos(bposi(3,1)) - jposi(1,2)*cos(bposi(3,2)) - jposi(2,1)*sin(bposi(3,1)) + jposi(2,2)*sin(bposi(3,2))) + (jnorm(2)*cos(bposi(3,1)) + jnorm(1)*sin(bposi(3,1)))*(bposi(2,1) - bposi(2,2) + jposi(2,1)*cos(bposi(3,1)) - jposi(2,2)*cos(bposi(3,2)) + jposi(1,1)*sin(bposi(3,1)) - jposi(1,2)*sin(bposi(3,2)))
			bposi(3,1) - bposi(3,2) - (bposi0(3,1)-bposi0(3,2))];
    
    elseif strcmp(jtype,'1DOF')
        bposi0 = joints{j}.init;
        C = [C
            bposi(joints{j}.dof,1) - bposi(joints{j}.dof,2) - (bposi0(joints{j}.dof,1)-bposi0(joints{j}.dof,2))];
    end
end

% Time-dependent constraints
for j = 1:numel(constraints)
    bdof = bcoords(data.constraints(j).body);
    cdof = bdof(data.constraints(j).dof);
    cexpr = data.constraints(j).expression;
    
    C = [C; cexpr(t)-x(cdof)];
end

end