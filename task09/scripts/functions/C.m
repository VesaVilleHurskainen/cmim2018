% Routine to compute constraint vector C
function C = C(x,t,data)

C = [];

joints = data.joints;
constraints = data.constraints;

% Joints
for j = 1:numel(joints)
    jtype = joints{j}.type;
    jbody = joints{j}.bodies;
    jposi = joints{j}.positions;
    
    if jbody(1) == 0
        bposi = [[0;0;0], x(bcoords(jbody(2)))];
    elseif jbody(2) == 0
        bposi = [x(bcoords(jbody(1))), [0;0;0]];
    else
        bposi = [x(bcoords(jbody(1))), x(bcoords(jbody(2)))];
    end

    if strcmp(jtype,'revolute')
        C = [C
            bposi(1,1) - bposi(1,2) + jposi(1,1)*cos(bposi(3,1)) - jposi(1,2)*cos(bposi(3,2)) - jposi(2,1)*sin(bposi(3,1)) + jposi(2,2)*sin(bposi(3,2))
            bposi(2,1) - bposi(2,2) + jposi(1,1)*sin(bposi(3,1)) - jposi(1,2)*sin(bposi(3,2)) + jposi(2,1)*cos(bposi(3,1)) - jposi(2,2)*cos(bposi(3,2))];
    elseif strcmp(jtype,'translational')
        jnorm = joints{j}.normal;
        if jbody(1) == 0
            bposi0 = [[0;0;0], data.x0(bcoords(jbody(2)))];
        elseif jbody(2) == 0
            bposi0 = [data.x0(bcoords(jbody(1))), [0;0;0]];
        else
            bposi0 = [data.x0(bcoords(jbody(1))), data.x0(bcoords(jbody(2)))];
        end
        C = [C
            (jnorm(1)*cos(bposi(3,1)) - jnorm(2)*sin(bposi(3,1)))*(bposi(1,1) - bposi(1,2) + jposi(1,1)*cos(bposi(3,1)) - jposi(1,2)*cos(bposi(3,2)) - jposi(2,1)*sin(bposi(3,1)) + jposi(2,2)*sin(bposi(3,2))) + (jnorm(2)*cos(bposi(3,1)) + jnorm(1)*sin(bposi(3,1)))*(bposi(2,1) - bposi(2,2) + jposi(2,1)*cos(bposi(3,1)) - jposi(2,2)*cos(bposi(3,2)) + jposi(1,1)*sin(bposi(3,1)) - jposi(1,2)*sin(bposi(3,2)))
			bposi(3,1) - bposi(3,2) - (bposi0(3,1)-bposi0(3,2))];
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