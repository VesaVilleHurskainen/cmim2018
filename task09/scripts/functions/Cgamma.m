% Constraint equations for acceleration computation
function Cg = Cgamma(x,xd,t,data)

joints = data.joints;
constraints = data.constraints;

Cqq = [];
for j = 1:numel(joints)
    jtype = joints{j}.type;
    jbody = joints{j}.bodies;
    jposi = joints{j}.positions;
    
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

    xa = bpos(1,1);
    ya = bpos(2,1);
    ta = bpos(3,1);
    xb = bpos(1,2);
    yb = bpos(2,2);
    tb = bpos(3,2);
    xja = jposi(1,1);
    yja = jposi(2,1);
    xjb = jposi(1,2);
    yjb = jposi(2,2);
    xad = bvel(1,1);
    yad = bvel(2,1);
    tad = bvel(3,1);
    xbd = bvel(1,2);
    ybd = bvel(2,2);
    tbd = bvel(3,2);

    if strcmp(jtype,'revolute')
        Cqq = [Cqq
           (yja*sin(ta) - xja*cos(ta))*tad^2 + (xjb*cos(tb) - yjb*sin(tb))*tbd^2
         (- yja*cos(ta) - xja*sin(ta))*tad^2 + (yjb*cos(tb) + xjb*sin(tb))*tbd^2];
    elseif strcmp(jtype,'translational')
        jnorm = joints{j}.normal;
        Cqq = [Cqq
			tad^2*xa*jnorm(2)*sin(ta) - tad^2*jnorm(1)*ya*sin(ta) - tad^2*xb*jnorm(2)*sin(ta) + tad^2*jnorm(1)*yb*sin(ta) + tad^2*xjb*jnorm(1)*cos(ta - tb) + tbd^2*xjb*jnorm(1)*cos(ta - tb) + tad^2*yjb*jnorm(2)*cos(ta - tb) + tbd^2*yjb*jnorm(2)*cos(ta - tb) - tad^2*xjb*jnorm(2)*sin(ta - tb) + tad^2*jnorm(1)*yjb*sin(ta - tb) - tbd^2*xjb*jnorm(2)*sin(ta - tb) + tbd^2*jnorm(1)*yjb*sin(ta - tb) - 2*tad*xad*jnorm(2)*cos(ta) + 2*tad*jnorm(1)*yad*cos(ta) + 2*tad*xbd*jnorm(2)*cos(ta) - 2*tad*jnorm(1)*ybd*cos(ta) - 2*tad*xad*jnorm(1)*sin(ta) + 2*tad*xbd*jnorm(1)*sin(ta) - 2*tad*yad*jnorm(2)*sin(ta) + 2*tad*ybd*jnorm(2)*sin(ta) - tad^2*xa*jnorm(1)*cos(ta) + tad^2*xb*jnorm(1)*cos(ta) - tad^2*ya*jnorm(2)*cos(ta) + tad^2*yb*jnorm(2)*cos(ta) - 2*tad*tbd*xjb*jnorm(1)*cos(ta - tb) - 2*tad*tbd*yjb*jnorm(2)*cos(ta - tb) + 2*tad*tbd*xjb*jnorm(2)*sin(ta - tb) - 2*tad*tbd*jnorm(1)*yjb*sin(ta - tb)
			0];
    end
end
for j = 1:numel(constraints)   
    Cqq = [Cqq; 0];
end

Ctt = [];
for j = 1:numel(joints)
    jtype = joints{j}.type;
    if strcmp(jtype,'revolute')
        Ctt = [Ctt; zeros(2,1)];
    elseif strcmp(jtype,'translational')
        Ctt = [Ctt; zeros(2,1)];
    end
end
for j = 1:numel(constraints)
    cddiff = data.constraints(j).ddiff;
    
    Ctt = [Ctt; cddiff(t)];
end

Cqt = zeros(size(Ctt));

Cg = - Cqq - Cqt - Ctt;

end