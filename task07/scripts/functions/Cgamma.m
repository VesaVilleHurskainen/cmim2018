% Constraint equations for acceleration computation
function Cg = Cgamma(x,xd,t,data)

joints = data.joints;
constraints = data.constraints;

Cqq = [];
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
        xja = jposi(1,1);
        yja = jposi(2,1);
        xjb = jposi(1,2);
        yjb = jposi(2,2);
        ta = bpos(3,1);
        tb = bpos(3,2);
        tad = bvel(3,1);
        tbd = bvel(3,2);
        Cqq = [Cqq
           (yja*sin(ta) - xja*cos(ta))*tad^2 + (xjb*cos(tb) - yjb*sin(tb))*tbd^2
         (- yja*cos(ta) - xja*sin(ta))*tad^2 + (yjb*cos(tb) + xjb*sin(tb))*tbd^2];
    end
end
for j = 1:numel(constraints)   
    Cqq = [Cqq; 0];
end


Ctt = [];
for j = 1:numel(joints)
    if strcmp(jtype,'revolute')
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