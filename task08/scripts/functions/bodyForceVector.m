% Routine to determine generalized (gravity) force vector for one body
function Qb = bodyForceVector(body,g)

type = body.type;
m = body.mass;

if strcmp(type,'slenderRod')
    Qb = [m*g;0];
end