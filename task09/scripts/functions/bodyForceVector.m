% Routine to determine generalized force vector for one body
function Qb = bodyForceVector(body,forces,x,g)

type = body.type;
m = body.mass;

if strcmp(type,'slenderRod')
    % Force due to gravity
    Qb = [m*g;0];
    
    % Generalized point forces
    for i=1:numel(forces)
        Ff = forces{i}.forcevector;
        uf = forces{i}.location;
        Qb = Qb + [Ff; Ff'*[cos(x(3)) -sin(x(3)); sin(x(3)) cos(x(3))]*uf];
    end
end