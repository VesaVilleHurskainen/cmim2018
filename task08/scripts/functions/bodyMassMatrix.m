% Routine to determine mass matrix of a single ridig body
function Mb = bodyMassMatrix(body)

type = body.type;

if strcmp(type,'slenderRod') 
    L = body.length;
    m = body.mass;
    
    Mb = diag([m, m, m*L^2/12]);
end

end