% ODE function for dynamics solution using Baumgarte stabilization
function dydt = dynamicsFunction(t,y,data)

ncoords = 3*numel(data.bodies);
yp = y(1:ncoords,1);
yd = y(ncoords+1:ncoords*2,1);

alpha = data.alpha;
beta = data.beta;

M = totalMassMatrix(t,data);
Q = totalForceVector(t,yp,data);

if data.consts > 0
    Cdot = Cq(yp,data)*yd + Ct(t,data);
else
    Cdot = [];
end

G = Cgamma(yp,yd,t,data);

LHS = [M, Cq(yp,data)'; Cq(yp,data), zeros(length(Cdot))];

RHS = [Q; G - 2*alpha*Cdot - beta^2*C(yp,t,data)];

dydt = [yd; LHS\RHS];

end