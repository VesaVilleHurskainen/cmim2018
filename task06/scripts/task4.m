clear
close all

% Define symbolic variables ( q(1) = r, q(2) = phi )
q = sym('q',[2,1]);

% Define equations and Jacobian
eq = [q(1)*sin(q(2)); q(1)*cos(q(2))] - [4; 3];
J = jacobian(eq,q);

% Transform from symbolic equation to function
fun = matlabFunction(eq,'Vars',{q});
jac = matlabFunction(J,'Vars',{q});

% Solve using Newton-Raphson
x0 = [4; pi/4];
sol = newtonRaphson(fun,x0,jac,1e-6,1000);

% Display results
disp('Coordinates q:')
disp(sol)