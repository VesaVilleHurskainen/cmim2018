clear
close all

% Define symbolic variables ( x(1) = r, x(2) = phi )
x = sym('x',[2,1]);

% Define equations and Jacobian
eq = [x(1)*sin(x(2)); x(1)*cos(x(2))] - [4; 3];
J = jacobian(eq,x);

% Transform from symbolic equation to function
fun = matlabFunction(eq,'Vars',{x});
jac = matlabFunction(J,'Vars',{x});

% Solve using Newton-Raphson
x0 = [4; pi/4];
sol = newtonRaphson(fun,x0,jac,1e-6,1000);

% Display results
disp('Coordinates q:')
disp(sol)