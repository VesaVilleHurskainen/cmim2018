clear

m = 2;
L = 1;

g = [0; -9.81];

M = massMatrix(m,L);
Q = forceVector(m,g);

fun = @(t,y) [y(7:12); M\Q];

q0 = [L/2 0 0 3*L/2 0 0]';
q0d = zeros(size(q0));
y0 = [q0;q0d];

tspan = 0:0.01:1;

[t,y] = ode45(fun,tspan,y0);

% Verification of task 3
syms sL real
sy = sym('y',[12,1]);

[C, Cq, Ct, G] = constraints(sy, sL);

tCq = jacobian(C, sy(1:6));
disp(simplify(tCq - Cq))