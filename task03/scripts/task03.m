clear

syms m c k A real
assumeAlso(m,'positive')
assumeAlso(k,'positive')
assumeAlso(c,'positive')

syms x(t) f(t)
assume(t,'real')
assumeAlso(t>=0)
xd = diff(x,t);
xdd = diff(xd,t);

% Set initial conditions
x0 = A;
xd0 = 0;

% Equation of motion
EQM = m*xdd + c*xd + k*x == 0;

% Solve
fpos = dsolve(EQM,[x(0) == x0, xd(0) == xd0])

% Numerical soulution
fsub = subs(fpos,[A,m,k,c],[0.1,1,100,0.1])

fplot(fsub)