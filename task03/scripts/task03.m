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
EQM = m*xdd + c*xd + k*x == f;

% Solve
fpos = dsolve(EQM,[x(0) == x0, xd(0) == xd0])

syms cc zeta omega real

fpos_rewrite = simplify(expand(subs(subs(simplify(subs(subs(fpos, 4*k*m, cc*cc), c, zeta*cc)),cc,2*sqrt(k*m)),sqrt(k/m),omega)))

% Numerical solution
fsub = subs(fpos_rewrite,[A,m,k,c],[0.1,1,100,0.1])