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

% Solve response
fpos = dsolve(EQM,[x(0) == x0, xd(0) == xd0]);

% Substitute
syms cc zeta omega real
assumeAlso(cc>0)
assumeAlso(zeta>0)
assumeAlso(omega>0)

fpos_rewrite1 = simplify(subs(expand(fpos), c, zeta*cc));
fpos_rewrite2 = simplify(subs(expand(fpos_rewrite1), 4*k*m, cc*cc));
fpos_rewrite3 = simplify(subs(expand(fpos_rewrite2), cc, 2*sqrt(k*m)));
fpos_rewrite4 = simplify(subs(expand(fpos_rewrite3), sqrt(k/m), omega));

disp('Symbolically derived response function:')
pretty(fpos_rewrite4);


% Numerical solution parameters
tspan = [0,20];
A_p = -1;
omega_p = 2;
zeta_p = 0.99;
m_p = 1;

% Force function
f_p = 1*sin(1*t);

% Solve response numerically
k_p = omega_p*m_p;
fsub = simplify(expand(subs(fpos_rewrite4,[A,omega,zeta,m,k,f],[A_p, omega_p, zeta_p, m_p, k_p, f_p])));

% Plot response
fplot(fsub,tspan)
title('Butts')
legend('datas')
grid on

filename = 'plot';
print(filename,'-dpdf')
% formatplot(gcf,filename)