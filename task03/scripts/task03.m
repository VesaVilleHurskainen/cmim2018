clear
close all

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

% Solve response
fpos = dsolve(EQM,[x(0) == x0, xd(0) == xd0]);

% Substitute
syms cc zeta omega F omega_f real
assumeAlso(cc>0)
assumeAlso(zeta>0)
assumeAlso(omega>0)
assumeAlso(omega_f>0)

fpos_rewrite1 = simplify(subs(expand(fpos), c, zeta*cc));
fpos_rewrite2 = simplify(subs(expand(fpos_rewrite1), 4*k*m, cc*cc));
fpos_rewrite3 = simplify(subs(expand(fpos_rewrite2), cc, 2*sqrt(k*m)));
fpos_rewrite4 = simplify(subs(expand(fpos_rewrite3), sqrt(k/m), omega));
fpos_rewrite5 = simplify(subs(expand(fpos_rewrite4), m, k/omega^2));

disp('Symbolically derived response function:')
pretty(fpos_rewrite5);


% Force function
f_p = F*sin(omega_f*t);

% Substitute force function, harmonic force
fpos_harmonic = simplify(subs(expand(fpos_rewrite5), f, f_p));

% Numerical solution parameters
tspan = [0,10];
omega_p = 2;
omega_fp = 2;
zeta_p = 0.5;
k_p = 10;
F_p = 10;
A_p = 0;

% Steady state response
ampX = @(omega_f,zeta) F_p/(k_p*sqrt((1-omega_f^2/omega_p^2)^2+(2*zeta*omega_f/omega_p)^2));
phi = @(omega_f,zeta) atan((-2*zeta*omega_f/omega_p)/(1-omega_f^2/omega_p^2));
respfun = @(omega_f,zeta,t) ampX(omega_f,zeta)*sin(omega_f*t+phi(omega_f,zeta));

% Substitute parameters to symbolically solved response
respfun_sym = simplify(expand(subs(fpos_harmonic,[A,omega,omega_f,zeta,k,F],[A_p, omega_p, omega_fp, zeta_p, k_p, F_p])));

% Compare steady state response to symbolically solved one
figure
fplot(respfun_sym,tspan,'k-','LineWidth',1.5);
hold on
respfun_sub = @(t) respfun(omega_fp,zeta_p,t);
fplot(respfun_sub,tspan,'k--','LineWidth',1.5)
hold off
legend('Response','Steady state')
ylabel('Position')
xlabel('Time')
formatPlot(gcf,'Times New Roman',12)

% Save figure
filename = 'response';
print(filename,'-depsc');


% Generate amplitude figure
omegavec = 0:0.01:5;
zetavec = [0,0.1,0.2,0.3,0.5,1];

freqratio = omegavec/omega_p;
ampratio = zeros(length(omegavec),length(zetavec));
phaseangle = ampratio;

for i = 1:length(omegavec)
    for j = 1:length(zetavec)
        ampratio(i,j) = ampX(omegavec(i),zetavec(j))*k_p/F_p;
        phaseangle(i,j) = phi(omegavec(i),zetavec(j));
        if phaseangle(i,j)>0; phaseangle(i,j) = phaseangle(i,j) - pi; end
    end
end

figure
plot(freqratio,ampratio,'k-','LineWidth',1.5)
ylim([0,7.5])
xlabel('Frequency ratio $\frac{\omega}{\omega_n}$','Interpreter','latex')
ylabel('Amplitude ratio $X \frac{k}{F}$','Interpreter','latex')

annotation('textarrow',[0.6,0.52],[0.3,0.16],'String','$\zeta = 1$','Interpreter','latex')
annotation('textarrow',[0.58,0.48],[0.4,0.21],'String','$\zeta = 0.5$','Interpreter','latex')
annotation('textarrow',[0.57,0.45],[0.5,0.29],'String','$\zeta = 0.3$','Interpreter','latex')
annotation('textarrow',[0.55,0.44],[0.6,0.39],'String','$\zeta = 0.2$','Interpreter','latex')
annotation('textarrow',[0.55,0.45],[0.71,0.64],'String','$\zeta = 0.1$','Interpreter','latex')
annotation('textarrow',[0.55,0.47],[0.8,0.8],'String','$\zeta = 0$','Interpreter','latex')

formatPlot(gcf,'Times New Roman',12)

% Save figure
filename = 'amplitude';
print(filename,'-depsc');

figure
plot(freqratio,phaseangle,'k-','LineWidth',1.5)
ylim([-pi,0])
xlabel('Frequency ratio $\frac{\omega}{\omega_n}$','Interpreter','latex')
ylabel('Phase angle $\phi$','Interpreter','latex')

annotation('textarrow',[0.8,0.75],[0.4,0.16],'String','$\zeta = 0.1$','Interpreter','latex')
annotation('textarrow',[0.75,0.66],[0.5,0.21],'String','$\zeta = 0.2$','Interpreter','latex')
annotation('textarrow',[0.67,0.55],[0.6,0.32],'String','$\zeta = 0.3$','Interpreter','latex')
annotation('textarrow',[0.6,0.49],[0.71,0.45],'String','$\zeta = 0.5$','Interpreter','latex')
annotation('textarrow',[0.55,0.46],[0.8,0.51],'String','$\zeta = 1$','Interpreter','latex')

formatPlot(gcf,'Times New Roman',12)

% Save figure
filename = 'phaseangle';
print(filename,'-depsc');