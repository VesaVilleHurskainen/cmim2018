clear
syms m L y t k c real;
syms x(t);
syms theta(t);

% Compute Lagrangian
% T = (1/2)*m*diff(x,t)^2 + (1/2)*int((2*m/L)*diff(x+y*sin(theta),t)^2,y,0,L) + (1/2)*int((2*m/L)*diff(y*cos(theta),t)^2,y,0,L);
T = (1/2)*m*diff(x,t)^2 + (1/2)*int((2*m/L)*(diff(x+y*sin(theta),t)^2 + diff(y*cos(theta),t)^2),y,0,L);
U = (1/2)*k*x^2 + k*(x+L*sin(theta))^2;
Lag = simplify(T-U);

% Derive Lagrange equation
syms v omega xs thetas
dif1 = jacobian(subs(Lag,[diff(x,t),diff(theta,t)],[v,omega]),[v;omega])
dif2 = diff(subs(dif1,[v,omega],[diff(x,t),diff(theta,t)]),t)
dif3 = subs(jacobian(subs(Lag,[x,theta],[xs,thetas]),[xs;thetas]),[xs,thetas],[x,theta])
Q = subs(jacobian(c*v^2+c*(v+L*omega*cos(theta))^2,[v,omega]),[v,omega],[diff(x,t),diff(theta,t)])

eq = simplify(dif2 - dif3 + c*diff(x,t) + Q)

% Simplify for presentation
syms xd xdd thetad thetadd real
simplify(subs(eq,[diff(x,t),diff(theta,t),diff(x,t,t),diff(theta,t,t)],[xd,thetad,xdd,thetadd]))