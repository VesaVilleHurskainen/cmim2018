clear
close all

x = sym('x', [4,1], 'real');
syms m c k L t real

% Set time parameters
t0 = 0;
t1 = 3;
tstep = 0.001;

tspan = t0:tstep:t1;

% System equation of motion (derived separately)
Eqs = [x(3)
       x(4)
[3*m, L*m*cos(x(2)); L*m*cos(x(2)), 2*L^2*m/3]\...
(-[- L*m*sin(x(2))*x(4)^2 + 2*L*c*cos(x(2))*x(4) + 5*c*x(3) + 3*k*x(1) + 2*L*k*sin(x(2))
c*x(3) + 2*L^2*k*cos(x(2))*sin(x(2)) + 2*L*k*cos(x(2))*x(1) + 2*L*c*cos(x(2))*(x(3) + L*x(4)*cos(x(2))) - L*m*x(4)*x(3)*sin(x(2))])];
 

% System jacobian
J = jacobian(Eqs,x);

% Set parameters and transfrom syms -> function
mp = 1;
cp = 0.5;
kp = 100;
Lp = 1;

x0 = [0.1,0,0,0]';

fun = matlabFunction(subs(Eqs,[m,c,k,L],[mp,cp,kp,Lp]),'Vars',{t,x});
jac = matlabFunction(subs(J,[m,c,k,L],[mp,cp,kp,Lp]),'Vars',{x});

% Solve using backward Euler method and ode15s
tol = 1e-6;
maxiter = 1e3;
[tbe,xbe] = bwdEuler(fun,tspan,x0,jac,tol,maxiter);
[t15,x15] = ode15s(fun,tspan,x0);


% Plot responses
figure
plot(tbe,xbe(1:2,:));
hold on
plot(t15,x15(:,1:2)');
hold off

% Plot difference
figure
plot(tspan,x15(:,1:2)'-xbe(1:2,:));

% Animate system for verification
% Set body dimensions for drawing
H_box = 0.2;
W_box = 0.3;
W_rod = 0.05;

figure
for i = 1:length(tspan)
    % Get positions
    x_box = xbe(1,i);
    t_rod = xbe(2,i);
    
    % Clear figure and draw box
    clf
    rbox = [x_box-W_box/2, x_box+W_box/2, x_box+W_box/2, x_box-W_box/2
           -H_box/2, -H_box/2, H_box/2, H_box/2];
    patch(rbox(1,:),rbox(2,:),'red')
    
    % Draw rod (using rotation matrix)
    rot = [cos(t_rod) -sin(t_rod); sin(t_rod) cos(t_rod)];
    rrod = [x_box*ones(1,4); zeros(1,4)] ...
            + rot*[-W_rod/2, W_rod/2, W_rod/2, -W_rod/2; 0, 0, -Lp, -Lp];
    patch(rrod(1,:),rrod(2,:),'blue')
    
    % Format figure
    title(['Time: ',num2str(tspan(i))])
    axis equal
    grid on
    ylim([-1.05 0.15])
    drawnow
end