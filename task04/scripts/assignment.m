clear
close all

% Integrator and time settings
integrator = 'ode45';

t0 = 0;
t1 = 1;
tstep = 0.001;

tspan = t0:tstep:t1;

% System parameters
m1 = 2;
m2 = 1.5;
m3 = 1;
k1 = 20000;
k2 = 15000;
k3 = 10000;

% Starting positions and velocities
y0 = [-0.1,0,0.1,0,0,0]';

% System mass matrix (point mass system)
M = diag([m1, m2, m3]);

% System stiffness matrix (derived by hand)
K = [k1+k2, -k2,     0
    -k2,     k2+k3, -k3
     0,     -k3,     k3];
 
% Natural frequencies of the system
[V,A] = eig(K,M);
omega = sqrt(diag(A));
disp(omega)

% Transform to modal coordinates
Mw = V'*M*V;
Kw = V'*K*V;

% Verify transofrmed matrices
disp('Modal mass matrix verification: Mw - I =')
disp(Mw - eye(3))

disp('Modal stiffness matrix verification: Kw - A =')
disp(Kw - A)

% Initial conditions in modal coordinates
p0 = [V zeros(size(V)); zeros(size(V)) V] \ y0; 

% Formulate as function and solve with integrator of choice
fun = @(t,y)[y(4:6); -(M\K)*y(1:3)];
fun_modal = @(t,p)[p(4:6); -(Mw\Kw)*p(1:3)];

if strcmp(integrator,'rungeKutta4')
    [t,y] = rungeKutta4(fun,tspan,y0);
    [tp,p] = rungeKutta4(fun_modal,tspan,p0);
    
elseif strcmp(integrator,'simEuler')
    u0 = y0(1:3);
    v0 = y0(4:6);
    up0 = p0(1:3);
    vp0 = p0(4:6);
    fun_f = @(t,v) v;
    fun_g = @(t,u) -(M\K)*u;
    fun_f_modal = @(t,vp) vp;
    fun_g_modal = @(t,up) -(Mw\Kw)*up;
    
    [t,u,v] = simEuler(fun_f,fun_g,tspan,u0,v0);
    [tp,up,vp] = simEuler(fun_f_modal,fun_g_modal,tspan,up0,vp0);
    y = [u;v];
    
elseif strcmp(integrator,'ode45')   
    [t,y] = ode45(fun,tspan,y0);
    [tp,p] = ode45(fun_modal,tspan,p0);
    y = y';
    p = p';
    
elseif strcmp(integrator,'ode15s')   
    [t,y] = ode15s(fun,tspan,y0);
    [tp,p] = ode15s(fun_modal,tspan,p0);
    y = y';
    p = p';
end

% Transform back from modal coordinates
y_modal = zeros(size(p));
for i = 1:length(tp)
    y_modal(:,i) = [V zeros(size(V)); zeros(size(V)) V] * p(:,i);
end

% Plot results
figure
plot(t,y(1:3,:));
grid on

figure
plot(tp,y_modal(1:3,:));
grid on