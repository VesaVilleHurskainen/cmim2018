% Script to optimize grabber
% April 2018, Vesa-Ville Hurskainen

% Set initial value
pB = [2;4];
pC = [-3;4];
x0 = [pB; pC];

% Set limits
ub = [4 7 -1 4]';
lb = [0 4 -5 1]';

% Set optimization options
% opts = optimoptions('fmincon', 'Algorithm', 'interior-point','MaxFunctionEvaluations',1000);
% opts = optimoptions('fmincon', 'Algorithm', 'sqp','MaxFunctionEvaluations',1000);
opts = optimoptions('fmincon', 'Algorithm', 'active-set','MaxFunctionEvaluations',1000);

% Optimize
[x,fval,exitflag,output] = fmincon(@optFun,x0,[],[],[],[],lb,ub,[],opts);
disp(x)
disp(output)