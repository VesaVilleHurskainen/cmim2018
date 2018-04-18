%

pB = [2;4];
pC = [-3;4];

x0 = [pB; pC];

A = eye(4);

b = [4 7 -1 4]';

lb = [0 4 -5 1]';

fmincon(@optFun,x0,A,b,[],[],lb,b);