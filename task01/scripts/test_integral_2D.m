% Computational Methods in Mechanics 2018
% Task 1, 2D test script
% VVH

% Two-dimensional integration using code from Grzegorz
fun = @(x,y) sin(x) + cos(y);

x0 = 0;
x1 = 1;
y0 = 0;
y1 = 1;
ns = 100;

rounds = 2;
type = 3;

profile on
for r = 1:rounds
    if type == 1
        result = integral2(fun,x0,x1,y0,y1);
    elseif type == 2
        fun_intx = @(y) integral_trapezoid(@(x) fun(x,y),x0,x1,ns);
        result = integral_trapezoid(fun_intx,y0,y1,ns);
    end
end
profile report

disp(result)