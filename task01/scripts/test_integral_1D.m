% Computational Methods in Mechanics 2018
% Task 1, 1D test script
% VVH

% Comparison of integration algorithms in one-dimensional case
fun = @(x) sin(x);

x0 = 0;
x1 = 1;
ns = 1e6;

rounds = 10;
type = 2;

profile on
for r = 1:rounds
    if type == 1        % Matlab built-in trapezoidal rule
        result = (x1-x0)/n*trapz(fun(linspace(x0,x1,ns)));
    elseif type == 2    % Original code from Grzegorz
        result = integral_trapezoid(fun,x0,x1,ns);
    elseif type == 3    % Optimized code (by vectorization)
        result = integral_trapezoid_optimized(fun,x0,x1,ns);
    end
end
profile report

disp(result)