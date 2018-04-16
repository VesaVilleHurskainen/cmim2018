% Simple implementation of Newton-Raphson method for solution of nonlinear equations.
function [x,i] = newtonRaphson(fun, x0, jac, tol, maxiter)

x = x0;
i = 0;

while any(abs(fun(x)) > tol)
    x = x - jac(x)\fun(x);
    i = i+1;
    if i >= maxiter
        disp('newtonRaphson reached maximum number of iterations, tolerance not met!');
        return
    end
end