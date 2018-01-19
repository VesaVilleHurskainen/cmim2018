function result = integral_trapezoid_optimized( fun, low_limit, up_limit, no_splits )
%INTEGRAL_TRAPEZOID Calculate the integral of the 1D function.
%   Function that calculate the integral of 1D continous function using
%   well-known trapezoidal rule. Naive approach.
%   fun - handle of a function to integrate,
%   low_limit - lower limit of an integral,
%   up_limit - upper limit of an integral,
%   no_splits - number of trapezoids.

% integration step
h = (up_limit - low_limit) / no_splits;

% points
p = low_limit:h:up_limit;

% function values at points
fi = fun(p);

% result
result = 0.5*h*sum(fi(1:end-1)+fi(2:end));