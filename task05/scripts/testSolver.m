fun = @(x) x.^2-9;
points = -10:0.01:10;

roots = bruteForce(fun,points)