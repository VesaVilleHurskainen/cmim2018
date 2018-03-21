function J = jacobian_fdiff(fun,y,h)

n = length(y);
J = zeros(n,n);

for i = 1:n
    y1 = y;
    y1(i) = y1(i)+h;
    f1 = fun(y1);
    J(:,i) = (f1-fun(y))/h;
end