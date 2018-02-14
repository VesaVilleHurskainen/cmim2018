function roots = bruteForce(fun, x)

y = fun(x);

% First, find roots exactly on investigated points
roots = x(y==0);

% Then, find roots in intervals using brute force method
i = find(y(1:end-1).*y(2:end) < 0);
roots = [roots, x(i)-((x(i+1)-x(i))./(y(i+1)-y(i))).*y(i)];

% Sort roots smallest to largest
roots = sort(roots);