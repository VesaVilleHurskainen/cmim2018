function M = massMatrix(m,L)

M = diag([m,m,m*L^2/12,m,m,m*L^2/12]);