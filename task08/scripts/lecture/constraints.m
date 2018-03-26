function [C, Cq, Ct, G] = constraints(y,L)

r1 = y(1:2);
t1 = y(3);
r2 = y(4:5);
t2 = y(6);
u1_1 = [-L/2; 0];
u1_2 = [L/2; 0];
u2_2 = u1_1;
A1 = rot(t1);
A2 = rot(t2);

C1 = r1 + A1*u1_1;
C2 = r1 + A1*u1_2 - (r2 + A2*u2_2);

C = [C1;C2];

I2 = eye(2);
Om = [0, -1; 1, 0];
Cq = [I2, Om*A1*u1_1, zeros(2,3)
    I2, Om*A1*u2_1, -I2, Om*A2*u2_2];

qt = y(7:12);

Ct = Cq*qt;

t1t = qt(3);
t2t = qt(6);

G = -[-A1*u1_1*t1t^2
    -A1*u2_1*t1t^2 + A2*u2_2*t2t^2

end


function A = rot(t)

A = [cos(t) -sin(t); sin(t) cos(t)];

end