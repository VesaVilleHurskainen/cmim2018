% Routine to build total force vector. Note: only gravity and point forces implemented currently.
function Q = totalForceVector(~,x,data)

bodies = data.bodies;
forces = data.forces;

Q = zeros(3*numel(bodies),1);

for i = 1:numel(bodies)
    bforces = {};
    for j = 1:numel(forces)
        if forces{j}.body == i
            bforces = [bforces, forces{j}];
        end
    end
    xb = x(bcoords(i));
    Q(bcoords(i)) = Q(bcoords(i)) + bodyForceVector(bodies{i},bforces,xb,data.g);
end

end