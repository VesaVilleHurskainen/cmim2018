% Routine to build total force vector. Note: only gravity forces implemented currently.
function Q = totalForceVector(~,data)

bodies = data.bodies;

Q = zeros(3*numel(bodies),1);

for i = 1:numel(bodies)
    Q(bcoords(i)) = Q(bcoords(i)) + bodyForceVector(bodies(i),data.g);
end

end