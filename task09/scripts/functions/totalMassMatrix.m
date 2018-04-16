% Routine to build total mass matrix.
function M = totalMassMatrix(~,data)

bodies = data.bodies;

M = zeros(3*numel(bodies));

for i = 1:numel(bodies)
    M(bcoords(i),bcoords(i)) = M(bcoords(i),bcoords(i)) + bodyMassMatrix(bodies{i});
end

end