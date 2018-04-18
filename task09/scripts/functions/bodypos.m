% Helper function to create 2 body position vectors from global pos. vec.
function bpos = bodypos(bodies,x)

if bodies(1) == 0
        bpos = [[0;0;0], x(bcoords(bodies(2)))];
    elseif bodies(2) == 0
        bpos = [x(bcoords(bodies(1))), [0;0;0]];
    else
        bpos = [x(bcoords(bodies(1))), x(bcoords(bodies(2)))];
end

end