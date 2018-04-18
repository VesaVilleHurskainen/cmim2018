% Helper function for setting coordinates according to body numbers
function bpos = bodypos(bodies,x)

if bodies(1) == 0
        bpos = [[0;0;0], x(bcoords(bodies(2)))];
    elseif bodies(2) == 0
        bpos = [x(bcoords(bodies(1))), [0;0;0]];
    else
        bpos = [x(bcoords(bodies(1))), x(bcoords(bodies(2)))];
end

end