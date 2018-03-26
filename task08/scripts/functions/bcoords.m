% Helper function to calculate global coordinate numbers from body number
function q = bcoords(bnum)

q = 3*(bnum-1)+1 : 3*bnum;