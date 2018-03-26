% Simple animated visualization of displacements with regard to time
function [] = pproc_animate(x,t,data)

bodies = data.bodies;

figure

for i = 1:length(t)
    clf
    axis equal
    grid on
    hold on
    for j = 1:numel(bodies)
        body = bodies(j);
        xb = x(bcoords(j),i);
        
        if strcmp(body.type,'slenderRod')
            r1 = [xb(1);xb(2)] + [cos(xb(3)) -sin(xb(3)); sin(xb(3)) cos(xb(3))]*[-body.length/2;0];
            r2 = [xb(1);xb(2)] + [cos(xb(3)) -sin(xb(3)); sin(xb(3)) cos(xb(3))]*[body.length/2;0];
            plot([r1(1),r2(1)],[r1(2),r2(2)])
        end
    end
    title(['t = ',num2str(t(i),'%2.3f')]);
    hold off
    drawnow
end

end