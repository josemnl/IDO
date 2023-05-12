function X = linearTrajectoryVel(p1,p2,vel)
    dist = sqrt((p1(1)-p2(1))^2+(p1(2)-p2(2))^2);
    n = ceil(dist/vel)+1;
    x = linspace(p1(1),p2(1),n)';
    y = linspace(p1(2),p2(2),n)';
    z = linspace(0,0,n)';
    X = [x y z];
end