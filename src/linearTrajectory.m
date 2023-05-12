function X = linearTrajectory(p1,p2,n)
    x = linspace(p1(1),p2(1),n)';
    y = linspace(p1(2),p2(2),n)';
    X = [x y];
end