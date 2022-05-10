clf;
% Create meshgrid for contour and initialize symbolic toolbox
[xG, yG] = meshgrid(-1.5:0.1:2.5, -3.37:0.1:1);
syms xN yN a b
poteq = log(sqrt((xN-a).^2 +(yN-b).^2));
f_contour = 0;

[f, fx, fy] = deal(0);

% 0.07 was chosen as an arbitrary constant to get the contour to be steep
% enough. This constant is used to change the agressivness of the obstacle
% sources.
arbitrary_constant = 0.06;

% % Add sources for the walls
for i = -3.07:0.05:0.7
    f = f + log(sqrt((yG-i).^2 + (xG+1.2).^2)) + log(sqrt((yG-i).^2 + (xG-2.2).^2));
    f_contour = f_contour + subs(poteq, [a,b], [-1.2,i]) + subs(poteq, [a,b], [2.2,i]);
end
for i = -1.2:0.05:2.2
    f = f + log(sqrt((xG-i).^2 + (yG+3.07).^2)) + log(sqrt((xG-i).^2 + (yG-0.7).^2));
    f_contour = f_contour + subs(poteq, [a,b], [i,-3.07]) + subs(poteq, [a,b], [i,0.7]);
end

% Add sources for the obstacles
square_centers = [-0.25, -1; 1, -0.7; 1.41, -2];
for i = 1:length(square_centers)
    for theta = 0:0.4:2*pi
        aS = 0.353*cos(theta) + square_centers(i,1);
        bS = 0.353*sin(theta) + square_centers(i,2);
        f = f - arbitrary_constant*log(sqrt((xG-aS).^2 + (yG-bS).^2));
        f_contour = f_contour - subs(poteq, [a,b], [aS,bS]);
    end
end

% Add a sink for the BoB
for theta = 0:0.4:2*pi
    aB = 0.25*cos(theta) + 0.75;
    bB = 0.25*sin(theta) - 2.5;
    f = f + log(sqrt((xG-aB).^2 + (yG-bB).^2));
    f_contour = f_contour + subs(poteq, [a,b], [aB,bB]);
end

f_contour % Contour equation
contour3(xG, yG, f, 100)
title("Gauntlet Challenge Contour Plot")
xlabel('i_hat [m]')
ylabel('j_hat [m]')
zlabel('f(x,y)')
view([-60 64])