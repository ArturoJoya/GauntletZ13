% Code
%initialize syms for the Neato to use based on the points fitted
%establish potential field equations
syms xN yN a b
poteq = log(sqrt((xN-a).^2 +(yN-b).^2));
fNeato = 0;
%global points
%[xG,yG]=meshgrid(-1.5:0.01:2.5,-3.37:0.01:1);
%[xG,yG]=meshgrid(-10:0.01:10,-10:0.01:10);

%Outline
for aO = -1.2:0.05:2.2
    fNeato = fNeato + subs(poteq,[a,b], [aO,0.7]) + subs(poteq,[a,b], [aO,-3.07]);
end
for bO = -3.07:0.05:0.7
    fNeato = fNeato + subs(poteq,[a,b], [-1.5,bO]) + subs(poteq,[a,b], [2.5,bO]);
end
% %Squares as circles
square_centers = [-0.25, -1; 1, -0.7; 1.41, -2]; 
for i = 1:length(square_centers)
    for t = 0:0.4:2*pi
        aC = square_centers(i,1) + 0.353*cos(t);
        bC = square_centers(i,2) + 0.353*sin(t);
        fNeato = fNeato - arbitrary_constant.*subs(poteq,[a,b], [aC,bC]);
    end
end
%BoB
for t = 0:0.4:2*pi
    aC = 0.75 + 0.25*cos(t);
    bC = -2.5 + 0.25*sin(t);
    fNeato = fNeato + subs(poteq,[a,b], [aC,bC]);
end
gradNeato = gradient(fNeato, [xN, yN]);

r_i = [-0.5; 0];
delta = 0.9; % delta > 1 causes stepsize to grow; this prevents convergence
lam = 0.05;
tol = 1e-3;
n = 0;
n_max = 7;
grad_i = double(subs(gradNeato, {xN, yN}, {r_i(1), r_i(2)}));
R = [r_i];

% Run the algorithm
%R = gradient_ascend(gradNeato, r_0, delta, lambda_0, tolerance, n_max);
while (n < n_max) && (norm(grad_i)> tol)
    r_i = r_i - lam.*grad_i;
    grad_i = (double(subs(gradNeato, {xN, yN}, {r_i(1), r_i(2)})));
    lam = delta * lam;
    n = n + 1;
    R(:, end+1) = r_i;
end
clf
contour(xG,yG,f, 'k', 'ShowText', 'On')
hold on
contour3(xG, yG, f, 100)
view([0 90])
plot(R(1, :), R(2, :), "r")
plot(R(1, :), R(2, :), "r.", "MarkerSize", 10)
plot(R(1, 1), R(2, 1), "k.", "MarkerSize", 20)
plot(R(1, end), R(2, end), "r.", "MarkerSize", 20)
title('Gauntlet Final Calculated Path')
xlabel('[m]')
ylabel('[m]')