load myGauntletmap.mat
%Function for completing level 1 of the gauntlet challenge
clf

%define map
points=[GFramePoints(1,:)',GFramePoints(2,:)'];
x = points(:,1);
y = points(:,2);
figure(2)
plot(x,y,'ks')
title('Scan Data- Clean')
xlabel('[m]')
ylabel('[m]')


%define Neato parameters
base = 0.235; %m
lambda = 0.05;

%Generate map potentials from given coordinates

%initialize potential field and gradient
f = 0;
fx = 0;
fy = 0;

%initialize syms for the Neato to use based on the points fitted
%establish potential field equations
syms xN yN a b
poteq = log(sqrt((xN-a).^2 +(yN-b).^2));
fNeato = 0;
%global points
%[xG,yG]=meshgrid(-1.5:0.01:2.5,-3.37:0.01:1);
[xG,yG]=meshgrid(-10:0.01:10,-10:0.01:10);

%Outline
for aO = -1.2:0.05:2.2
    f = f + log(sqrt((xG-aO).^2 + (yG-0.7).^2)) + log(sqrt((xG-aO).^2 + (yG+3.07).^2));
    fx = fx + (xG-aO)./((xG-aO).^2 +(yG-0.7).^2) + (xG-aO)./((xG-aO).^2 +(yG+3.07).^2);
    fy = fy + (yG-0.7)./((xG-aO).^2 +(yG-0.7).^2) + (yG+3.37)./((xG-aO).^2 +(yG+3.07).^2);
    fNeato = fNeato + subs(poteq,[a,b], [aO,0.7]) + subs(poteq,[a,b], [aO,-3.07]);
    figure(2)
    hold on
    plot(aO,-3.07, 'r.')
    plot(aO,0.7,'r.')
end
for bO = -3.07:0.05:0.7
    f = f + log(sqrt((xG+1.2).^2 + (yG-bO).^2)) + log(sqrt((xG-2.2).^2 + (yG-bO).^2));
    fx = fx + (xG+1.2)./((xG+1.2).^2 +(yG-bO).^2) + (xG-2.2)./((xG-2.2).^2 +(yG-bO).^2);
    fy = fy + (yG-bO)./((xG+1.2).^2 +(yG-bO).^2) + (yG-bO)./((xG-2.2).^2 +(yG-bO).^2);
    fNeato = fNeato + subs(poteq,[a,b], [-1.5,bO]) + subs(poteq,[a,b], [2.5,bO]);
    figure(2)
    hold on
    plot(-1.2,bO,'r.')
    plot(2.2,bO,'r.')
end
%Squares as circles
square_centers = [-0.25, -1; 1, -0.7; 1.41, -2]; 
for i = 1:length(square_centers)
    for t = linspace(0,2*pi,60)
        aC = square_centers(i,1) + 0.25*cos(t);
        bC = square_centers(i,2) + 0.25*sin(t);
        f = f + log(sqrt((xG-aC).^2 +(yG-bC).^2));
        fx = fx + (xG-aC)./((xG-aC).^2 +(yG-bC).^2);
        fy = fy + (yG-bC)./((xG-aC).^2 +(yG-bC).^2);
        fNeato = fNeato + subs(poteq,[a,b], [aC,bC]);
        figure(2)
        hold on
        plot(aC,bC,'r.')
    end
end
%BoB
for t = linspace(0,2*pi,1075)
    aC = 0.75 + 0.25*cos(t);
    bC = -2.5 + 0.25*sin(t);
    f = f - log(sqrt((xG-aC).^2 +(yG-bC).^2));
    fx = fx - (xG-aC)./((xG-aC).^2 +(yG-bC).^2);
    fy = fy - (yG-bC)./((xG-aC).^2 +(yG-bC).^2);
    fNeato = fNeato - subs(poteq,[a,b], [aC,bC]);
    figure(2)
    hold on
    plot(aC,bC,'b.')
end
gradNeato = gradient(fNeato, [xN, yN]);

contour(xG,yG,f, 'k', 'ShowText', 'On')
quiver(xG,yG,fx,fy)
axis equal

%Run gradient ascent
r_i = [0; 0];
delta = 0.9;% delta > 1 causes stepsize to grow; this prevents convergence
lam = 0.05;
tol = 1e-3;
n = 0;
n_max = 25;
grad_i = double(subs(gradNeato, {xN, yN}, {r_i(1), r_i(2)}));
R = [r_i];

% Run the algorithm
%R = gradient_ascend(gradNeato, r_0, delta, lambda_0, tolerance, n_max);
while (n < n_max) && (norm(grad_i)> tol)
    r_i = r_i + lam.*grad_i;
    grad_i = (double(subs(gradNeato, {xN, yN}, {r_i(1), r_i(2)})));
    lam = delta * lam;
    n = n + 1;
    R(:, end+1) = r_i;
end
figure(2);
plot(R(1, :), R(2, :), "r")
plot(R(1, :), R(2, :), "r.", "MarkerSize", 10)
plot(R(1, 1), R(2, 1), "k.", "MarkerSize", 20)
plot(R(1, end), R(2, end), "r.", "MarkerSize", 20)
