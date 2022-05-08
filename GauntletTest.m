function NeatoPath = GauntletTest()

%define Neato parameters
base = 0.235; %m
lambda = 0.05;

%define Circle fitting parameters
d = 0.005; %m
n = 1000; %iterations for circle fit
R = 0.25; %m
fNeato = 0;

%Visualize field and create syms equations
VisualizeField
%generate gradient for Neato to use
gradNeato = gradient(fNeato, [xN, yN]);

%Run gradient ascent
r_0 = [0, 0];
delta = 0.9; % delta > 1 causes stepsize to grow; this prevents convergence
lambda_0 = 0.05;
tolerance = 1e-3;
n_max = 25;

% Run the algorithm
R = gradient_ascend(gradNeato, r_0, delta, lambda_0, tolerance, n_max);

figure(1);
plot(R(:, 1), R(:, 2), "r")
plot(R(:, 1), R(:, 2), "r.", "MarkerSize", 10)
plot(R(1, 1), R(1, 2), "k.", "MarkerSize", 20)
plot(R(end, 1), R(end, 2), "r.", "MarkerSize", 20)

end