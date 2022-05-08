function R = gradient_ascend(grad, r_0, lam_0, delta, tol, n_max)
syms xN yN
grad = grad';
r_i = r_0
lam = lam_0;
grad_i = r_i;
n = 0;
R = [r_i]

while (n < n_max) && (norm((double(subs(grad, {xN, yN}, {r_i(1), r_i(2)}))))> tol)
    um = double(subs(grad, {xN, yN}, {r_i(1), r_i(2)}))
    r_i = r_i + lam*(double(subs(grad, {xN, yN}, {r_i(1), r_i(2)})))
    grad_i = (double(subs(grad, {xN, yN}, {r_i(1), r_i(2)})))
    lam = delta * lam;
    n = n + 1
    R(end+1, :) = r_i
end