function [Xn, Un] = ForwardShootOpt(Xc, Uc, k, K, x_start, dt)
global NUM_CTRL;
global X_DIM U_DIM;

Xn = zeros(X_DIM, NUM_CTRL+1);
Un = ones(U_DIM, NUM_CTRL);

Xn(:, 1) = x_start;
for t = 1 : NUM_CTRL
    du = k(:, t) + K(:, :, t) * (Xn(:, t) - Xc(:, t));
    Un(:, t) = Uc(:, t) + du;
    Xn(:, t+1) = dynamics_finite(Xn(:, t)', Un(:, t), dt);
end
