function X = ForwardShoot(U, x_start, dt)
global NUM_CTRL;
global X_DIM;

X = zeros(X_DIM, NUM_CTRL+1);

X(:, 1) = x_start;
for ii = 1 : NUM_CTRL
    X(:, ii+1) = dynamics(X(:, ii), U(:, ii), dt);
end