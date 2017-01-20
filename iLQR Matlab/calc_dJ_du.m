function dJ_du = calc_dJ_du( sdv, X, U, x_goal, obstacles )
%CALC_J
global U_DIM;
global NUM_CTRL;

dJ_du = zeros(U_DIM, 1);

for t = 1 : NUM_CTRL
    dJ_du = dJ_du + calc_dl_du(sdv, X(:, t), U(:, t), x_goal, obstacles, t);
end
dJ_du = dJ_du + calc_dl_du(sdv, X(:, NUM_CTRL+1), zeros(U_DIM, 1), x_goal, obstacles, NUM_CTRL+1);

