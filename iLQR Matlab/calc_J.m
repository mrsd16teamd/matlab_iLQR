function J = calc_J( X, U )
%CALC_J 
global NUM_CTRL;
global U_DIM;

J = 0;
for t = 1 : NUM_CTRL
    J = J + calc_l(X(:, t), U(:, t));
end
J = J + calc_l(X(:, NUM_CTRL+1), zeros(U_DIM, 1));

