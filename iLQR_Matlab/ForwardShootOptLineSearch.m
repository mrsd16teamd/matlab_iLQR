function [Xn, Un] = ForwardShootOptLineSearch(sdv, Xc, Uc, k, K, x_start, x_goal, obstacles, l_u, dt)
global NUM_CTRL;
global X_DIM U_DIM;

Xn = zeros(X_DIM, NUM_CTRL+1);
Un = ones(U_DIM, NUM_CTRL);
P = zeros(NUM_CTRL, 1);

is_done = 0;
alpha = 1;
tau = 1/2;
c = 1/2;

cnt = 0;
while (~is_done)
    cnt = cnt + 1;
    
    Xn(:, 1) = x_start;
    for t = 1 : NUM_CTRL
        du = k(:, t) + K(:, :, t) * (Xn(:, t) - Xc(:, t));
        P(t) = norm(du);
        Un(:, t) = Uc(:, t) + alpha * du;
        Xn(:, t+1) = sdv.ss_fd(Xn(:, t), Un(:, t), dt);
    end
    
    % Has some problems!
    m = P' * l_u;
    
    Jc = calc_J(sdv, Xc, Uc, x_start, x_goal, obstacles);
    Jn = calc_J(sdv, Xn, Un, x_start, x_goal, obstacles);
    
    if Jn-Jc <= alpha * c * m
        is_done = 1;
    end
    
    alpha = alpha * tau;
end

end