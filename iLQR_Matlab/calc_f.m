function dx = calc_f( x, u )
%CALC_F calculate dx, where X is [x, y, h], U is [steer, speed]
    global X_DIM;
    dx = dynamics(x, u);
    assert(numel(dx) == X_DIM);
end

