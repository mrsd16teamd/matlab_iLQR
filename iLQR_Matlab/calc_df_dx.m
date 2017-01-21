function A = calc_df_dx( x, u )
%CALC_DF_DX: calculate df/dx
    global DEFAULTSTEPSIZE;
    global X_DIM;
    A = zeros(X_DIM, X_DIM);

    xr = x;
    xl = x;
    for i = 1 : X_DIM
        xr(i) = xr(i) + DEFAULTSTEPSIZE;
        xl(i) = xl(i) - DEFAULTSTEPSIZE;
        A(:,i) = (calc_f(xr, u) - calc_f(xl, u)) / (2 * DEFAULTSTEPSIZE);
        xr(i) = x(i);
        xl(i) = x(i);
    end
end