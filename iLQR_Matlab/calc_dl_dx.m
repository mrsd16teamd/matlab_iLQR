function dl = calc_dl_dx( x, u )
%CALC_DL_DX 
global DEFAULTSTEPSIZE;
global X_DIM;

dl = zeros(X_DIM, 1);
xr = x;
xl = x;
for ii = 1 : X_DIM
    xr(ii) = xr(ii) + DEFAULTSTEPSIZE;
    xl(ii) = xl(ii) - DEFAULTSTEPSIZE;
    dl(ii) = (calc_l(xr, u) - calc_l(xl, u)) / (2 * DEFAULTSTEPSIZE);
    xr(ii) = x(ii);
    xl(ii) = x(ii);
end

end

