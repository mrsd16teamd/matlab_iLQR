function ddl = calc_ddl_dxdx( x, u )
%CALC_DDL_DXDX 
global DEFAULTSTEPSIZE;
global X_DIM;
ddl = zeros(X_DIM, X_DIM);

p = calc_l(x, u);

xr = x;
xl = x;
for ii = 1 : X_DIM
    xr(ii) = xr(ii) + DEFAULTSTEPSIZE;
    xl(ii) = xl(ii) - DEFAULTSTEPSIZE;
    ddl(ii, ii) = (calc_l(xl, u) - 2.0 * p + calc_l(xr, u)) / (DEFAULTSTEPSIZE * DEFAULTSTEPSIZE);
    xr(ii) = x(ii);
    xl(ii) = x(ii);
end

xtr = x;
xtl = x;
xbr = x;
xbl = x;
for ii = 1 : X_DIM
    xtr(ii) = xtr(ii) + DEFAULTSTEPSIZE;
    xtl(ii) = xtl(ii) - DEFAULTSTEPSIZE;
    xbr(ii) = xbr(ii) + DEFAULTSTEPSIZE;
    xbl(ii) = xbl(ii) - DEFAULTSTEPSIZE;
    for jj = 1 : ii
        xtr(jj) = xtr(jj) + DEFAULTSTEPSIZE;
        xtl(jj) = xtl(jj) + DEFAULTSTEPSIZE;
        xbr(jj) = xbr(jj) - DEFAULTSTEPSIZE;
        xbl(jj) = xbl(jj) - DEFAULTSTEPSIZE;
        ddl(ii, jj) = (calc_l(xbl, u) + calc_l(xtr, u) - calc_l(xtl, u) - calc_l(xbr, u)) / ...
            (4.0 * DEFAULTSTEPSIZE * DEFAULTSTEPSIZE);
        ddl(jj, ii) = ddl(ii, jj);
        xtr(jj) = x(jj);
        xtl(jj) = x(jj);
        xbr(jj) = x(jj);
        xbl(jj) = x(jj);
    end
    xtr(ii) = x(ii);
    xtl(ii) = x(ii);
    xbr(ii) = x(ii);
    xbl(ii) = x(ii);
end

end

