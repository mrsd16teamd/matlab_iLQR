function ddl = calc_ddl_dudx( x, u )
%CALC_DDL_DUDX 
global DEFAULTSTEPSIZE;
global X_DIM U_DIM;

ddl = zeros(X_DIM, U_DIM);
xtr = x; xtl = x; xbr = x; xbl = x;
utr = u; utl = u; ubr = u; ubl = u;

for ii = 1 : X_DIM
    xtr(ii) = xtr(ii) + DEFAULTSTEPSIZE;
    xtl(ii) = xtl(ii) - DEFAULTSTEPSIZE;
    xbr(ii) = xbr(ii) + DEFAULTSTEPSIZE;
    xbl(ii) = xbl(ii) - DEFAULTSTEPSIZE;
    for jj = 1 : U_DIM
        utr(jj) = utr(jj) + DEFAULTSTEPSIZE;
        utl(jj) = utl(jj) + DEFAULTSTEPSIZE;
        ubr(jj) = ubr(jj) - DEFAULTSTEPSIZE;
        ubl(jj) = ubl(jj) - DEFAULTSTEPSIZE;
        ddl(ii, jj) = (calc_l(xbl, ubl) + calc_l(xtr, utr) - calc_l(xtl, utl) - calc_l(xbr, ubr)) / ...
            (4.0 * DEFAULTSTEPSIZE * DEFAULTSTEPSIZE);
        utr(jj) = u(jj);
        utl(jj) = u(jj);
        ubr(jj) = u(jj);
        ubl(jj) = u(jj);
    end
    xtr(ii) = x(ii);
    xtl(ii) = x(ii);
    xbr(ii) = x(ii);
    xbl(ii) = x(ii);
end
ddl = ddl';
end

