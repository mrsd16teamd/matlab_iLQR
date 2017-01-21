function ddl = calc_ddl_dudu( x, u )
%CALC_DDL_DUDU 
global DEFAULTSTEPSIZE;
global U_DIM;
ddl = zeros(U_DIM, U_DIM);

p = calc_l(x, u);

ur = u; ul = u;
for ii = 1 : U_DIM
    ur(ii) = ur(ii) + DEFAULTSTEPSIZE;
    ul(ii) = ul(ii) - DEFAULTSTEPSIZE;
    ddl(ii, ii) = (calc_l(x, ul) - 2.0 * p + calc_l(x, ur)) / ...
        (DEFAULTSTEPSIZE * DEFAULTSTEPSIZE);
    ur(ii) = u(ii);
    ul(ii) = u(ii);
end

utr = u; utl = u; ubr = u; ubl = u;
for ii = 1 : U_DIM
    utr(ii) = utr(ii) + DEFAULTSTEPSIZE;
    utl(ii) = utl(ii) - DEFAULTSTEPSIZE;
    ubr(ii) = ubr(ii) + DEFAULTSTEPSIZE;
    ubl(ii) = ubl(ii) - DEFAULTSTEPSIZE;
    for jj = 1 : ii
        utr(jj) = utr(jj) + DEFAULTSTEPSIZE;
        utl(jj) = utl(jj) + DEFAULTSTEPSIZE;
        ubr(jj) = ubr(jj) - DEFAULTSTEPSIZE;
        ubl(jj) = ubl(jj) - DEFAULTSTEPSIZE;
        ddl(ii, jj) = (calc_l(x, ubl) + calc_l(x, utr) - calc_l(x, utl) - calc_l(x, ubr)) / ...
            (4.0 * DEFAULTSTEPSIZE * DEFAULTSTEPSIZE);
        ddl(jj, ii) = ddl(ii, jj);
        utr(jj) = u(jj); 
        utl(jj) = u(jj); 
        ubr(jj) = u(jj); 
        ubl(jj) = u(jj);
    end
    utr(ii) = u(ii);
    utl(ii) = u(ii);
    ubr(ii) = u(ii);
    ubl(ii) = u(ii);
end

end

