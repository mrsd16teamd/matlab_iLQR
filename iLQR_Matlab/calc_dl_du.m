function dl = calc_dl_du( x, u) %obstacles, t )
%CALC_DL_DU

global DEFAULTSTEPSIZE;
global U_DIM;

dl = zeros(U_DIM, 1);
ur = u;
ul = u;
for ii = 1 : U_DIM
    ur(ii) = ur(ii) + DEFAULTSTEPSIZE;
    ul(ii) = ul(ii) - DEFAULTSTEPSIZE;
    dl(ii) = (calc_l(x, ur) - ... % obstacles, t)
        calc_l(x, ul)) /...
        (2 * DEFAULTSTEPSIZE);
    ur(ii) = u(ii);
    ul(ii) = u(ii);
end
end


