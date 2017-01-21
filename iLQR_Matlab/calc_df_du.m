function B = calc_df_du( sdv, x, u )
%CALC_DF_DU: calculate df/du
    global DEFAULTSTEPSIZE;
    global X_DIM U_DIM;
    B = zeros(X_DIM, U_DIM);
    
    ur = u;
    ul = u;
    
    for i = 1 : U_DIM
        ur(i) = ur(i) + DEFAULTSTEPSIZE;
        ul(i) = ul(i) - DEFAULTSTEPSIZE;
        B(:, i) = (calc_f(sdv, x, ur) - calc_f(sdv, x, ul)) / (2 * DEFAULTSTEPSIZE);
        
        ur(i) = u(i);
        ul(i) = u(i);
    end

end

