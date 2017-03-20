function [Fx,Fy] = tire_dyn(Ux, Ux_cmd, mu, mu_slide, Fz, C_x, C_alpha, alpha)

    % longitude wheel slip
    if (Ux_cmd == Ux)
        K = 0;
    elseif Ux == 0
        Fx = sign(Ux_cmd)*mu*Fz;
        Fy = 0;
        return;
    else
        K = (Ux_cmd-Ux)/abs(Ux);
    end
    
    % instead of avoiding -1, now look for positive equivalent
    reverse = 1;
    if K < 0
        reverse = -1;
        K = abs(K);
    end
    
    % alpha > pi/2 cannot be adapted to this formula
    % because of the use of tan(). Use the equivalent angle instead.
    if abs(alpha) > pi/2
        alpha = (pi-abs(alpha))*sign(alpha);
    end
 
    gamma = sqrt(C_x^2*(K/(1+K))^2+C_alpha^2*(tan(alpha)/(1+K))^2);
    
    if gamma <= 3*mu*Fz
        F = gamma - 1/(3*mu*Fz)*(2-mu_slide/mu)*gamma^2 + 1/(9*mu^2*Fz^2)*(1-(2/3)*(mu_slide/mu))*gamma^3;
    else
        % more accurate modeling with peak friction value
        F = mu_slide*Fz;
    end
    
    if gamma == 0
        Fx = 0;
        Fy = 0;
    else
        Fx = C_x/gamma * (K/(1+K)) * F * reverse;
        Fy = -C_alpha/gamma * (tan(alpha)/(1+K)) * F;
    end
end