function [Fx,Fy] = tire_dyn(K, mu, Fz, C_x, C_alpha, alpha)
%#codegen
    if K == -1
        K = -0.99;
    elseif abs(K) == Inf
        Fx = sign(K)*mu*Fz;
        Fy = 0;
        return;
    end
 
    gamma = sqrt(C_x^2*(K/(1+K))^2+C_alpha^2*(tan(alpha)/(1+K))^2);
    
    if gamma <= 3*mu*Fz
        F = gamma - 1/(3*mu*Fz)*gamma^2 + 1/(27*mu^2*Fz^2)*gamma^3;
    else
        F = mu*Fz;
    end
    
    if gamma == 0
        Fx = 0;
        Fy = 0;
    else
        Fx = C_x/gamma * (K/(1+K)) * F;
        Fy = -C_alpha/gamma * (tan(alpha)/(1+K)) * F;
    end
end