function [Fx,Fy] = tire_dyn(K, mu, Fz, C_x, C_alpha, alpha)
%#codegen

% TODO: Make this continuous somehow - add transition region with sigmoid?
    % instead of avoiding K=-1, now look for positive equivalent
    reverse = 1;
    if K < 0
        reverse = -1;
        K = abs(K);
    elseif abs(K) == Inf
        % Fx = sign(K)*mu*0.9*Fz;
        Fx = sign(K)*mu*Fz;
        Fy = 0;
        return;
    end
    
    % alpha > pi/2 cannot be adapted to this formula
    % because of the use of tan(). Use the equivalent angle instead.
    if abs(alpha) > pi/2
        alpha = (abs(alpha)-pi/2)*sign(alpha);
    end
 
    gamma = sqrt(C_x^2*(K/(1+K))^2+C_alpha^2*(tan(alpha)/(1+K))^2);
    
    if gamma <= 3*mu*Fz
        F = gamma - 1/(3*mu*Fz)*gamma^2 + 1/(27*mu^2*Fz^2)*gamma^3;
    else
        % more accurate modeling with peak friction value
        % F = (mu*0.9 + (0.1*mu)/(1 + ((gamma-3*mu*Fz)/9)^2))*Fz;
        F = mu*Fz;
    end
    
    if gamma == 0
        Fx = 0;
        Fy = 0;
    else
        Fx = C_x/gamma * (K/(1+K)) * F * reverse;
        Fy = -C_alpha/gamma * (tan(alpha)/(1+K)) * F;
    end
end