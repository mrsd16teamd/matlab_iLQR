function [costs] = evaluate_cost_traj(x,u)
T = size(x,2);
costs = zeros(T,6);

u = [u, [NaN;NaN]];

% costs = [lu, lf, lx, ldu, ld, lobs];
% u:final, f:final, x:state, du:change in control, ld:drift, lobs:obstacle

c_total = 0;
for i = 1:T
    [c,c_all] = car_cost(x(:,i),u(:,i));
    c_total = c_total+c;
    costs(i,:) = c_all;
end

disp(c_total);

end