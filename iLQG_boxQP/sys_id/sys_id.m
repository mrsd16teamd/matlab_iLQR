function sys_id()

global stateData;
global params_to_optimize default_params;

load('stateData_02-16-17_16-44.mat');
disp('Loaded data.');

%%%%%
% params = [C_alpha, C_x, Iz, mu, mu_spin]
params_to_optimize = [1, 1, 1, 1, 1];
ALG = 1; % 1 = CMA-ES, 2 = fmincon, 3 = fminsearch, 4, = fminunc
max_f_evals = 100;

% Best methods so far: 3
%%%%

% x = [x, y, phi, Ux, Uy, r]
% u = [Ux_cmd, delta]

m = 2.35;           % mass (kg)
L = 0.257;          % wheelbase (m)
g = 9.81;   
b = 0.14328;        % CoG to rear axle
a = L-b;            % CoG to front axle
G_rear = m*g*a/L;
mu = 5.2/G_rear;   % currently 0.5098
mu_spin = 4.3/G_rear; % currently 0.4215

p0 = [300, 330, 0.02065948883, mu, mu_spin];   
default_params = p0;

% params: [C_alpha, C_x, Iz, mu, mu_spin]
lb = [100,  100,  0,   0.25, 0.25];
ub = [1000, 1000, 0.1, 1,    1];

sys_id_criterion(p0);

disp('Starting optimization...');
switch ALG
    case 1 % CMA-ES, capped at 500 iterations
        sigma0 = ones(length(p0),1)*50; 
        opts = cmaes;
        opts.MaxIter = max_f_evals;
        [xmin, fmin, counteval, stopflag, out, bestever] = cmaes('sys_id_criterion', p0, sigma0, opts);
        answer = xmin; fval = fmin; exitflag=1;
    case 2  % fmincon  
        % options are 'interior-point', 'trust-region-reflective', 'sqp',
        % 'active-set'
        options = optimset('Display','iter','MaxFunEvals',max_f_evals,'Algorithm','interior-point');
        [answer,fval,exitflag]=fmincon(@sys_id_criterion,p0, [], [], [], [], lb, ub, [], options);
    case 3
        options = optimset('Display','iter','MaxFunEvals',max_f_evals,'PlotFcns',@optimplotfval);
        [answer,fval,exitflag,output] = fminsearch(@sys_id_criterion, p0, options);
    case 4
        options = optimset('Display','iter','MaxFunEvals',max_f_evals,'PlotFcns',@optimplotfval);
        [answer,fval,exitflag,output] = fminunc(@sys_id_criterion, p0, options);     
end

best_params = answer;
disp(best_params);
disp(fval);

end