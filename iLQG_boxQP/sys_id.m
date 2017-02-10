function [params] = sys_id(file_path)

global msgs_state msgs_cmd;
global params_to_optimize;

% Log file with control inputs and states with timestamps 
file_name = 'sudden_obstacle.bag';
bagfile = rosbag(file_name);

bag_state = select(bagfile, 'Topic', '/scanmatch_odom');
bag_cmd = select(bagfile, 'Topic', '/cmd_vel');

msgs_state = readMessages(bag_state);
msgs_cmd = readMessages(bag_cmd); % Need this to be geometry_msgs/TwistStamped


% x = [x, y, phi, Ux, Uy, r]
% u = [Ux_cmd, delta]

% Choose parameters of interest

m = 2.35;           % mass (kg)
L = 0.257;          % wheelbase (m)
g = 9.81;   
b = 0.14328;        % CoG to rear axle
a = L-b;            % CoG to front axle
G_rear = m*g*a/L;
mu = 5.2/G_rear;   
mu_spin = 4.3/G_rear; 

% params = [C_alpha, C_x, Iz, mu, mu_spin]
params_to_optimize = [1, 1, 1, 1, 1];
p0 = [300, 330, 0.02065948883, mu, mu_spin];     

sys_id_criterion(p0);

% Send data to optimizer
ALG = 1;

switch ALG
    case 1 % CMA-ES, capped at 500 iterations
        sigma0 = ones(n_joints*3,1)*0.2; % TODO edit this to adjust to param size
        opts = cmaes;
        opts.MaxIter = 500;
        [xmin, fmin, counteval, stopflag, out, bestever] = cmaes('sys_id_criterion', p0, sigma0, opts);
        answer = xmin; fval = fmin; exitflag=1;
    case 2  % SQP
        options = optimset('Display','iter','MaxFunEvals',1000000,'Algorithm','sqp');
        [answer,fval,exitflag]=fmincon(@sys_id_criterion,p0, [], [], [], [], [], [], [], options);
end

end