% % Generates Final Moose Test Trajectory 
% % by prepending speed ramp before execution

% load('../saved_trajectories/moose_test_lowIz.mat')

dt = 0.02;
init_vel = 3;
init_steer = 0;

ramp_th = linspace(0,init_vel,1/dt);
ramp_st = zeros(1,1/dt);

ramp_th = [ramp_th, init_vel*ones(1,0.5/dt)];
ramp_st = [ramp_st, zeros(1,0.5/dt)];

u = [[ramp_th;ramp_st], U];

save('moose_test_traj.mat','u','dt','X','U')