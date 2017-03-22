% % Generates Final Test Trajectory 
% % by prepending speed ramp before execution

% load('../saved_trajectories/moose_test_lowIz.mat')

dt = 0.02;
init_vel = 3;
init_steer = 0;

ramp_time = 1;
steady_time = 0.5;
accel = init_vel/ramp_time;

ramp_th = linspace(0,init_vel,ramp_time/dt);
ramp_th = [ramp_th, init_vel*ones(1,steady_time/dt)];
ramp_st = zeros(1,length(ramp_th));

ramp_x = linspace(0,init_vel*ramp_time/2,1/dt);
ramp_x = [ramp_x, ramp_x(end)+linspace(accel*dt,accel*steady_time,steady_time/dt)];
ramp_y = zeros(1,length(ramp_x));

u = [[ramp_th;ramp_st], U];
x = [[ramp_x;ramp_y], [ramp_x(end)+X(1,:);ramp_y(end)+X(2,:)]];

save('test.mat','u','x','dt','X','U')