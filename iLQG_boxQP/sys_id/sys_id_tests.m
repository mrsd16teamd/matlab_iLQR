%% System Identification experiment scripts

% Run individual sections as required

%% Initialize Global ROS node

rosinit('http://192.168.1.118:11311')

% Initialize Publishers

twist_chatpub = rospublisher('/cmd_vel','geometry_msgs/Twist');
twist_msg = rosmessage(twist_chatpub);
% vesc_chatpub = rospublisher('/commands/motor/speed','std_msgs/Float64');
% vesc_msg = rosmessage(vesc_chatpub);
disp('Created following pubishers:')
disp(twist_chatpub)
% disp(vesc_chatpub)

%%  Tire Stiffness Experiment (Steering Ramp)
 
dt = 0.01; % Publish period
time_horizon = 2;
steer_angles = -linspace(0,0.76,time_horizon/dt);   % -0.68,+0.76. R, L
lin_vel = 1;

for i=1:length(steer_angles)
    twist_msg.Angular.Z = steer_angles(i);
    twist_msg.Linear.X = lin_vel;
    send(twist_chatpub,twist_msg);
    pause(dt)
end

% Send zero to ensure robot stops at end
twist_msg.Linear.X = 0;
twist_msg.Angular.Z = 0;
send(twist_chatpub,twist_msg);

ctrl_ip = [repmat(lin_vel,1,length(steer_angles));steer_angles];

file_name = ['steer_ramp',datestr(now,'_mm-dd-yy_HH:MM')];
save([file_name,'.mat'],'dt','time_horizon','ctrl_ip');
disp('done');

%% Launch Acceleration Experiment

time_horizon = 5;
lin_vel = [0,0.5];
twist_msg.Angular.Z = 0;
    
for i=1:2
    finalTime = datenum(clock + [0, 0, 0, 0, 0, time_horizon]);
    vesc_msg.Data = lin_vel(i);  % Vx
    while datenum(clock) < finalTime
        send(vesc_chatpub,vesc_msg);
        send(twist_chatpub,twist_msg);
    end
end

% file_name = ['launch',datestr(now,'_mm-dd-yy_HH:MM')];
% save([file_name,'.mat'],'time_horizon','lin_vel');

%% Donuts/Figure of 8 drifting
 
max_steer = 0.77;
fraction = 2/3;
max_vel = 0.25;
time_horizon = 15;

twist_msg.Linear.X = 0;
twist_msg.Angular.Z = fraction*max_steer;
send(twist_chatpub,twist_msg);
pause(2)

finalTime = datenum(clock + [0, 0, 0, 0, 0, time_horizon]);
while datenum(clock) < finalTime
    twist_msg.Angular.Z = fraction*max_steer;
    twist_msg.Linear.X = max_vel;
    send(twist_chatpub,twist_msg);
end
% % Uncomment loop below for figure of 8 drifting
% finalTime = datenum(clock + [0, 0, 0, 0, 0, time_horizon]);
% while datenum(clock) < finalTime
%     twist_msg.Angular.Z = -max_steer;
%     vesc_msg.Data = -1664*max_vel;
%     send(twist_chatpub,twist_msg);
%     send(vesc_chatpub,vesc_msg);
% end

twist_msg.Linear.X = 0;
twist_msg.Angular.Z = 0;
send(twist_chatpub,twist_msg);