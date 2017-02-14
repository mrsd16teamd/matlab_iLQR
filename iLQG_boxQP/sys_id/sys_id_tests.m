%% System Identification experiment scripts

% Run individual sections as required

%% Initialize Global ROS node

rosinit('http://192.168.1.118:11311')

%% Initialize Publishers

twist_chatpub = rospublisher('/cmd_vel','geometry_msgs/Twist');
twist_msg = rosmessage(twist_chatpub);
vesc_chatpub = rospublisher('/commands/motor/speed','std_msgs/Float64');
vesc_msg = rosmessage(vesc_chatpub);
vel = 1;
disp('Created following pubishers:')
disp(twist_chatpub)
disp(vesc_chatpub)

%%  Tire Stiffness Experiment (Steering Ramp)
 
dt = 0.05; % Publish period
time_horizon = 5;
steer_angles = linspace(0,0.96,time_horizon/dt);
lin_vel = 0.5;
vesc_msg.Data = -1664*lin_vel;  % Vx
 
for i=1:length(steer_angles)
    twist_msg.Angular.Z = steer_angles(i);
    send(twist_chatpub,twist_msg);
    send(vesc_chatpub,vesc_msg);
    pause(dt)
end

ctrl_ip = [repmat(lin_vel,1,length(steer_angles));steer_angles];

file_name = ['steer_ramp',datestr(now,'_mm-dd-yy_HH:MM')];
save([file_name,'.mat'],'dt','time_horizon','ctrl_ip');

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
 
max_steer = 0.96;
max_vel = 4;
time_horizon = 5;
finalTime = datenum(clock + [0, 0, 0, 0, 0, time_horizon]);
while datenum(clock) < finalTime
    twist_msg.Angular.Z = max_steer;
    vesc_msg.Data = -1664*max_vel;
    send(twist_chatpub,twist_msg);
    send(vesc_chatpub,vesc_msg);
end
% % Uncomment loop below for figure of 8 drifting
% finalTime = datenum(clock + [0, 0, 0, 0, 0, time_horizon]);
% while datenum(clock) < finalTime
%     twist_msg.Angular.Z = -max_steer;
%     vesc_msg.Data = -1664*max_vel;
%     send(twist_chatpub,twist_msg);
%     send(vesc_chatpub,vesc_msg);
% end

