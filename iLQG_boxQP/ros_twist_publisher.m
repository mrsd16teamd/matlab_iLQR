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

%% Execute iLQG generated trajectory
%  Frequency same as traj data

load('traj.mat')

for i=1:size(u,2)
    finalTime = datenum(clock + [0, 0, 0, 0, 0, dt]);
    twist_msg.Angular.Z = u(2,i);   % Steering
    vesc_msg.Data = -1664*u(1,i);   % Throttle
    while datenum(clock) < finalTime
        send(twist_chatpub,twist_msg);
        send(vesc_chatpub,vesc_msg);
    end
end

% Publish Twist messages

% for i=1:size(u,2)
%     finalTime = datenum(clock + [0, 0, 0, 0, 0, dt]);
%     msg.Linear.X = u(1,i);
%     msg.Angular.Z = u(2,i);
%     while datenum(clock) < finalTime
%         send(twist_chatpub,msg);
%     end
% end



%% Shut down global node

rosshutdown