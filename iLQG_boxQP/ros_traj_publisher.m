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

% Interpolat data for double frequency
th=interp1(dt:dt:size(u,2)*dt,u(1,:),dt/2:dt/2:size(u,2)*dt,'spline');
st=interp1(dt:dt:size(u,2)*dt,u(2,:),dt/2:dt/2:size(u,2)*dt,'spline');
u=[th;st];

%% Publish vesc messages

for i=1:size(u,2)
    finalTime = datenum(clock + [0, 0, 0, 0, 0, dt]);
    twist_msg.Angular.Z = u(2,i);   % Steering
    vesc_msg.Data = -1664*u(1,i);   % Throttle
    while datenum(clock) < finalTime
        send(twist_chatpub,twist_msg);
        send(vesc_chatpub,vesc_msg);
    end
end


% % Debug script
% while(1)
%     vesc_msg.Data = -1664*1;   % Throttle
%     twist_msg.Angular.Z = 0.4;   % Steering
%     send(twist_chatpub,twist_msg);
%     send(vesc_chatpub,vesc_msg);
%     pause(0.05);
% end

%% Publish Twist messages

% pause(5)

for i=1:size(u,2)
%     finalTime = datenum(clock + [0, 0, 0, 0, 0, dt]);
    twist_msg.Linear.X = u(1,i);
    twist_msg.Angular.Z = 2.75*u(2,i);
%     while datenum(clock) < finalTime
        send(twist_chatpub,twist_msg);
%     end
    pause(dt)
end


%% Shut down global node

rosshutdown