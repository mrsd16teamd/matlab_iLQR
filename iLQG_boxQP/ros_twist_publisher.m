%% Load trajectory & Initialize Global ROS node

load('traj.mat')

rosinit('http://192.168.1.118:11311')
twist_chatpub = rospublisher('/cmd_vel','geometry_msgs/Twist');
msg = rosmessage(twist_chatpub);
pause(2) % Wait to ensure publisher is registered

%% TODO:Kinematic eqns?
 
% wheelbase = 0.257;
% vx = u(1,:);
% steer_angle = acos(vx./sqrt(u(1,:).^2 + u(2,:).^2)); 

%% Publish Twist messages
% Frequency same as traj data

for i=1:size(u,2)
    finalTime = datenum(clock + [0, 0, 0, 0, 0, dt]);
    msg.Linear.X = u(1,i);
    msg.Angular.Z = u(2,i);
    while datenum(clock) < finalTime
        send(twist_chatpub,msg);
    end
end

%% Shut down global node

rosshutdown