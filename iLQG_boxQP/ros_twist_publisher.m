load('traj.mat')

rosinit('http://192.168.1.118:11311')
twist_chatpub = rospublisher('/cmd_vel','geometry_msgs/Twist');
msg = rosmessage(twist_chatpub);
pause(2) % Wai to ensure publisher is registered
%%

wheelbase = 0.257;
vx = u(1,:);
steer_angle = acos(vx./sqrt(u(1,:).^2 + u(2,:).^2)); % TODO:Kinematic eqns!


%%

for i=1:size(u,2) 
    msg.Linear.X = vx(i);
    msg.Angular.Z = steer_angle(i);
    send(twist_chatpub,msg);
    pause(dt)
end

%%
rosshutdown