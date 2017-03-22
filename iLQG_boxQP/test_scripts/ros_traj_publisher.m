%% ROS Trajectory Publisher

function ros_traj_publisher(traj_name)

  if nargin < 1
    traj_name = fullfile(which('traj_smooth.mat'));
  end
  load(traj_name)

%% Initialize Global ROS node if not already active

tx1 = rosdevice('192.168.1.118','ubuntu','ubuntu');

try
    rosinit('http://192.168.1.118:11311')
catch EXP
    if string(EXP.identifier)=='robotics:ros:node:GlobalNodeNotRunning'
        disp('ROS Master not running!')
    elseif string(EXP.identifier)=='robotics:ros:node:NoMasterConnection'
        disp('Cannot connect to ROS master! Check network')
        return
    end
end

%% Initialize Publishers

twist_chatpub = rospublisher('/cmd_vel','geometry_msgs/Twist');
% twist_chatpub2 = rospublisher('/cmd_vel_stamped','geometry_msgs/TwistStamped');

twist_msg = rosmessage(twist_chatpub);
% twist_msg2 = rosmessage(twist_chatpub2);

traj_chatpub = rospublisher('/traj','nav_msgs/Odometry');
traj_msg = rosmessage(traj_chatpub);

disp('Created following pubishers:');
disp(twist_chatpub);
disp(traj_chatpub);

%% Execute iLQG generated trajectory
%  Frequency same as traj data

dt=0.02;

% % Change publish frequency
% data_freq = 1/dt;
% target_freq = 20;
% scale = target_freq/data_freq;
% dt = dt/scale;
% % Interpolate data for double frequency
% th=interp1(dt:dt:size(u,2)*dt,u(1,:),dt/scale:dt/scale:size(u,2)*dt,'spline');
% st=interp1(dt:dt:size(u,2)*dt,u(2,:),dt/scale:dt/scale:size(u,2)*dt,'spline');
% u=[th;st];

% % Publish Twist messages

for i=1:size(u,2)    
    twist_msg.Linear.X = u(1,i);
    twist_msg.Angular.Z = u(2,i);
        
    traj_msg.Header.FrameId='/map';
    traj_msg.Twist.Twist = twist_msg;
    traj_msg.Pose.Pose.Position.X = x(1,i);
    traj_msg.Pose.Pose.Position.Y = x(2,i);
    
    send(twist_chatpub,twist_msg);
    send(traj_chatpub,traj_msg);
    pause(dt)
end

twist_msg.Linear.X = 0;
twist_msg.Angular.Z = 0;
send(twist_chatpub,twist_msg);

%% Publish vesc messages

% vesc_chatpub = rospublisher('/commands/motor/speed','std_msgs/Float64');
% vesc_msg = rosmessage(vesc_chatpub);
% disp(vesc_chatpub)

% for i=1:size(u,2)
%     finalTime = datenum(clock + [0, 0, 0, 0, 0, dt/scale]);
%     twist_msg.Angular.Z = 1*u(2,i);   % Steering
%     twist_msg.Linear.X = u(1,i);
% %     vesc_msg.Data = -1664*u(1,i);   % Throttle
%     send(twist_chatpub,twist_msg);
% %     send(vesc_chatpub,vesc_msg);
%     while datenum(clock) < finalTime
%     end
% %     pause(dt)
% end
% twist_msg.Angular.Z = 0;   % Steering
% twist_msg.Linear.X = 0;
% send(twist_chatpub,twist_msg);
% disp('done')

% % Debug script
% while(1)
%     vesc_msg.Data = -1664*1;   % Throttle
%     twist_msg.Angular.Z = 0;   % Steering
%     send(twist_chatpub,twist_msg);
%     send(vesc_chatpub,vesc_msg);
%     pause(0.05);
% end

%% Shut down global node

rosshutdown