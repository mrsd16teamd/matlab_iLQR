%% Subscribe and record specific topic for SysID experiments
% Run this whole script, not by section!

function sys_id_record

% tx1 = rosdevice('192.168.1.118','ubuntu','ubuntu');

%% Initialize Global ROS node if not already active
try
    rosinit('http://192.168.1.118:11311')
catch EXP
    if string(EXP.identifier)=='robotics:ros:node:GlobalNodeNotRunning'
        disp('ROS Master not running!')
    elseif string(EXP.identifier)=='robotics:ros:node:NoMasterConnection'
        disp('Cannot connect to ROS master! Check network')
        return
    end
    clear EXP
end

%% Initialize Subscribers

amcl_sub = rossubscriber('/amcl_pose_echo', 'geometry_msgs/PoseWithCovarianceStamped', 'BufferSize', 10);
twist_sub = rossubscriber('/cmd_vel', 'geometry_msgs/Twist', 'BufferSize', 10);
odom_sub = rossubscriber('/odometry/filtered', 'nav_msgs/Odometry', 'BufferSize',10);

odom_msg = odom_sub.LatestMessage;
amcl_msg = amcl_sub.LatestMessage;
twist_msg = twist_sub.LatestMessage;

%% Odom callback

global stateData
stateData = struct([]);
% stateData is a struct with 3 fields:
% Header
% X = [x,y,theta,vx,vy,wz]
% U = [steer,throttle]

global KEY_IS_PRESSED
KEY_IS_PRESSED = 0;
gcf
set(gcf, 'KeyPressFcn', @myKeyPressFcn)

disp('Press SPACE to stop recording')

odom_sub.NewMessageFcn = {@odom_sub_callback,amcl_sub,twist_sub};

while ~KEY_IS_PRESSED
      drawnow
end

disp('Done, saving mat file...')
close all;

% file_name = ['stateData',datestr(now,'_mm-dd-yy_HH:MM')];
file_name = 'ramp_3right_v2';
save([file_name,'.mat'],'stateData');

rosshutdown

end

function odom_sub_callback(~,odom_msg,amcl_sub,twist_sub)

global stateData

% stateData is a struct with 3 fields:
% Header
% X = [x,y,theta,vx,vy,wz]
% U = [steer,throttle]


amcl_msg = amcl_sub.LatestMessage;
twist_msg = twist_sub.LatestMessage;

vx = odom_msg.Twist.Twist.Linear.X;
vy = odom_msg.Twist.Twist.Linear.Y;
wz = odom_msg.Twist.Twist.Angular.Z;

x = amcl_msg.Pose.Pose.Position.X;
y = amcl_msg.Pose.Pose.Position.Y;

qx = amcl_msg.Pose.Pose.Orientation.X;
qy = amcl_msg.Pose.Pose.Orientation.Y;
qz = amcl_msg.Pose.Pose.Orientation.Z;
qw = amcl_msg.Pose.Pose.Orientation.W;
ori = quat2eul([qw,qx,qy,qz]);
theta = ori(1);

throttle = twist_msg.Linear.X;
steer = twist_msg.Angular.Z;

state.Header = odom_msg.Header;
state.X = [x,y,theta,vx,vy,wz];
state.U = [throttle,steer];

stateData = [stateData;state];

end

function myKeyPressFcn(~, event)
global KEY_IS_PRESSED
if strcmp(event.Key,'space')
    KEY_IS_PRESSED  = 1;
end
disp('key is pressed') 
end