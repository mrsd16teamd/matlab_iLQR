%% Subscribe and record specific topic for SysID experiments

function sys_id_record

tx1 = rosdevice('192.168.1.118','ubuntu','ubuntu');

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
twist_sub = rossubscriber('/cmd_vel_stamped', 'geometry_msgs/TwistStamped', 'BufferSize', 10);
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

while ~KEY_IS_PRESSED
      drawnow
      odom_sub.NewMessageFcn = {@odom_sub_callback,amcl_sub,twist_sub};
end

disp('done')

file_name = ['stateData',datestr(now,'_mm-dd-yy_HH:MM')];
save([file_name,'.mat'],'stateData');


end

function myKeyPressFcn(~, event)
global KEY_IS_PRESSED
if strcmp(event.Key,'space')
    KEY_IS_PRESSED  = 1;
end
disp('key is pressed') 
end