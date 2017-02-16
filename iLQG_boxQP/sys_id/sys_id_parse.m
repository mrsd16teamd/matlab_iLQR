%% Script to parse ROSbags for SysID tests

function sys_id_parse(bagfilename)

  if nargin < 1
    bagfilename = 'stateestimate.bag';
  end

% Read data
bag = rosbag(bagfilename);

odometry_filtered = readMessages(select(bag,'Topic','/odometry/filtered'));
amcl_pose = readMessages(select(bag,'Topic','/amcl_pose'));
cmd_vel = readMessages(select(bag,'Topic','/cmd_vel'));

%% Parse data

% vel = [time, vx, vy, wz]
vel = zeros(length(odometry_filtered),4);
for i=1:length(odometry_filtered)
    vel(i,1) = odometry_filtered{i}.Header.Stamp.Nsec;
    vel(i,2) = odometry_filtered{i}.Twist.Twist.Linear.X;
    vel(i,3) = odometry_filtered{i}.Twist.Twist.Linear.Y;
    vel(i,4) = odometry_filtered{i}.Twist.Twist.Angular.Z;
end

% pos = [time, x, y, theta]
pos = zeros(length(amcl_pose),4);
for i=1:length(amcl_pose)
    pos(i,1) = amcl_pose{i}.Header.Stamp.Nsec;
    pos(i,2) = amcl_pose{i}.Pose.Pose.Position.X;
    pos(i,3) = amcl_pose{i}.Pose.Pose.Position.Y;
    x = amcl_pose{i}.Pose.Pose.Orientation.X;
    y = amcl_pose{i}.Pose.Pose.Orientation.Y;
    z = amcl_pose{i}.Pose.Pose.Orientation.Z;
    w = amcl_pose{i}.Pose.Pose.Orientation.W;
    ori = quat2eul([w,x,y,z]);
    pos(i,4) = ori(1);
end
clear('x','y','z','w');

% u = [throttle, steering]
u = zeros(length(cmd_vel),2);
for i=1:length(cmd_vel)
    u(i,1) = cmd_vel{i}.Header.Stamp.Nsec;
    u(i,2) = cmd_vel{i}.Twist.Linear.X;
    u(i,4) = cmd_vel{i}.Twist.Angular.Z;
end
end
