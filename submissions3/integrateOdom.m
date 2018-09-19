function [configuration,initial_pose] = integrateOdom(initial_pose, dist, angle)
% integrateOdom: calculate the configuration of the robot using dead
% reckoning techniques
%
%   [configuration] = integrateOdom(initial_pose, dist, angle) returns the
%   configuration of the robot based on the inputs given
%
%   INPUTS
%       initial_pose      the initial pose of the robot, 3-by-1 vector
%       representing [x,y,theta]'
%       dist      a 1xN vector read from DistanceSensorRoomba representing the
%       distance travelled
%       angle     a 1xN vector read from AngleSensorRoomba representing the
%       angle the robot turned
%   OUTPUTS
%       configuration     A 3-by-1 vector representing the current pose
%
%
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #2
%   Yiwen Hua
%   NetID: yh686
count = size(dist,2);
configuration = zeros(3,count+1);
configuration(:,1) = initial_pose;
for i = 1:count
    if angle(i) ~= 0
        x = dist(i)/angle(i)*(sin(angle(i)+initial_pose(3))-sin(initial_pose(3)))+initial_pose(1);
        y = dist(i)/angle(i)*(cos(initial_pose(3))-cos(angle(i)+initial_pose(3)))+initial_pose(2);
        theta = angle(i)+initial_pose(3);
        configuration(:,i+1) = [x, y, theta]';
        initial_pose = configuration(:,i+1);
    else
        x = dist(i)*cos(initial_pose(3))+initial_pose(1);
        y = dist(i)*sin(initial_pose(3))+initial_pose(2);
        theta = initial_pose(3);
        configuration(:,i+1) = [x, y, theta]';
        initial_pose = configuration(:,i+1);
    end
end
configuration = configuration(:,end);