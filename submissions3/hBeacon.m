function beaconPredict = hBeacon(tagID,robotPose,beaconLoc)
% hbeacon: the beacon measurement 
%
%   INPUTS
%       tagID: the beacon the robot detects 
%       robotPose [3x1]: robot pose [x,y,theta]
%       beaconLoc: the glocal coordinates of beacon Nx2 [x,y]
%   OUTPUTS
%       beacon measurement [x,y]
%
%
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #2
%   Yiwen Hua
%   NetID: yh686
ind = beaconLoc(:,1)==tagID;
[beaconPredict] = global2robot(robotPose,beaconLoc(ind,2:3));