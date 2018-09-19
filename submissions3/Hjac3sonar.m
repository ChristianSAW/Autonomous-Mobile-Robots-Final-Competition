function Hsonart = Hjac3sonar(robotPose,map)
maxRange = 3;
angles = [pi/2,0,-pi/2];
robotRad = 0.2;
deltax = 1/1000;
deltay = 1/1000;
delta = 1/1000;
[sma] = sonarPredict(robotPose,map);
%{**********************************************************************}%
hx = [deltax,0,0]';
robotPosevx = robotPose+hx;
[smahx] = sonarPredict(robotPosevx,map);
Hsonart(1,1) = (smahx(1)-sma(1))/deltax;
Hsonart(2,1) = (smahx(2)-sma(2))/deltax;
Hsonart(3,1) = (smahx(3)-sma(3))/deltax;
%{**********************************************************************}%
hy = [0,deltay,0]';
robotPosevy = robotPose+hy;
[smahy] = sonarPredict(robotPosevy,map);
Hsonart(1,2) = (smahy(1)-sma(1))/deltay;
Hsonart(2,2) = (smahy(2)-sma(2))/deltay;
Hsonart(3,2) = (smahy(3)-sma(3))/deltay;
%{**********************************************************************}%
ha = [0,0,delta]';
robotPoseva = robotPose+ha;
[smaha] = sonarPredict(robotPoseva,map);
Hsonart(1,3) = (smaha(1)-sma(1))/delta;
Hsonart(2,3) = (smaha(2)-sma(2))/delta;
Hsonart(3,3) = (smaha(3)-sma(3))/delta;
% Hsonart(1,3) = (smaha(1)-sma(1))/rad2deg(delta);
% Hsonart(2,3) = (smaha(2)-sma(2))/rad2deg(delta);
% Hsonart(3,3) = (smaha(3)-sma(3))/rad2deg(delta);