function[range] = sonarPredict(robotPose,map,robotRad,angles,maxRange)
% SONARPREDICT: predict the sonar range measurements for a robot operating
% in a known map.
%
%   RANGE = SONARPREDICT(ROBOTPOSE,MAP,ROBOTRAD,ANGLES,MAXRANGE) returns
%   the expected sonar range measurements for a robot operating in a known
%   map.
%
%   INPUTS
%       robotPose   3-by-1 pose vector in global coordinates (x,y,theta)
%       map         N-by-4 matrix containing the coordinates of walls in
%                   the environment: [x1, y1, x2, y2]
%       robotRad    robot radius (meters)
%       angles      K-by-1 vector of the angular positions of the sonar
%                   sensor(s) in robot coordinates, where 0 points forward
%       maxRange    maximum sonar range (meters)
%
%   OUTPUTS
%       range       K-by-1 vector of sonar ranges (meters)
%
%
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework 3
%   Hua, Yiwen
maxRange = 3;
robotRad = 0.17;
angles = [-pi/2,0,pi/2];
Tib = [cos(robotPose(3)) -sin(robotPose(3)) robotPose(1);
    sin(robotPose(3)) cos(robotPose(3)) robotPose(2);
    0 0 1];
range = zeros(length(angles),1);
for i = 1:length(angles)
    
    wB1 = [cos(angles(i))*robotRad, sin(angles(i))*robotRad, 1]';
    wB2 = [cos(angles(i))*100, sin(angles(i))*100, 1]';
    wI1 = Tib*wB1;
    wI1(3) = [];
    wI2 = Tib*wB2;
    wI2(3) = [];
    index = 1;
    for j = 1:size(map,1)
        [isect,x,y,ua]= intersectPoint(wI1(1),wI1(2),wI2(1),wI2(2),map(j,1),map(j,2),map(j,3),map(j,4));
        if isect == true
            X(index) = x;
            Y(index) = y;
            index = index +1;
        end
    end
    measurement = 10e10;
    for j = 1:(index-1)
        measurement_c = sqrt((X(j)-wI1(1))^2+(Y(j)-wI1(2))^2);
        if measurement > measurement_c
            measurement = measurement_c;
        end
    end
    if measurement <= maxRange
        range(i) = measurement;
    else
        range(i) = maxRange;
    end
end