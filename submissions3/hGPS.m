function GPS_m = hGPS(truthPose,map)
% This function predicts the GPS measurement based on the given information
% like overhead localization/truthPose.
% Input: truthPose(x,y,theta) [3x1]
% Output: pose of robot (x,y) [3x1]
GPS_m = zeros(3,1);
GPS_m(1) = truthPose(1);
GPS_m(2) = truthPose(2);
GPS_m(3) = truthPose(3);