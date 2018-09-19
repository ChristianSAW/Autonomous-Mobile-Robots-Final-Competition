function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
% 
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,WHEEL2CENTER) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot whose wheel motors saturate at +/- maxV.
% 
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       wheel2Center distance from the wheels to the center of the robot(in meters)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
% 
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #1
%   Yiwen Hua
% The left and front wheels are limited to +-0.5m/s
% velocity of the right wheel
vR = (2*fwdVel+angVel*2*wheel2Center)/2;
% velocity of the left wheel
vL = (2*fwdVel-angVel*2*wheel2Center)/2;
scale_factor = 1;
if abs(vR) >= maxV && abs(vL) < maxV
    scale_factor = abs(vR)/(maxV-0.001);
end
if abs(vL) >= maxV && abs(vR) < maxV
    scale_factor = abs(vL)/(maxV-0.001);
end
if abs(vR) >= maxV && abs(vL) >= maxV
    if abs(vR) > abs(vL)
        scale_factor = abs(vR)/(maxV-0.001);
    else 
        scale_factor = abs(vL)/(maxV-0.001);
    end
end
vR = vR/scale_factor;
vL = vL/scale_factor;
cmdV = (vR+vL)/2;
cmdW = (vR-vL)/2/wheel2Center;