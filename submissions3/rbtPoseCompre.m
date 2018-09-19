function smlegh = rbtPoseCompre(Pose1,Pose2,threshold)
% rbtPoseCompre: check if the two robot poses are close enough respective
% to a threshhold
%
%   INPUTS
%       Pose1: robot1 x,y coordinates 
%       Pose2: robot2 x,y coordinates
%   OUTPUTS
%       smlehg: a binary value to show if small enough
%
%
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #2
%   Yiwen Hua
%   NetID: yh686
diff = norm(Pose2-Pose1);
if diff < threshold 
    smlegh = 1;
else
    smlegh = 0;
end