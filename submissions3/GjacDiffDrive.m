function Gt = GjacDiffDrive(ini_pose,d,phi)
% This function is to find the jacobian matrix of the dynamics function
% Input: given pose(x,y,theta) [1x3]
% Output: Jacobian Matrix, Gt [3x3]
% syms d phi
x = ini_pose(1);
y = ini_pose(2);
theta = ini_pose(3);
if phi ~= 0
Gt = [1 0 d/phi*(cos(theta+phi)-cos(theta));...
    0 1 d/phi*(sin(theta+phi)-sin(theta));...
    0 0 1];
else
    Gt = [1 0 -d*sin(theta);
          0 1 d*cos(theta);
          0 0 1];
end