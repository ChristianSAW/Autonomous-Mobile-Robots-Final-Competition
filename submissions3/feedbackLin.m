function [V,w] = feedbackLin(Vx,Vy,epsilon,theta)
% Vx and Vy are given with respect to the inertial frame
% epsilon is the length arm
% theta is the angle between the robot x_axis and global x_axis
result = [1 0;0 1/epsilon]*[cos(theta) sin(theta);-sin(theta) cos(theta)]*[Vx,Vy]';
V = result(1);
w = result(2);
