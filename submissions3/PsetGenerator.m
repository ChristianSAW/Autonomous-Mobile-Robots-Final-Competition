function [Pset] = PsetGenerator(mu, numP, covariance)
% PsetGenerator: generate particle sets based on the covariance
% 
% 
%   INPUTS
%       mu    center particle [x,y,theta]
%       numP     Number of particles, scalar
%       covariance   [1x3] (covx,covy,covt)
% 
%   OUTPUTS
%       Pset particle set
% 
% 
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #1
%   Yiwen Hua
Pset(1,:) = normrnd(mu(1),covariance(1),[1,numP]);
Pset(2,:) = normrnd(mu(2),covariance(2),[1,numP]);
Pset(3,:) = normrnd(mu(2),covariance(3),[1,numP]);
wt = rand(1,numP);
Pset = [Pset;wt];