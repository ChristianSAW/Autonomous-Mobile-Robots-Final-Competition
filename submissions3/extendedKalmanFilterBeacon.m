function [mu,sigma] = extendedKalmanFilterBeacon(mu0,sigma0,zt,ut,Q,R,g,G,h,H,map,tagID,Beacon)
% General EKF which contains the predict and update.
% Given bel_bar(x0), map, state transition(dynamics) model,
% measurement model, u1:t, z1:t, Q, R
%Input: 
%        mu0:a 1x3 vector (x,y,theta)
%        sigma0: 3x3 covariance matrix
%        zt:a 3x1 vector(x,y,theta)' 
%        ut:a 1x2 vector(dist, angle)
% **********************************************************************%
% Sensor Position
% ; %a [kx1] vector

% measurement zt
% Initial configuration of the robot mu0, sigma0
% Input actions read from odometry.
% 
% Christian Welling
% csw73
%
dist = ut(1); % distance travelled, time = t;
angle = ut(2); % angle turned, time = t;
%***********************************************************************%
muBar1 = feval(g,mu0, dist, angle);
Gt = feval(G,mu0, dist, angle);
sigmaBar1 = Gt*sigma0*Gt'+R;
Ht = feval(H,tagID,mu0,Beacon); 
heval = feval(h,tagID,mu0,Beacon);

% Change the dimension due to NaN from actural sonar measurement
index = find(zt < inf);
zt = zt(index);
Q = Q(index,index);
heval = heval(index);
Ht = Ht(index,:);

Kt = sigmaBar1*Ht'*inv(Ht*sigmaBar1*Ht'+Q);
mu = muBar1+Kt*(zt'-heval');
sigma = (eye(3)-Kt*Ht)*sigmaBar1;


