% General Partical filter function
% Christian Welling
% csw73
function [PSet1, beaconEvoke] = particleFilter(PSet0,zt,ut,predict,sonarFun,map,maplimits,switchTime,beaconReading,beaconLoc,beaconEvoke,CreatePort)
% This function is to use particle filter to do the robot localization.
% Input
%       PSet: previous set [4xN] (x,y,theta,wt)'
%       ut: action input, a vector [1x2] (dist,angle)
%       zt: measurement, a vector [3x1] (x,y,theta)'
%       predict = prediction function
%       update = update function
%       maplimits:[xmax xmin ymax ymin]
%       switchTime: the time when the robot finishes the initial
%       localization
% Output
%       PSet1 [4xN]
num_pt = size(PSet0,2);
dist = ut(1);
angle = ut(2);
xt_bar = ones(4,num_pt); % [x,y,theta,wt]' coords
xt = zeros(4,num_pt);
time = toc;
ita = 0;
% SetFwdVelAngVelCreate(CreatePort, 0,0);
for i = 1:num_pt
    xt_bar(1:3,i) = feval(predict,PSet0(1:3,i), dist, angle);
    % Switch the sampling method for x,y, and theta
    if time < switchTime-1
        mu = 0;
        sigma = 0.1;            % nominally 0.3
        xt_bar(1,i) = xt_bar(1,i)+normrnd(mu,sigma);
        xt_bar(2,i) = xt_bar(2,i)+normrnd(mu,sigma);
        xt_bar(3,i) = xt_bar(3,i)+normrnd(mu,sigma);
    else
        mu = 0;
        sigma = 0.05;           % nominally 0.1
        xt_bar(1,i) = xt_bar(1,i)+normrnd(mu,sigma);
        xt_bar(2,i) = xt_bar(2,i)+normrnd(mu,sigma);
        xt_bar(3,i) = xt_bar(3,i)+normrnd(mu,sigma);
    end
    if xt_bar(3,i) > 2*pi
        xt_bar(3,i) = xt_bar(3,i) - 2*pi;
    end
    if xt_bar(3,i) < 0
        xt_bar(3,i) = 2*pi + xt_bar(3,i);
    end
    % Eliminate all the points outside the map
    if xt_bar(1,i) > maplimits(1)
        xt_bar(1,i) = maplimits(1);
    end
    if xt_bar(1,i) < maplimits(2)
        xt_bar(1,i) = maplimits(2);
    end
    if xt_bar(2,i) > maplimits(3)
        xt_bar(2,i) = maplimits(3);
    end
    if xt_bar(2,i) < maplimits(4)
        xt_bar(2,i) = maplimits(4);
    end
    zt_exp = feval(sonarFun,xt_bar(1:3,i),map);
    diff_total = 0;
    diff_beacon = 0;
    for j = 1:length(zt)
        if isnan(zt(j)) == 1 && zt_exp(j) >= 3
            diff = 0;
        elseif isnan(zt(j)) == 1 && zt_exp(j) < 3
            diff = 3;
        elseif isnan(zt(j)) == 0  && zt_exp(j) >= 3
            diff = 3;
        else
            diff = (abs(zt(j) - zt_exp(j)))^1;
        end
        % Check if there is a new beacon reading
        if beaconEvoke == 1
            % Find which beacon is detected
            beaconNum = beaconReading(end,2);
            beaconInd = beaconLoc(:,1)==beaconNum;
            for k = 1:size(map,1)
                [isect,x,y,~]= intersectPoint(xt_bar(1,i),xt_bar(2,i),beaconLoc(beaconInd,2),beaconLoc(beaconInd,3),map(k,1),map(k,2),map(k,3),map(k,4));
                if isect == 1
                    diff_beacon = 100;
                    break
                end
            end
        end
        diff_total = diff_total + diff+diff_beacon;
    end
    if diff_total == 0
        diff_total = inf;
    end
    xt_bar(4,i) =xt_bar(4,i)*1/diff_total;
    ita = ita +xt_bar(4,i);
end

beaconEvoke = 0;
% ita = 1;
xt_bar(4,:) = xt_bar(4,:)/ita;
total_wt = sum(xt_bar(4,:));
for i = 1:num_pt
    % draw i with probability of wt
    draw = rand*total_wt;
    wt_range = 0;
    for  j= 1:num_pt
        wt_range = wt_range + xt_bar(4,j);
        if draw <= wt_range
            xt(:,i) = xt_bar(:,j);
            break
        end
    end
end
PSet1 = xt;
end
