function [OptWalls, map, UpdateMapFlag, ConfidenceArray] = CheckOptWalls(robotPose, map, OptWalls, SonarReadings, ConfidenceArray, CreatePort)
% CheckOptWalls: check the existance of optional walls
%
%
%   INPUTS
%       robotPose: robot's [x,y,theta] in global coordinates
%       map: a matrix(NX4) which has the vertices of the known walls [x1,y1,x2,y2]
%       OptWalls: a matrix(NX4) which has the vertices of the linear wall
%       [x1,y1,x2,y2]
%       SonarReadings: [R,F,L] Three readings from the right, front and left
%       sonar
%
%   OUTPUTS
%       NewWall: either NaN or the detected new wall
%
%
%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Final Project
%   Yiwen Hua
%   Christian Welling
robotrad = 0.17;
SonarOrien = [-pi/2,0,pi/2];
SonarRange = 1.8;                   % before it was 2.3
UpdateMapFlag = 0;
DetectThresh = 0.35;                % before it was 0.35
reverseFlag = 0;
accumulateFlag = zeros(size(OptWalls,1));
% VicinityThresh = 0.1;
% Find the sonar coordinates in glocal
% RightSonarCoords = robot2global(robotPose,[cos(SonarOrien(1)*robotrad) sin(SonarOrien(1)*robotrad)]);
% RightSonarRange = robot2global(robotPose,[cos(SonarOrien(1))*(robotrad+SonarRange) sin(SonarOrien(1))*(robotrad+SonarRange)]);
% FrontSonarCoords = robot2global(robotPose,[cos(SonarOrien(2)*robotrad) sin(SonarOrien(2)*robotrad)]);
% FrontSonarRange = robot2global(robotPose,[cos(SonarOrien(2))*(robotrad+SonarRange) sin(SonarOrien(2))*(robotrad+SonarRange)]);
% LeftSonarCoords = robot2global(robotPose,[cos(SonarOrien(3)*robotrad) sin(SonarOrien(3)*robotrad)]);
% LeftSonarRange = robot2global(robotPose,[cos(SonarOrien(3))*(robotrad+SonarRange) sin(SonarOrien(3))*(robotrad+SonarRange)]);
for i = 1:length(SonarOrien)
    % For three sonar, loop through each of them
    SonarCoords = robot2global(robotPose,[cos(SonarOrien(i))*robotrad sin(SonarOrien(i))*robotrad]);
    SonarEnd = robot2global(robotPose,[cos(SonarOrien(i))*(robotrad+SonarRange) sin(SonarOrien(i))*(robotrad+SonarRange)]);
    %     mm = plot([SonarCoords(1),SonarEnd(1)],[SonarCoords(2),SonarEnd(2)],'b-');
    %     pause(0.1)
    %     set(mm,'Visible','off');
    for j = 1:size(OptWalls,1)
        [isectvisible,x,y,~]= intersectPoint(SonarCoords(1),SonarCoords(2),SonarEnd(1),SonarEnd(2),OptWalls(j,1),OptWalls(j,2),OptWalls(j,3),OptWalls(j,4));
        if isectvisible == 1
            for k = 1:size(map,1)
                [isectcollision,~,~,~] = intersectPoint(SonarCoords(1),SonarCoords(2),x,y,map(k,1),map(k,2),map(k,3),map(k,4));
                if isectcollision == 1
                    break
                end
            end
            Dist1 = norm([x y]-[OptWalls(j,1),OptWalls(j,2)]);
            Dist2 = norm([x y]-[OptWalls(j,3),OptWalls(j,4)]);
            DistLine = findDist(OptWalls(j,1),OptWalls(j,2),OptWalls(j,3),OptWalls(j,4));
            VicinityThresh = 0.20*DistLine;
            if Dist1 > VicinityThresh && Dist2 > VicinityThresh
                VicinityCheck = 0;
            else
                VicinityCheck = 1;
            end
            if isectcollision == 0 && VicinityCheck == 0
                Dist = findDist(SonarCoords(1),SonarCoords(2),x,y);
                %                 SetFwdVelAngVelCreate(CreatePort, 0,0);
                U(1) = plot([OptWalls(j,1) OptWalls(j,3)],[OptWalls(j,2) OptWalls(j,4)],'b--');
                U(2) = plot(x,y,'*b','MarkerSize',8);
%                 U(3) = plot([SonarCoords(1),SonarEnd(1)],[SonarCoords(2),SonarEnd(2)],'y-');
                DebugCheck = Dist-SonarReadings(i);
                %                 set(U,'Visible','off')
                if abs(Dist-SonarReadings(i)) < DetectThresh
                    if ConfidenceArray(j) < 0 
                        accumulateFlag(j) = j;
                        %                         break
                    end
                    if accumulateFlag(j) == 1
                        reverseFlag = j;
                        accumulateFlag(j) = 0;
                    end
                    U(1) = plot([OptWalls(j,1) OptWalls(j,3)],[OptWalls(j,2) OptWalls(j,4)],'r--');
                U(2) = plot(x,y,'*r','MarkerSize',8,'LineWidth',4);
%                 U(3) = plot([SonarCoords(1),SonarEnd(1)],[SonarCoords(2),SonarEnd(2)],'y-');
                    ConfidenceArray(j) = ConfidenceArray(j)+1;
                    disp('Success')
                    disp('IntersectPoints')
                    disp(x)
                    disp(y)
                    disp('CheckingDistance')
                    Fun = Dist-SonarReadings(i);
                    disp(Fun)
                    disp('ConfidenceArray')
                    disp(ConfidenceArray)
                    disp('..................')
                    txt = '\leftarrow T';
                    text(x,y,txt,'Color','r')
                else
                    U(1) = plot([OptWalls(j,1) OptWalls(j,3)],[OptWalls(j,2) OptWalls(j,4)],'b--');
                U(2) = plot(x,y,'*b','MarkerSize',8,'LineWidth',4);
%                 U(3) = plot([SonarCoords(1),SonarEnd(1)],[SonarCoords(2),SonarEnd(2)],'y-');
                    ConfidenceArray(j) = ConfidenceArray(j) - 1;
                    disp('Failue')
                    disp('IntersectPoints')
                    disp(x)
                    disp(y)
                    disp('CheckingDistance')
                    Fun = Dist-SonarReadings(i);
                    disp(Fun)
                    disp('ConfidenceArray')
                    disp(ConfidenceArray)
                    disp('..................')
                    txt = '\leftarrow F';
                    text(x,y,txt,'Color','b')
                    %                     plot([SonarCoords(1),SonarEnd(1)],[SonarCoords(2),SonarEnd(2)],'b-');
                end
            end
        end
    end
end
% SetFwdVelAngVelCreate(CreatePort, 0,0);
WallYesInd = find(ConfidenceArray > 1);
WallNoInd = find(ConfidenceArray < -2);
% SetFwdVelAngVelCreate(CreatePort, 0,0);
if isempty(WallYesInd) == 0
    for i = 1:size(WallYesInd,1)
        disp('The optional wall with the following vertice exist')
        disp([OptWalls(WallYesInd(i),1) OptWalls(WallYesInd(i),2) OptWalls(WallYesInd(i),3) OptWalls(WallYesInd(i),4)])
        map = [map;OptWalls(WallYesInd(i),:)];
        UpdateMapFlag = 1;
        ConfidenceArray(WallYesInd(i)) = [];
        OptWalls(WallYesInd(i),:) = [];
    end
end
if isempty(WallNoInd) == 0
    for i = 1:size(WallNoInd,1)
        ConfidenceArray(WallNoInd(i)) = [];
        disp('The optional wall with the following vertice does not exist')
        disp([OptWalls(WallNoInd(i),1) OptWalls(WallNoInd(i),2) OptWalls(WallNoInd(i),3) OptWalls(WallNoInd(i),4)])
        OptWalls(WallNoInd(i),:) = [];
    end
end
if reverseFlag ~= 0
    disp('The optional wall with the following vertice exist')
    disp([OptWalls(reverseFlag,1) OptWalls(reverseFlag,2) OptWalls(reverseFlag,3) OptWalls(reverseFlag,4)])
    map = [map;OptWalls(reverseFlag,:)];
    UpdateMapFlag = 1;
    ConfidenceArray(reverseFlag) = [];
    OptWalls(reverseFlag,:) = [];
end
% for j = 1:size(OptWalls,1)
%     if ConfidenceArray(j) >= 2
%         map = [map;OptWalls(j,:)];
%         UpdateMapFlag = 1;
%         ConfidenceArray(j) = [];
%         OptWalls(j,:) = [];
%         break
%     end
%     if ConfidenceArray(j) < -2
%         disp('The optional wall with the following vertice does not exist')
%         disp([OptWalls(j,1) OptWalls(j,2) OptWalls(j,3) OptWalls(j,4)])
%         ConfidenceArray(j) = [];
%         OptWalls(j,:) = [];
%     end
% end
end

