function [VisitedW,VisitedECW,VisitedWFlag,VisitedECWFlag,waypoints,ECwaypoints] = WaypointsCheck(robotPose,waypoints,ECwaypoints,closeEnough,VisitedW,VisitedECW)
VisitedWFlag = 0;
VisitedECWFlag = 0;
for i = 1:size(waypoints,1)
    Dist = findDist(robotPose(1),robotPose(2),waypoints(i,1),waypoints(i,2));
    if Dist < closeEnough
        VisitedW = [VisitedW;waypoints(i,:)];
        VisitedWFlag = 1;
        waypoints(i,:) = [];
        break
    end
end
if VisitedWFlag == 0
    for i = 1:size(ECwaypoints,1)
    Dist = findDist(robotPose(1),robotPose(2),ECwaypoints(i,1),ECwaypoints(i,2));
    if Dist < closeEnough
        VisitedECW = [VisitedECW;ECwaypoints(i,:)];
        VisitedECWFlag = 1;
        ECwaypoints(i,:) = [];
        break
    end
    end
end