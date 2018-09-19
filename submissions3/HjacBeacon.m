function [Hbeacont] = HjacBeacon(tagID,robotPose,beaconLoc)

Hbeacont = zeros(2,3);

x = robotPose(1);
y = robotPose(2);
th = robotPose(3);

beacon = hBeacon(tagID,robotPose,beaconLoc);

delta = 0.0001;

xPlusdx = x+delta;
yPlusdy = y+delta;
thPlusdth = th+delta;

poseX = [xPlusdx,y,th];
poseY = [x,yPlusdy,th];
poseth = [x,y,thPlusdth];

beaconX = hBeacon(tagID,poseX,beaconLoc);
beaconY = hBeacon(tagID,poseY,beaconLoc);
beaconth = hBeacon(tagID,poseth,beaconLoc);

Hbeacont(:,1) = (beaconX-beacon)/delta;
Hbeacont(:,2) = (beaconY-beacon)/delta;
Hbeacont(:,3) = (beaconth-beacon)/delta;

end
