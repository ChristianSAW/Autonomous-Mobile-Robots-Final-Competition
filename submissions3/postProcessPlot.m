function postProcessPlot()
load('Team3_run1.mat');
load('run1.mat');
load('compMap.mat');
load('PlannedPath.mat');

Xtp = truthLog.truthPose(:,2);
Ytp = truthLog.truthPose(:,3);
Ttp = truthLog.truthPose(:,4);

Xl = dataStore.robotPose(:,1);
Yl = dataStore.robotPose(:,2);

% plotting 
figure
hold on
plot(path(:,1),path(:,2),'-k')
plot(Xtp,Ytp,'-b')
plot(Xl,Yl,'-g')
plot(waypoints(:,1),waypoints(:,2),'sg','MarkerEdgeColor','k',...
                                             'MarkerFaceColor','g',...
                                             'MarkerSize',6);

plot(ECwaypoints(:,1),ECwaypoints(:,2),'dr','MarkerEdgeColor','k',...
                                                 'MarkerFaceColor','r',...
                                                 'MarkerSize',6);

plot(beaconLoc(:,2),beaconLoc(:,3),'ob','MarkerEdgeColor','k',...
                                                 'MarkerFaceColor','b',...
                                                 'MarkerSize',6);
                                             
plot([optWalls(2,1),optWalls(2,3)],[optWalls(2,2),optWalls(2,4)],'-m')
plotmap(map)
%plotmap(optWalls(2,:))
legend('Path','TruthPose','Localization',...
    'Waypoints','Extra Credit Waypoints','Beacon','Optional Wall')
xlabel('X Position [m]')
ylabel('Y Position [m]')
title('Final Competition Result')


% plotting Grid 
figure
hold on
plot(path(:,1),path(:,2),'-k')

plot(waypoints(:,1),waypoints(:,2),'sg','MarkerEdgeColor','k',...
                                             'MarkerFaceColor','g',...
                                             'MarkerSize',6);

plot(ECwaypoints(:,1),ECwaypoints(:,2),'dr','MarkerEdgeColor','k',...
                                                 'MarkerFaceColor','r',...
                                                 'MarkerSize',6);

plot(beaconLoc(:,2),beaconLoc(:,3),'ob','MarkerEdgeColor','k',...
                                                 'MarkerFaceColor','b',...
                                                 'MarkerSize',6);
                                             
plot([optWalls(2,1),optWalls(2,3)],[optWalls(2,2),optWalls(2,4)],'-m')
PathsFinal = globalPathFindingGrid(map,waypoints,ECwaypoints,[Xtp(1) Ytp(1)],0.17);
plotmap(map)
%plotmap(optWalls(2,:))
legend('Path','Waypoints','Extra Credit Waypoints','Beacon','Optional Wall')
xlabel('X Position [m]')
ylabel('Y Position [m]')
title('Final Competition PRM Grid')




end