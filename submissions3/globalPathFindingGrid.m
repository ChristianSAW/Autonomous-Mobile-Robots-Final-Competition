function PathsFinal = globalPathFindingGrid(map,waypoints,ECwaypoints,initPoint,robotRad)
%%Find a path from initial point to all waypoints with minimum costs
%map             Current map,Lx4
%waypoints       Waypoints in the map,Nx2
%ECwaypoints     Extra credit waypoints in the map,Mx2
%initPoint       Coordinate of the initial point,1x2
%robotRad        Radius of the robot
%PathsFinal      A matrix cosisting of all points on the path, Px2
%AMR 2017 Final Competition
%Yejing Wang, yw687

% map = [map;[-3 3.5 -2 3.5]];

%map limits [xmin ymin xmax ymax]
mapLimits = [min(map(:,1)) min(map(:,2)) max(map(:,1)) max(map(:,2))];

% %%plot
% %plot map
% figure
% for i = 1:size(map,1)
%     plot([map(i,1),map(i,3)],[map(i,2),map(i,4)],'k');
%     hold on
% end
% 
% %plot waypoints
% for i = 1:size(waypoints,1)
%     plot(waypoints(i,1),waypoints(i,2),'sg','MarkerEdgeColor','k',...
%                                             'MarkerFaceColor','g',...
%                                             'MarkerSize',6);
% end
% 
% %plot ECwaypoints
% for i = 1:size(ECwaypoints,1)
%     plot(ECwaypoints(i,1),ECwaypoints(i,2),'dr','MarkerEdgeColor','k',...
%                                                 'MarkerFaceColor','r',...
%                                                 'MarkerSize',6);
% end
% 
% plot(initPoint(1),initPoint(2),'xk');
% 
% title('Global Path Finding — Grid')
% %legend(H,'current map','waypoints','ECwaypoints','current location','local area')
% axis([mapLimits(1) mapLimits(3) mapLimits(2) mapLimits(4)])
% grid
% 

%%Generate PRM using low-dispersion method
stopPoints = [initPoint;waypoints;ECwaypoints];
step = 0.25;
%Node, including initPoint, waypoints and ECwaypoints
[Xm,Ym] = meshgrid(mapLimits(1)+0.18:step:mapLimits(3),mapLimits(2)-0.1:step:mapLimits(4));
Xmr = reshape(Xm,size(Xm,1)*size(Xm,2),1);
Ymr = reshape(Ym,size(Ym,1)*size(Ym,2),1);
X = [stopPoints(:,1);Xmr];
Y = [stopPoints(:,2);Ymr];
Node = [X,Y];
plot(X,Y,'ro')
Connectivity = zeros(size(X,1));

for i = 1
    for j = (i+1):size(X,1)
        distance = norm([X(i),Y(i)]-[X(j),Y(j)]);
        vj = VisibilityJudgeWallCircularInit(map,[X(i),Y(i)],[X(j),Y(j)],robotRad);
        if distance <= step && vj == 1
             plot([X(i),X(j)],[Y(i),Y(j)],'b');
            Connectivity(i,j) = 1;
            Connectivity(j,i) = 1;
        end
    end
end

for i = 2:size(X,1)
    for j = (i+1):size(X,1)
        distance = norm([X(i),Y(i)]-[X(j),Y(j)]);
        vj = VisibilityJudgeWallCircularGrid(map,[X(i),Y(i)],[X(j),Y(j)],robotRad);
        if distance <= step && vj == 1
             plot([X(i),X(j)],[Y(i),Y(j)],'b');
            Connectivity(i,j) = 1;
            Connectivity(j,i) = 1;
        end
    end
end


%%Calculate paths and costs between stopPoints
%Cost                       A matrix whose element Cost(i,j) is the cost from ith waypoint 
%                           to jth waypoint,RxR
%ConnectivityStopPoints     Connectivity matrix of waypoints, 0s on diagonal,RxR
%Paths                      A cell whose Paths{i,j} is the matrix containg all points on 
%                           the path from ith waypoint to jth waypoint
Cost = zeros(size(stopPoints,1));
ConnectivityStopPoints = zeros(size(stopPoints,1));
Paths = cell(size(stopPoints,1)-1);
for i = 1:(size(stopPoints,1)-1)
    for j = (i+1):size(stopPoints,1)
% for i = 5
%     for j = 9
        [costs,paths] = dijkstra(Connectivity,Node,i,j);
        if j <= size(waypoints,1)+1
            Cost(i,j) = costs;
            Cost(j,i) = costs;
        else
            Cost(i,j) = costs/2;
            Cost(j,i) = costs/2;
        end
        ConnectivityStopPoints(i,j) = 1;
        ConnectivityStopPoints(j,i) = 1;
        Paths{i,j} = paths;
        Paths{j,i} = fliplr(paths);
%         for k = 1:(size(paths,2)-1)
%             plot([Node(Paths{j,i}(k),1),Node(Paths{j,i}(k+1),1)],[Node(Paths{j,i}(k),2),Node(Paths{j,i}(k+1),2)],'r','LineWidth',2);
% %             plot([Node(Paths{i,j}(k),1),Node(Paths{i,j}(k+1),1)],[Node(Paths{i,j}(k),2),Node(Paths{i,j}(k+1),2)],'g','LineWidth',2);
%         end
    end
end

%%Calculate costs of all possible paths among stopPoints
%Perm         All possible permutation of waypoints,Qx2
%CostAll      Cost matrix of possible paths connecting waypoints,Qx2 
Perm = perms(1:(size(stopPoints,1)-1));
CostAll = zeros((size(stopPoints,1)-1),1);
for i = 1:size(Perm,1)
    Way = Perm(i,:);
    CostWay = Cost(1,Way(1)+1);
    for j = 1:size(Way,2)-1
        CostWay = CostWay+Cost(Way(j)+1,Way(j+1)+1);
    end
    CostAll(i) = CostWay;
end

%Get sequence of stopPoints with minimum cost
WaySelected = Perm(find(CostAll == min(CostAll)),:);
if size(WaySelected,1) ~= 1
    WaySelected = WaySelected(1,:);
end

%Generate final path
PathsFinalIndex = Paths{1,WaySelected(1)+1};
for i = 1:size(WaySelected,2)-1
    PathsFinalIndex = [PathsFinalIndex,Paths{WaySelected(i)+1,WaySelected(i+1)+1}(2:end)];
end

%Plot final path
PathsFinal = zeros(size(PathsFinalIndex,2),2);
for i = 1:size(PathsFinalIndex,2)
    PathsFinal(i,:) = Node(PathsFinalIndex(i),:);
end

% for i = 1:size(PathsFinalIndex,2)-1
%     plot([PathsFinal(i,1),PathsFinal(i+1,1)],[PathsFinal(i,2),PathsFinal(i+1,2)],'r','LineWidth',2);
% end

% for i = 1:size(PathsFinalIndex,2)-1
%     plot([Node(PathsFinalIndex(i),1),Node(PathsFinalIndex(i+1),1)],[Node(PathsFinalIndex(i),2),Node(PathsFinalIndex(i+1),2)],'r','LineWidth',2);
% end
end