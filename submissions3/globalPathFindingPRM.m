function PathsFinal = globalPathFindingPRM(map,waypoints,ECwaypoints,initPoint,robotRad)
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
% 
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
% title('Local Path Finding')
% %legend(H,'current map','waypoints','ECwaypoints','current location','local area')
% axis([mapLimits(1) mapLimits(3) mapLimits(2) mapLimits(4)])
% grid
% 

%%Generate PRM using low-discrepancy method
pointNum = 500;
p = haltonset(2,'Skip',1e3,'Leap',1e2);
p = scramble(p,'RR2');
pointLowD = net(p,pointNum);
stopPoints = [initPoint;waypoints;ECwaypoints];
%Node, including initPoint, waypoints and ECwaypoints
% X = [stopPoints(:,1);mapLimits(1)+pointLowD(:,1)*(mapLimits(3)-mapLimits(1))];
% Y = [stopPoints(:,2);mapLimits(2)+pointLowD(:,2)*(mapLimits(4)-mapLimits(2))];
X = [stopPoints(:,1);mapLimits(1)+pointLowD(:,1)*(mapLimits(3)-mapLimits(1))];
Y = [stopPoints(:,2);mapLimits(2)+pointLowD(:,2)*(mapLimits(4)-mapLimits(2))];
Node = [X,Y];
% plot(X,Y,'ro')
Connectivity = zeros(size(X,1));

%Connect 5 nearest points (trying 20 nearest points)
ConnectTry = 20;
ConnectNum = 5;
for i = 1
    Distance = [];
    for j = 1:size(X,1)
        distance = norm([X(i),Y(i)]-[X(j),Y(j)]);
        Distance = [Distance;distance];
    end
    DistanceSorted = sort(Distance);
    DistanceConnect = DistanceSorted(2:(ConnectTry+1));
    for p = 1:ConnectTry
        index = find(Distance == DistanceConnect(p));
        vj = VisibilityJudgeWallCircularInit(map,[X(i),Y(i)],[X(index),Y(index)],robotRad);
        if vj == 1 && sum(Connectivity(i,:)) < ConnectNum && sum(Connectivity(index,:)) < ConnectNum
%             plot([X(i),X(index)],[Y(i),Y(index)],'b');
            Connectivity(i,index) = 1;
            Connectivity(index,i) = 1;
        end
    end
end

for i = 2:size(X,1)
    Distance = [];
    for j = 1:size(X,1)
        distance = norm([X(i),Y(i)]-[X(j),Y(j)]);
        Distance = [Distance;distance];
    end
    DistanceSorted = sort(Distance);
    DistanceConnect = DistanceSorted(2:(ConnectTry+1));
    for p = 1:ConnectTry
        index = find(Distance == DistanceConnect(p));
        vj = VisibilityJudgeWallCircular2(map,[X(i),Y(i)],[X(index),Y(index)],robotRad);
        if vj == 1 && sum(Connectivity(i,:)) < ConnectNum && sum(Connectivity(index,:)) < ConnectNum
%             plot([X(i),X(index)],[Y(i),Y(index)],'b');
            Connectivity(i,index) = 1;
            Connectivity(index,i) = 1;
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