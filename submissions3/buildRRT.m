function [path,costs] = buildRRT(map,initPoint,goalPoint,robotRad)
%Build Rapidly-exploring Random Trees (point robot) using uniform random sampling
%currenetMap     Current map
%initPoint       Coordinate of the initial point,1x2
%goalPoint       Coordinate of the goal point,1x2
%robotRad        Radius of the robot
%path            A matrix containing all the points from iniiPoint to goalPoint,nx2
%H               Figure handle, output for legending use
% 
% Christian Welling
% csw73
%
mapLimits = [min(map(:,1)) min(map(:,2)) max(map(:,1)) max(map(:,2))];
stepSize = 0.75;

Node = initPoint;
familyTree = [];

while(1)
    %Judge whether the goal is visible from the current node
    vjGoal = VisibilityJudgeWallCircular(map,Node(end,:),goalPoint,robotRad);
    if vjGoal == 1
        Node = [Node;goalPoint];
        familyTree = [familyTree;size(Node,1)-1,size(Node,1)];
        break
    else
        %Generate new point q_rand
        pointnew = [mapLimits(1)+rand*(mapLimits(3)-mapLimits(1)),mapLimits(2)+rand*(mapLimits(4)-mapLimits(2))];
        distance = zeros(size(Node,1),1);
        for i = 1:size(Node,1)
            distance(i) = norm(Node(i,:)-pointnew);
        end
        index = find(distance == min(distance));
        pointInTree = Node(index,:);
        pointNew = pointInTree+stepSize*(pointnew-pointInTree)/norm((pointnew-pointInTree));
        %Judge whether q_near is visible from q_rand
        vj = VisibilityJudgeWallCircular(map,pointInTree,pointNew,robotRad);
        wm = withinMapCircular(mapLimits,pointNew,robotRad);
        if vj == 1  && wm == 1
            Node = [Node;pointNew];
            familyTree = [familyTree;index,size(Node,1)];
        end
    end
end

path = goalPoint;
child = familyTree(end,2);
while(1)
    parent = familyTree(find(familyTree(:,2) == child),1);
    path = [Node(parent,:);path];
    if parent == 1
        break
    else
        child = parent;
    end
end

costs = 0;
for i = 1:size(path,1)-1
    costs = costs+norm(path(i+1,:)-path(i,:));
end
%     legend(H,'Initial point','Goal point','RRT','Path')
end