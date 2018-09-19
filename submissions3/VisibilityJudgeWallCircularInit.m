function vj = VisibilityJudgeWallCircularInit(map,point1,point2,robotRad)
%Judge whether two circles/cicular robots are visible to each other
%map             Map with edges and walls,Nx4
%point1(2)       Coordinate of the two point2,1x2
%robotRad        Radius of the two two circles/robots

% for i = 1:size(map,1)
%     H(1) = plot([map(i,1),map(i,3)],[map(i,2),map(i,4)],'k');
%     hold on
% end
%
% plot([point1(1),point2(1)],[point1(2),point2(2)],'r')
% hold on

thickness = 0.056;
vj = 1;
mapLimits = [min(map(:,1)) min(map(:,2)) max(map(:,1)) max(map(:,2))];
%Map       Inner walls
indexLB = find(map(:,1) == min(map(:,1)) & map(:,2) == min(map(:,2)));
indexLT = find(map(:,1) == min(map(:,1)) & map(:,2) == max(map(:,2)));
indexRT = find(map(:,1) == max(map(:,1)) & map(:,2) == max(map(:,2)));
indexRB= find(map(:,1) == max(map(:,1)) & map(:,2) == min(map(:,2)));
[Map,ps] = removerows(map,'ind',[indexLB,indexLT,indexRT,indexRB]);

%Judge if the three lines are collision-free
for k = 1:size(Map,1)
    length = norm([Map(k,3),Map(k,4)]-[Map(k,1),Map(k,2)]);
    alpha = atan2((Map(k,4)-Map(k,2)),(Map(k,3)-Map(k,1)));
    ver1up = [Map(k,1),Map(k,2)]+(robotRad+thickness)*[-sin(alpha),cos(alpha)]+([Map(k,1),Map(k,2)]-[Map(k,3),Map(k,4)])/length*robotRad;
    ver2up = [Map(k,3),Map(k,4)]+(robotRad+thickness)*[-sin(alpha),cos(alpha)]+([Map(k,3),Map(k,4)]-[Map(k,1),Map(k,2)])/length*robotRad;
    ver1down = [Map(k,1),Map(k,2)]-(robotRad+thickness)*[-sin(alpha),cos(alpha)]+([Map(k,1),Map(k,2)]-[Map(k,3),Map(k,4)])/length*robotRad;
    ver2down = [Map(k,3),Map(k,4)]-(robotRad+thickness)*[-sin(alpha),cos(alpha)]+([Map(k,3),Map(k,4)]-[Map(k,1),Map(k,2)])/length*robotRad;
%     plot([ver1up(1),ver1down(1)],[ver1up(2),ver1down(2)],'b')
%     plot([ver1up(1),ver2up(1)],[ver1up(2),ver2up(2)],'b')
%     plot([ver2up(1),ver2down(1)],[ver2up(2),ver2down(2)],'b')
%     plot([ver1down(1),ver2down(1)],[ver1down(2),ver2down(2)],'b')
    [isectup,x,y,ua]= intersectPoint(point1(1),point1(2),point2(1),point2(2),ver1up(1),ver1up(2),ver2up(1),ver2up(2));
    if isectup == 1
        vj = 0;
        break
    else
        [isectdown,x,y,ua]= intersectPoint(point1(1),point1(2),point2(1),point2(2),ver1down(1),ver1down(2),ver2down(1),ver2down(2));
        if isectdown == 1
            vj = 0;
            break
        else
            [isectleft,x,y,ua]= intersectPoint(point1(1),point1(2),point2(1),point2(2),ver1up(1),ver1up(2),ver1down(1),ver1down(2));
            if isectleft == 1
                vj = 0;
                break
            else
                [isectright,x,y,ua]= intersectPoint(point1(1),point1(2),point2(1),point2(2),ver2up(1),ver2up(2),ver2down(1),ver2down(2));
                if isectright == 1
                    vj = 0;
                    break
                else
                    Vertices = [ver1up;ver2up;ver2down;ver1down];
                    inPolygon2 = inpolygon(point2(1),point2(2),Vertices(:,1),Vertices(:,2));
                    if inPolygon2 == 1
                        vj = 0;
                        break
                    else
                        wm2 = withinMapCircular(mapLimits,point2,robotRad);
                        if wm2 == 0
                            vj = 0;
                            break
                        end
                    end
                end
            end
        end
    end    
end