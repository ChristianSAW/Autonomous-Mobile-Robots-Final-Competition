function vj = VisibilityJudgeWallCircular1(map,point1,point2,robotRad)
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

vj = 1;

%Judge if the three lines are collision-free
for k = 1:size(map,1)
    length = norm([map(k,3),map(k,4)]-[map(k,1),map(k,2)]);
    alpha = atan2((map(k,4)-map(k,2)),(map(k,3)-map(k,1)));
    ver1up = [map(k,1),map(k,2)]+robotRad*[-sin(alpha),cos(alpha)]+([map(k,1),map(k,2)]-[map(k,3),map(k,4)])/length*robotRad;
    ver2up = [map(k,3),map(k,4)]+robotRad*[-sin(alpha),cos(alpha)]+([map(k,3),map(k,4)]-[map(k,1),map(k,2)])/length*robotRad;
    ver1down = [map(k,1),map(k,2)]-robotRad*[-sin(alpha),cos(alpha)]+([map(k,1),map(k,2)]-[map(k,3),map(k,4)])/length*robotRad;
    ver2down = [map(k,3),map(k,4)]-robotRad*[-sin(alpha),cos(alpha)]+([map(k,3),map(k,4)]-[map(k,1),map(k,2)])/length*robotRad;
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
            Vertices = [ver1up;ver2up;ver2down;ver1down];
            inPolygon1 = inpolygon(point1(1),point1(2),Vertices(:,1),Vertices(:,2));
            inPolygon2 = inpolygon(point2(1),point2(2),Vertices(:,1),Vertices(:,2));
            if inPolygon1 == 1 || inPolygon2 == 1
                vj = 0;
                break
            end
        end
    end
end

end