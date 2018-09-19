function wm = withinMapCircular(mapLimits,point,robotRad)
%Judge whether two circles/cicular robots are visible to each other
%mapLimits       1-by-4 vector defining the map boundary, where mapLimits = [x_min y_min x_max y_max]
%point           Coordinate of the position,1x2
%testPointNum    Number of the points used to judge whether a line is collision-free
%robotRad        Radius of the two two circles/robots

if (mapLimits(1)+robotRad) < point(1) && point(1) < (mapLimits(3)-robotRad) && (mapLimits(2)+robotRad) < point(2) && point(2) < (mapLimits(4)-robotRad) 
    wm = 1;
else
    wm = 0;
end
end