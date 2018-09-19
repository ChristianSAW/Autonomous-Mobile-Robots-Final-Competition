function plotmap(map)
for i = 1:size(map,1)
    plot([map(i,1),map(i,3)],[map(i,2),map(i,4)],'-r')
    hold on
%     pause(0.5)
end