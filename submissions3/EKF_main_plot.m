close all
clc
global dataStore
load('cornerMap.mat')
plot(dataStore.truthPose(:,2),dataStore.truthPose(:,3),'o')
hold on 
plot(dataStore.deadReck(1,:),dataStore.deadReck(2,:),'*')
plot(dataStore.ekfMu(:,1),dataStore.ekfMu(:,2),'x')

% plotmap(cornerMap)

map=cornerMap;
for i = 1:size(map,1)
    plot([map(i,1),map(i,3)],[map(i,2),map(i,4)],'-r')
    hold on
end
plotCovEllipse(dataStore.ekfMu(end,1:2),dataStore.ekfSigma(1:2,1:2))
title('EKF with GPS data')
legend('truthPose','deadReck','EKF_mu','Wall','Gaussian Ecllipse')