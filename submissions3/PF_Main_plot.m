% Particle Filter Main Plot
clf
load('cornerMap.mat')
% figure(1)
% for i = 1:size(dataStore.initialParticles,2)
%     x1 = dataStore.initialParticles(1,i);
%     y1 = dataStore.initialParticles(2,i);
%     theta = dataStore.initialParticles(3,i);
%     x2 = x1+cos(theta)*0.5;
%     y2 = y1+sin(theta)*0.5;
%     plot(x1,y1,'og')
%     hold on
%     arrow([x1,y1],[x2,y2])
% end
% plotmap(cornerMap)
% legend('Initial Particles')
% title('Initial Particle Set with headings')
% hold off

% figure(2)
% plotmap(cornerMap)
% hold on
% plot(dataStore.particles(1,:),dataStore.particles(2,:),'ob')
% hold off

% Plot the six trajectories
figure(1)
% True Trajectory

% load('owen_particle_50.mat')
% load('yejingwangLabPlot.mat')
plot(dataStore.truthPose(:,2),dataStore.truthPose(:,3),'.-g')
hold on
plotmap(cornerMap)
% estimate trajectory
for i = 1:length(dataStore.PFt)
h(i)=plot(dataStore.PFt(i,1),dataStore.PFt(i,2),'bx');
pause(0.01)
end
set(h,'Visible','off')
plot(dataStore.PFt(end,1),dataStore.PFt(end,2),'mo','MarkerSize',5,'LineWidth',5)
% load('yejingwangLabPlot1.mat')
% estimate trajectory
% plot(dataStore.PFt(:,1),dataStore.PFt(:,2),'-mx')
% % Initial particles
% plot(dataStore.stepParticles(1,:,1),dataStore.stepParticles(2,:,1),'x-k')
% % Final particle set
% plot(dataStore.particles(1,:),dataStore.particles(2,:),'.m')
legend('True trajectory','Estimated Trajectory')

% title('PF filter using 500 particles initial/final particle set and true trajectory')
% load('owen_particle_5000.mat')
% plot(dataStore.PFt(:,1),dataStore.PFt(:,2),'-m+')

% 
% 
% legend('True Trajectory','"Best" Particle Trajectory 50 particles','"Best" Particle Trajectory 5000 particles')
% title('Offline Particle Filter Estimation')
title('Particle Filter—Comparison between trajectories')
xlabel('x/m')
ylabel('y/m')