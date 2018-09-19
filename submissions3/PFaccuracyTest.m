% Test the accuracy of the particle filter
% global dataStore
% figure
% xdiff = abs(dataStore.truthPose(:,2)-dataStore.robotPose(:,1));
% ydiff = abs(dataStore.truthPose(:,3)-dataStore.robotPose(:,2));
% Diff = xdiff+ydiff;
% plot(dataStore.truthPose(:,1),Diff,'.-')

global dataStore
figure
xdiff = abs(dataStore.truthPose(:,2)-dataStore.robotPose(:,1));
ydiff = abs(dataStore.truthPose(:,3)-dataStore.robotPose(:,2));
Diff = xdiff+ydiff;
plot(dataStore.truthPose(:,1),Diff,'.-r')

hold on 

xdiffDR = abs(dataStore.truthPose(:,2)-dataStore.robotDR(:,1));
ydiffDR = abs(dataStore.truthPose(:,3)-dataStore.robotDR(:,2));
DiffDR = xdiffDR+ydiffDR;
plot(dataStore.truthPose(:,1),DiffDR,'.-g')


xdiff2 = abs(dataStore2.truthPose(:,2)-dataStore2.robotPose(:,1));
ydiff2 = abs(dataStore2.truthPose(:,3)-dataStore2.robotPose(:,2));
Diff2 = xdiff2+ydiff2;
plot(dataStore2.truthPose(:,1),Diff2,'.-b')

title('Independent Random Weighted Resampling')
%title('Low Variance Resampling')
legend('Perfict Initialization Low Sig', 'DeadReckoning', 'Perfect Init Reg Sig')