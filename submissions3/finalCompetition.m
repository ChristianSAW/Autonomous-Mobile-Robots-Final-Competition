function[dataStore] = finalCompetition(CreatePort,SonarPort,BeaconPort,tagNum,maxTime)
% backupBump: simple example program to use with iRobot Create (or simulator).
% Reads data from sensors, makes the robot hit, backup, move forward, and saves a datalog.
%
%   dataStore = TURNINPLACE(CreatePort,SonarPort,BeaconPort,tagNum,maxTime) runs
%
%   INPUTS
%       CreatePort  Create port object (get from running RoombaInit)
%       SonarPort   Sonar port object (get from running RoombaInit)
%       BeaconPort  Camera port object (get from running RoombaInit)
%       tagNum      robot number for overhead localization
%       maxTime     max time to run program (in seconds)
%
%   OUTPUTS
%       dataStore   struct containing logged data

%
%   NOTE: Assume differential-drive robot whose wheels turn at a constant
%         rate between sensor readings.
%
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots
% Owen Hua
% Christian Welling
% 
% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    SonarPort = CreatePort;
    BeaconPort = CreatePort;
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 3
    BeaconPort = CreatePort;
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 4
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 5
    maxTime = 500;
end


% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped

global dataStore
% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
    'odometry', [], ...
    'lidar', [], ...
    'sonar', [], ...
    'bump', [], ...
    'beacon', []);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;
% Define used variables
% load map
load('compMap.mat');
% load('ExampleMap3.mat');
i = 1;
ConfidenceArray = zeros(size(optWalls,1),1);
firstbeacon = 0;
maxTime = 60*10;
switchTime = 12;
robotRad = 0.17;
robotRadPrime = 0.2;
threshold = 0.2;
beaconCount = 0;
beaconEvoke = 0;
convergeFlag = 0;
closeEnough = 0.13;
% closeEnoughGoal = 0.1;
BumpCount = 0;
ViewCount = 1;
m = 2;%wayPointsCount
VisitedW = [];
VisitedECW = [];
% map = cornerMap;
xverts = [map(:,1),map(:,3)];
yverts = [map(:,2),map(:,4)];
xmax = max(xverts(:));
xmin = min(xverts(:));
ymax = max(yverts(:));
ymin = min(yverts(:));
figure;
xlabel('x[m]')
ylabel('y[m]')
title('Global Map')
hold on
plotmap(map)
tic
while toc < maxTime
    %%%%%%%%%%%%%%%%%%%%%%%%Step 1: Read Sensor Data%%%%%%%%%%%%%%%%%%%%%%%
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,SonarPort,BeaconPort,tagNum,noRobotCount,dataStore);
    %%%%%%%%%%%%%%%%%%%%%%%%Step 2: Find Robot initial Position%%%%%%%%%%%%
    Time = toc;
    % Use PF to find the robot initial position, and use other methods
    % after the particles converge
    if Time < switchTime
        % Particle Filter Initialization
        if i == 1
            num_p = 200;
            % %         Use truthPose to sample particles
            %                 xi = ones(1,num_p)*dataStore.truthPose(i,2);
            %                 yi = ones(1,num_p)*dataStore.truthPose(i,3);
            %                 thetai = ones(1,num_p)*dataStore.truthPose(i,4);
            %                 wt = rand(1,num_p);
            % Sample Particles at the waypoints
            xi = [];yi = [];thetai = [];
            for p = 1:size(waypoints,1)
                xp = ones(1,num_p)*waypoints(p,1);
                yp = ones(1,num_p)*waypoints(p,2);
                thetap = linspace(0,2*pi,num_p);
                xi = [xi xp];
                yi = [yi yp];
                thetai = [thetai thetap];
            end
            wt = rand(1,num_p*size(waypoints,1));
            PSet0 =[xi;yi;thetai;wt];
            dataStore.initialParticles = PSet0;
        end
        % Use Particle Filter to find the initial position of robot
        if beaconCount < size(dataStore.beacon,1)
            beaconCount = size(dataStore.beacon,1);
            beaconEvoke = 1;
        end
        [dataStore.particles, beaconEvoke] = particleFilter(PSet0,dataStore.sonar(end,2:4)',...
            [dataStore.odometry(end,2),dataStore.odometry(end,3)],...
            @integrateOdom,@sonarPredict,map,[xmax xmin ymax ymin],...
            switchTime,dataStore.beacon,beaconLoc,beaconEvoke,CreatePort);
        %Robot Localization Result if using the mean of all particles in PF
        robotPose(i,1) = mean(dataStore.particles(1,:));
        robotPose(i,2) = mean(dataStore.particles(2,:));
%         robotPose(i,3) = mean(dataStore.particles(3,:));
        % Special Treatment to angular pose
        [~,ind] = max(dataStore.particles(4,:));
        robotPose(i,3) = dataStore.particles(3,ind);
        Pset = dataStore.particles;
%         % Plot the particle set
%         h = plot(dataStore.particles(1,:),dataStore.particles(2,:),'bo');
%         pause(0.01)
%         set(h,'Visible','off')
        % Control for initialization
        wheel2Center = 0.13;
        % Turn in place to initialize
        maxV = 0.2;
        cmdV = 0;
        cmdW = 0.2;
        FWDVEL = cmdV;
        ANGVEL = cmdW;
        [cmdV,cmdW] = limitCmds(FWDVEL,ANGVEL,maxV,wheel2Center);
        % Turn and move around a small circle
        %         if (cIa < cIaM) && (cIb >= cIbM)
        %             maxV = 0.2;
        %             cmdV = 0.03;
        %             cmdW = 0.2;
        %             FWDVEL = cmdV;
        %             ANGVEL = cmdW;
        %             [cmdV,cmdW] = limitCmds(FWDVEL,ANGVEL,maxV,wheel2Center);
        %             cIa = cIa + 1;
        %             if cIa == cIaM
        %                 cIb = 1;
        %             end
        %         elseif (cIb < cIbM) && (cIa >= cIaM)
        %             maxV = 0.2;
        %             cmdV = 0;
        %             cmdW = 0.2;
        %             FWDVEL = cmdV;
        %             ANGVEL = cmdW;
        %             [cmdV,cmdW] = limitCmds(FWDVEL,ANGVEL,maxV,wheel2Center);
        %             cIb = cIb + 1;
        %             if cIb == cIbM
        %                 cIa = 1;
        %             end
        %         end
    else % Robot Initial position is converged
        
        if convergeFlag == 0
            % PathPlanning after converging
            SetFwdVelAngVelCreate(CreatePort, 0,0 );
            try
                Paths = globalPathFindingGrid(map,waypoints,ECwaypoints,robotPose(end,1:2),robotRadPrime);
            catch
                Paths = globalPathFindingPRM(map,waypoints,ECwaypoints,robotPose(end,1:2),robotRadPrime);    
            end
            
            PG = plot(Paths(:,1),Paths(:,2),'hk-');
            convergeFlag = NaN;
        end
        %%%%%%%%%%%%Step 3: Use Dead Reckoning to do localization based on the
        %result from previous localization data%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [robotDRL,~] = integrateOdom(robotPose(end,:), dataStore.odometry(end,2), dataStore.odometry(end,3));
        robotDRL = robotDRL'; % [3X1] to [1x2]
        %%%%%%%%%%%%Step 4: Use PF to do localization%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if beaconCount < size(dataStore.beacon,1)
            beaconCount = size(dataStore.beacon,1);
            beaconEvoke = 1;
        end
        [dataStore.particles, beaconEvoke] = particleFilter(PSet0,dataStore.sonar(end,2:4)',...
            [dataStore.odometry(end,2),dataStore.odometry(end,3)],...
            @integrateOdom,@sonarPredict,map,[xmax xmin ymax ymin],...
            switchTime,dataStore.beacon,beaconLoc,beaconEvoke,CreatePort);
        %Robot Localization Result if using the mean of all particles in PF
        robotPFL(1,1) = mean(dataStore.particles(1,:));
        robotPFL(1,2) = mean(dataStore.particles(2,:));
%         robotPFL(1,3) = mean(dataStore.particles(3,:));
        %Special case for angular pose
        [~,ind] = max(dataStore.particles(4,:));
        robotPFL(1,3) = dataStore.particles(3,ind);
        Pset = dataStore.particles;
        % Plot the particle set
%         h = plot(PSet0(1,:),PSet0(2,:),'yo');
%         pause(0.01)
%         set(h,'Visible','off')
        %%%%%%%%%%%%Step 5: Check Optional Wall%%%%%%%%%%%%%%%
        smlegh = rbtPoseCompre(robotDRL(1:2),robotPFL(1:2),threshold);
        robotPose(i,:) = robotPFL;
        %         SetFwdVelAngVelCreate(CreatePort, 0,0 );
        [optWalls, map, UpdateMapFlag,ConfidenceArray] = CheckOptWalls(robotPose(end,:), map, optWalls, dataStore.sonar(end,2:4),ConfidenceArray,CreatePort);
        if UpdateMapFlag == 1;
            
            set(PG,'Visible','off')
            plotmap(map)
            % Path Planning once the map is updated
            SetFwdVelAngVelCreate(CreatePort, 0,0 );
            try
                Paths = globalPathFindingGrid(map,waypoints,ECwaypoints,robotPose(end,1:2),robotRadPrime);
            catch
                Paths = globalPathFindingPRM(map,waypoints,ECwaypoints,robotPose(end,1:2),robotRadPrime);    
            end
            
            PG = plot(Paths(:,1),Paths(:,2),'hk-');
        end
%         %%%%%%%%%%%%%%%%Step 6: EKF beacon%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         if beaconCount < size(dataStore.beacon,1) && beaconEvoke == 1
%             %SetFwdVelAngVelCreate(CreatePort, 0,0 );
%             beaconCount = size(dataStore.beacon,1);
%             % Use the prediction from deadreckoning
%             mu0 = robotDRL;
%             %             sigma0 = [cov(PSet0(1,:)),0,0;
%             %                 0,cov(PSet0(2,:)),0,;
%             %                 0,0,cov(PSet0(3,:))];
%             sigma0(:,:,1) = [0.05,0,0;
%                 0,0.05,0;
%                 0,0,0.1];
%             zt = [dataStore.beacon(end,5),-dataStore.beacon(end,3)];
%             ut = [dataStore.odometry(end,2),dataStore.odometry(end,3)];
%             R = eye(3)*0.01;
%             Q = eye(3)*0.001;
%             g = @integrateOdom;
%             G = @GjacDiffDrive;
%             H = @HjacBeacon;
%             h = @hBeacon;
%             [mu,~] = extendedKalmanFilterBeacon(mu0,sigma0,zt,ut,Q,R,g,G,h,H,map,dataStore.beacon(end,2),beaconLoc);
%             [Pset] = PsetGenerator(mu, num_p*size(waypoints,1), [0.5 0.05 0.1]);
%             robotPose(i,:) = mu;
%             %h = plot(Pset(1,:),Pset(2,:),'yo');
%             %pause(0.0001)
%             %set(h,'Visible','off')
%         end
%         % Evoke beacon localization for next round
%         if beaconEvoke == 0
%             beaconCount = size(dataStore.beacon,1);
%             beaconEvoke = 1;
%         end
        %%%%%%%%%%%%%%%%Step 7: Path Planning and Map Info Updates%%%%%%%%%
        % CONTROL FUNCTION (send robot commands based on the result from path planning)
        %         SetFwdVelAngVelCreate(CreatePort, 0,0 );
        
        xb = robotPose(end,1);
        yb = robotPose(end,2);
        if UpdateMapFlag == 0
            distance = sqrt((Paths(m,1)-xb)^2+(Paths(m,2)-yb)^2);
            if distance <closeEnough && m == size(Paths,1)
                [VisitedW,VisitedECW,VisitedWFlag,VisitedECWFlag,~,~] = WaypointsCheck(robotPose(end,:),waypoints,ECwaypoints,closeEnough,VisitedW,VisitedECW);
                if VisitedWFlag == 1
                    BeepRoomba(CreatePort)
                    plot(VisitedW(end,1),VisitedW(end,2),'pk','LineWidth',3)
                end
                if VisitedECWFlag == 1
                    BeepRoomba(CreatePort)
                    BeepRoomba(CreatePort)
                    plot(VisitedECW(end,1),VisitedECW(end,2),'pr','LineWidth',3)
                end
                break
            end
            Z(ViewCount) = plot(Paths(m,1),Paths(m,2),'xm','MarkerSize',5,'LineWidth',3);
%             disp([Paths(m,1),Paths(m,2)])
            if distance < closeEnough
                m = m+1;
%                 set(Z(ViewCount),'Visible','off')
%                 ViewCount = ViewCount + 1;
            end
%             SetFwdVelAngVelCreate(CreatePort,0,0);
            Vx = Paths(m,1)-xb;
            Vy = Paths(m,2)-yb;
            %             disp(Vx)
            %             disp(Vy)
            % Set angular velocity
            epsilon = 0.15;
            theta = robotPose(end,3);
            [V,w] = feedbackLin(Vx,Vy,epsilon,theta);
            FWDVEL = V;
            ANGVEL = w;
            [cmdV,cmdW] = limitCmds(FWDVEL,ANGVEL,maxV,wheel2Center);
        else
            m = 2;
            distance = sqrt((Paths(m,1)-xb)^2+(Paths(m,2)-yb)^2);
            if distance <closeEnough && m == size(Paths,1)
                break
            end
            if distance < closeEnough
                m = m+1;
            end
            Vx = Paths(m,1)-xb;
            Vy = Paths(m,2)-yb;
            % Set angular velocity
            epsilon = 0.15;
            theta = robotPose(end,3);
            [V,w] = feedbackLin(Vx,Vy,epsilon,theta);
            FWDVEL = V;
            ANGVEL = w;
            [cmdV,cmdW] = limitCmds(FWDVEL,ANGVEL,maxV,wheel2Center);
        end
        % Check whether the waypoints and ECwaypoints are close enough to the robot
        [VisitedW,VisitedECW,VisitedWFlag,VisitedECWFlag,waypoints,ECwaypoints] = WaypointsCheck(robotPose(end,:),waypoints,ECwaypoints,closeEnough,VisitedW,VisitedECW);
        if VisitedWFlag == 1
            BeepRoomba(CreatePort)
            plot(VisitedW(end,1),VisitedW(end,2),'pk','LineWidth',3)
        end
        if VisitedECWFlag == 1
            BeepRoomba(CreatePort)
            BeepRoomba(CreatePort)
            plot(VisitedECW(end,1),VisitedECW(end,2),'pr','LineWidth',3)
        end
        Ga = plot(robotPose(end,1),robotPose(end,2),'co');
    end
    % Update Particle set for the next round
    PSet0 = Pset;
    % Plot the current prediction
%     Orietx = cos(robotPose(end,3))*robotRad+robotPose(end,1);
%     Oriety = sin(robotPose(end,3))*robotRad+robotPose(end,2);
%     Da = plot([robotPose(end,1),Orietx],[robotPose(end,2) Oriety],'-k','LineWidth',5');
      Da = quiver(robotPose(end,1),robotPose(end,2),cos(robotPose(end,3))*robotRad,sin(robotPose(end,3))*robotRad,'Color','r','AutoScaleFactor',1.5,'MaxHeadSize',0.5);
%     % Plot to compare the truthPose
%     plot(dataStore.truthPose(end,2),dataStore.truthPose(end,3),'og')
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    elseif dataStore.bump(end,2) == 1 || dataStore.bump(end,3) == 1 || dataStore.bump(end,7) == 1
        BumpCount = BumpCount + 1;
        travelDist(CreatePort,0.2,-0.25);
        speed = 0.2;
        if dataStore.bump(end,2) == 1
            Angle = 75;
        elseif dataStore.bump(end,3) == 1
            Angle = -75;
        else
            Angle = 50*sign(rand-0.5);
        end
        turnAngle(CreatePort, speed, Angle);
    else
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
    end
    pause(0.1);
    %     If the robot keep bumps to the wall, go back to the previous
    %     waypoints.
    if BumpCount >=3 && m > 1
        m = m-1;
        BumpCount = 0;
    end
    i = i + 1;
%     set(Ga,'Visible','off')
    set(Da,'Visible','off')
end
dataStore.robotPose = robotPose;
dataStore.map = map;
dataStore.VisitedW = VisitedW;
dataStore.VisitedECW = VisitedECW;
disp('Final Competition Done')
plot(robotPose(end,1),robotPose(end,2),'mo','MarkerSize',5,'LineWidth',5)
hold off
SetFwdVelAngVelCreate(CreatePort,0,0);
