%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Complex Systems and Networks: Project Main file Code 
% by Rohith Boyinine and Dipin Nair
% last modified by Rohith Boyinine on 04/27/2021
% runs genetic algorithm for map 1
clc; clear all; close all;

%% Vehicle Stats
P.NumVeh = 6;       % No. of Surveillance Vehicles
P.NumSenVeh = 4;    % No of vehicles in Sensor Network

%% PSO and ACO calls
PSO_alg = false;   % set to true to run PSO for map 1
ACO_alg = false;   % set to true to run ACO
MapNum = 1; %1,2,3 % map Number

%P.LmX = [25;30;65;75];
%P.LmY = [20;90;15;75];
P.LmX = [0; 90];   % Landmarks in the environment
P.LmY = [20; 40];  %  
P.NumLm = numel(P.LmX);

%% Simulation Parameters
P.Sim_Time = 300;

%% Estimation Parameters

P.SigmaRange = 0.1;
P.SigmaVel = 0.1;
P.SigmaOmg = 0.01;

P.SensingRange = 30;

%% Environment Parameters
if MapNum == 1
    P.map = createMap1;
else
    if MapNum == 2
        P.map = createMap2;
    else
        P.map = createMap3;
    end
end

%% Particle Swarm Initialization
W = warning('off','all'); 
id = W.identifier; 
warning('off',id)

if PSO_alg == true
%% Area Properties
    P.SearchRadUAV = sqrt(250/pi);
    P.SearchAreaUAV = pi*P.SearchRadUAV^2;

    P.NumWP = ceil(P.map.SearchPolygonArea/P.SearchAreaUAV);
%% PSO Initialization
    P.posMinX = min(P.map.SearchSpacePolygon.Vertices(:,1));
    P.posMaxX = max(P.map.SearchSpacePolygon.Vertices(:,1));
    P.posMinY = min(P.map.SearchSpacePolygon.Vertices(:,2));
    P.posMaxY = max(P.map.SearchSpacePolygon.Vertices(:,2));

    P.kVelX = 0.09;
    P.kVelY = 0.09;

    P.velMaxX = P.kVelX*(P.posMaxX-P.posMinX);
    P.velMinX = -P.velMaxX;
    P.velMaxY = P.kVelY*(P.posMaxY-P.posMinY);
    P.velMinY = -P.velMaxY;
    
    particles.xPos = zeros(P.NumWP,1);
    particles.yPos = zeros(P.NumWP,1);
    particles.xVel = zeros(P.NumWP,1);
    particles.yVel = zeros(P.NumWP,1);
    particles.Best.Cost = [];
    particles.Best.xPos = [];
    particles.Best.yPos = [];
    particles.Cost = [];


    P.NumGen = 100;
    P.maxIter = 100;

    P.GlobalBest = inf;
    % Constriction coefficients
    kappa = 1;
    phi1 = 2.05;
    phi2 = 2.05;
    phi = phi1+phi2;
    chi = 2*kappa/abs(2-phi-sqrt(phi^2-4*phi));

    P.w = chi;%0.01;%chi;
    P.wDamp = 1.0;
    P.c1 = chi*phi1;
    P.c2 = chi*phi2;


    P.OptimalityCriteria = 2;
    P.lam.Env = 40;
    P.lam.Coverage = 100;
    P.lam.Overlap = 100;

    ans1 = PSOAlgorithm(particles,P);
    
    TotWP = [ans1.BestSoln.xPos ans1.BestSoln.yPos];
    if MapNum == 1
        save('PSO_sol_map1.mat','ans1');
    else
        if MapNum == 2
            save('PSO_sol_map2.mat','ans1');
        else
            save('PSO_sol_map3.mat','ans1');
        end
    end
else
    
    if MapNum == 1
        load PSO_sol_map1.mat;
    else
        if MapNum == 2
            load PSO_sol_map2.mat;
        else
            load PSO_sol_map3.mat;
        end
    end
    
    TotWP = [ans1.BestSoln.xPos ans1.BestSoln.yPos];
end

P.TotWP = TotWP;

if ACO_alg == true
    
    wpNum = waypointdistribution(P);
    
    for i = 1:1:P.NumVeh
        WPlist = [P.TotWP(wpNum(i).veh,1) P.TotWP(wpNum(i).veh,2)];
        ACO(i) = TSP_AntSystem(WPlist);
    end
    
    if MapNum == 1
        save('ACO_sol_map1.mat','ans1');
    else
        if MapNum == 2
            save('ACO_sol_map2.mat','ans1');
        else
            save('ACO_sol_map3.mat','ans1');
        end
    end
    
else
    
    if MapNum == 1
        load ACO_sol_map1.mat;
    else
        if MapNum == 2
            load ACO_sol_map2.mat;
        else
            load ACO_sol_map3.mat;
        end
    end
    
    for i = 1:1:P.NumVeh
        P.WP(i).WP = ACO(i).WPorder;
    end
    
end


%% Initial conditions for Surveillance Vehicles

P.x0 = [P.WP(1).WP(1,1)+5; P.WP(1).WP(1,2)+5; atan2(-5,-5)];
P.x1 = [P.WP(2).WP(1,1)+5; P.WP(2).WP(1,2)+5; atan2(-5,-5)];
P.x2 = [P.WP(3).WP(1,1)+5; P.WP(3).WP(1,2)+5; atan2(-5,-5)];
P.x3 = [P.WP(4).WP(1,1)+5; P.WP(4).WP(1,2)+5; atan2(-5,-5)];
P.x4 = [P.WP(5).WP(1,1)+5; P.WP(5).WP(1,2)+5; atan2(-5,-5)];
P.x5 = [P.WP(6).WP(1,1)+5; P.WP(6).WP(1,2)+5; atan2(-5,-5)];

%% Initial COnditions for Sensor Network Vehicles

P.sn_x0 = [5; 5; pi/2];
P.sn_x1 = [60; 90; 3*pi/2];
P.sn_x2 = [45; 84; pi];
P.sn_x3 = [45; 40-12; 0];

%% Genetic Algorithm Parameters
P.Ts = 0.1;
P.PredictionHorizon = 2;
P.t = 0:P.Ts:P.PredictionHorizon;
P.NumSteps = numel(P.t);
P.NumGen = 100;
P.NumStates = 3;
P.MaxIter = 100;

%% 
P.NumInitEqConstraints = 3;
P.NumLinIneqConstraints = 4;
P.NumInputs = 2;

P.vMax = 5;
P.vMin = 0;
P.wMax = 1;
P.wMin = -1;

%% Waypoints for Sensor Network
P.sn_WP(1).WP = [...
                    15   5;...
                    15  90;...
                    60 100;...
                    80  40;...
                    60  10;...
                ];
P.sn_WP(2).WP = [...
                    60 100;...
                    80  40;...
                    60  10;...
                    15   5;...
                    15  90;...
                ];
P.sn_WP(3).WP = [...
                    15  90;...
                    60 100;...
                    80  40;...
                    60  10;...
                    15   5;...
                ];
    
P.sn_WP(4).WP = [...
                    80  40;...
                    60  10;...
                    15   5;...
                    15  90;...
                    60 100;...
                ];
%%
