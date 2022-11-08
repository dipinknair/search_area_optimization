%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%
function out = SensorNetwork_Control(in,P)
%% Inertial States of Surveillance Vehicles
    % Vehicle - 0 true states
    NN              = 0;
    x(1,1)          = in(NN+1);
    y(1,1)          = in(NN+2);
    psi(1,1)        = in(NN+3);
    % Vehicle - 1 true states
    NN              = NN+3;
    x(2,1)          = in(NN+1);
    y(2,1)          = in(NN+2);
    psi(2,1)  = in(NN+3);
    % Vehicle - 2 true states
    NN              = NN+3;
    x(3,1)          = in(NN+1);
    y(3,1)          = in(NN+2);
    psi(3,1)        = in(NN+3);
    % Vehicle - 3 true states
    NN              = NN+3;
    x(4,1)          = in(NN+1);
    y(4,1)          = in(NN+2);
    psi(4,1)        = in(NN+3);
    % Vehicle - 4 true states
    NN              = NN+3;
    x(5,1)          = in(NN+1);
    y(5,1)          = in(NN+2);
    psi(5,1)        = in(NN+3);
    % Vehicle - 5 true states
    NN              = NN+3;
    x(6,1)          = in(NN+1);
    y(6,1)          = in(NN+2);
    psi(6,1)        = in(NN+3);
%% Control Inputs of Surveillance Vehicles
    % Vehicle - 0
    NN              = NN+4;
    v(1,1)          = in(NN+1);
    w(1,1)          = in(NN+2);
    % Vehicle - 1
    NN              = NN+2;
    v(2,1)          = in(NN+1);
    w(2,1)          = in(NN+2);
    % Vehicle - 2
    NN              = NN+2;
    v(3,1)          = in(NN+1);
    w(3,1)          = in(NN+2);
    % Vehicle - 3
    NN              = NN+2;
    v(4,1)          = in(NN+1);
    w(4,1)          = in(NN+2);
    % Vehicle - 4
    NN              = NN+2;
    v(5,1)          = in(NN+1);
    w(5,1)          = in(NN+2);
    % Vehicle - 5
    NN              = NN+2;
    v(6,1)          = in(NN+1);
    w(6,1)          = in(NN+2);
    
%% Inertial States of Sensor Network Vehicles
    NN              = NN+2;
    sn_x(1,1)       = in(NN+1);
    sn_y(1,1)       = in(NN+2);
    sn_psi(1,1)     = in(NN+3);
    % Vehicle - 1 true states
    NN              = NN+3;
    sn_x(2,1)       = in(NN+1);
    sn_y(2,1)       = in(NN+2);
    sn_psi(2,1)     = in(NN+3);
    % Vehicle - 2 true states
    NN              = NN+3;
    sn_x(3,1)       = in(NN+1);
    sn_y(3,1)       = in(NN+2);
    sn_psi(3,1)     = in(NN+3);
    % Vehicle - 3 true states
    NN              = NN+3;
    sn_x(4,1)       = in(NN+1);
    sn_y(4,1)       = in(NN+2);
    sn_psi(4,1)     = in(NN+3);
%%
    NN              = NN+3;
    t               = in(NN+1);
%% 
    persistent  senVeh count v0 w0 v1 w1 v2 w2 v3 w3
    
%% Model Predictive Control Architechture
    if rem(t,P.PredictionHorizon) == 0
%% Predict States
        for i = 1:1:P.NumVeh
            surVeh(i).states = PredictStates([x(i);y(i);psi(i)],[v(i);w(i)],P);
            
        end
%         for i = 1:1:P.NumSenVeh
%             senVeh(i).states = PredictStates([sn_x(i);sn_y(i);sn_psi(i)],[1;0.1],P);
%         end
%         
%         J = costFunction(surVeh,senVeh,P);
%% Initialize Solutions
        for i = 1:1:P.NumSenVeh
            solSet(i).omg = [];
        end
        for j = 1:1:P.NumGen
            for i = 1:1:P.NumSenVeh
                out = InitializeControls(P);
                solSet(j).omg = [solSet(j).omg;out];
            end
        end
%% Main Loop
        for i = 1:1:P.MaxIter
            for j = 1:1:P.NumSenVeh
                
                [new_solSet(j).omg,new_solSet(j).cost] = GeneticOperations(solSet(j),surVeh,senVeh,P);
                
            end
        end
                
    else
        if rem(t,P.Ts) == 0
            count = count+1;
            v0 = senVeh(1).vel(count);
            w0 = senVeh(1).ang.omg(count);
            v1 = senVeh(2).vel(count);
            w1 = senVeh(2).ang.omg(count);
            v2 = senVeh(3).vel(count);
            w2 = senVeh(3).ang.omg(count);
            v3 = senVeh(4).vel(count);
            w3 = senVeh(4).ang.omg(count);
        end
    end
    
    out = [v0;w0;v1;w1;v2;w2;v3;w3];
end

%% Prediction Function
function xHat = PredictStates(states,controls,P)
    px0    = states(1);
    py0    = states(2);
    psi0   = states(3);
    
    vel    = controls(1);
    omg    = controls(2);
    
    v      = vel*ones(P.NumSteps,1);
    w      = omg*ones(P.NumSteps,1);
    
    xHat   = zeros(P.NumStates*P.NumSteps,1);
    
    px     = zeros(P.NumSteps,1);
    py     = zeros(P.NumSteps,1);
    psi    = zeros(P.NumSteps,1);
    
    px(1)  = px0;
    py(1)  = py0;
    psi(1) = psi0;
    
    for i = 2:1:P.NumSteps
        px(i)  = px(i-1) + (v(i-1)*cos(psi(i-1)))*P.Ts;
        py(i)  = py(i-1) + (v(i-1)*sin(psi(i-1)))*P.Ts;
        psi(i) = psi(i-1) + w(i-1)*P.Ts;
    end
    
    for i = 1:1:P.NumSteps
        xHat(3*i-2) = px(i);
        xHat(3*i-1) = py(i);
        xHat(3*i-0) = psi(i);
    end
        
end

%% State Propagation
function xHat = PropagateStates(states,controls,P)
    px0    = states(1);
    py0    = states(2);
    psi0   = states(3);
    
    v      = controls(1,:)';
    w      = controls(2,:)';
    
    px     = zeros(P.NumSteps,1);
    py     = zeros(P.NumSteps,1);
    psi    = zeros(P.NumSteps,1);
    
    px(1)  = px0;
    py(1)  = py0;
    psi(1) = psi0;
    
    for i = 2:1:P.NumSteps
        px(i)  = px(i-1) + (v(i-1)*cos(psi(i-1)))*P.Ts;
        py(i)  = py(i-1) + (v(i-1)*sin(psi(i-1)))*P.Ts;
        psi(i) = psi(i-1) + w(i-1)*P.Ts;
    end
    
    for i = 1:1:P.NumSteps
        xHat(3*i-2) = px(i);
        xHat(3*i-1) = py(i);
        xHat(3*i-0) = psi(i);
    end
end

%% Initialize Population
function out = InitializeControls(P)

    a = -1;
    b = 1;
    
    %out.alpha = a + (b-a)*rand(1,P.NumSteps);
    out = a + (b-a)*rand(1,P.NumSteps);
%     out.omg(1,1) = omgPrev;
%     for i = 2:1:P.NumSteps
%         %out.omg(i,1) = out.omg(i-1,1) + out.alpha(i-1)*P.Ts;
%         out.omg(i,1) = 
%     end
    
        
end
%% Cost Function

function J = costFunction(surVeh,senVeh,P)
    DegreeMatrix   = zeros(P.NumVeh+P.NumSenVeh+P.NumLm);
    AdjMatrix      = zeros(P.NumVeh+P.NumSenVeh+P.NumLm);
    %GraphLaplacian = zeros(P.NumVeh+P.NumSenVeh+P.NumLm);
    J = 0;
    for i = 1:1:P.NumSteps
        netStruct.xPos = [];
        netStruct.yPos = [];
        for j = 1:1:P.NumVeh
            netStruct.xPos = [netStruct.xPos;surVeh(j).states(1)];
            netStruct.yPos = [netStruct.yPos;surVeh(j).states(2)];
        end
        for j = 1:1:P.NumSenVeh
            netStruct.xPos = [netStruct.xPos;senVeh(j).states(1)];
            netStruct.yPos = [netStruct.yPos;senVeh(j).states(2)];
        end
        for j = 1:1:P.NumLm
            netStruct.xPos = [netStruct.xPos;P.LmX(j)];
            netStruct.yPos = [netStruct.yPos;P.LmY(j)];
        end
        
        for j = 1:1:P.NumVeh+P.NumSenVeh+P.NumLm
            for k = 1:P.NumVeh+P.NumSenVeh+P.NumLm
                
                rho = sqrt((netStruct.xPos(j)-netStruct.xPos(k))^2+(netStruct.yPos(j)-netStruct.yPos(k))^2);
                
                if (rho <= P.SensingRange && rho ~= 0)
                    AdjMatrix(j,k) = 1;
                end
            end
        end
        
        for j = 1:1:P.NumVeh+P.NumSenVeh+P.NumLm
            DegreeMatrix(j,j) = sum(AdjMatrix(j,:));
        end
        
        GraphLaplacian = DegreeMatrix-AdjMatrix;
        
        J = J + max(mink(real(eig(GraphLaplacian)),2));
    end
end

%% Crossover Function
function out = CrossoverOperation(solSet,P)
    for i = 1:1:P.NumVeh
        numSol = numel(solSet(i).omg(:,1));
        selSol_Ids = randperm(numSol,numSol-rem(numSol,4));
        selSol_set = solSet(i).omg(selSol_Ids,:);
        
        sinPSol_Ids = randperm(numel(selSol_Ids),selSol_Ids/2);
        sinPSol_set = selSol_set(sinPSol_Ids,:);
        twoPSol_Ids = setdiff(selSol_Ids,sinPSol_Ids);
        twoPSol_set = selSol_set(twoPSol_Ids,:);
        
        
    end
        
end