%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%

function y  = GeneralSensor(in,P)
%% IMU data of Surveillance Vehicles
    % Vehicle - 0
    NN      = 0;
    v0      = in(NN+1) + P.SigmaVel*randn(1,1);
    w0      = in(NN+2) + P.SigmaOmg*randn(1,1);
    % Vehicle - 1
    NN      = NN+2;
    v1      = in(NN+1) + P.SigmaVel*randn(1,1);
    w1      = in(NN+2) + P.SigmaOmg*randn(1,1);
    % Vehicle - 2
    NN      = NN+2;
    v2      = in(NN+1) + P.SigmaVel*randn(1,1);
    w2      = in(NN+2) + P.SigmaOmg*randn(1,1);
    % Vehicle - 3
    NN      = NN+2;
    v3      = in(NN+1) + P.SigmaVel*randn(1,1);
    w3      = in(NN+2) + P.SigmaOmg*randn(1,1);
    % Vehicle - 4
    NN      = NN+2;
    v4      = in(NN+1) + P.SigmaVel*randn(1,1);
    w4      = in(NN+2) + P.SigmaOmg*randn(1,1);
    % Vehicle - 5
    NN      = NN+2;
    v5      = in(NN+1) + P.SigmaVel*randn(1,1);
    w5      = in(NN+2) + P.SigmaOmg*randn(1,1);
    
%% IMU data of Sensor Network Vehicles
    % Vehicle - 0
    NN      = NN+2;
    sn_v0   = in(NN+1) + P.SigmaVel*randn(1,1);
    sn_w0   = in(NN+2) + P.SigmaOmg*randn(1,1);
    % Vehicle - 1
    NN      = NN+2;
    sn_v1   = in(NN+1) + P.SigmaVel*randn(1,1);
    sn_w1   = in(NN+2) + P.SigmaOmg*randn(1,1);
    % Vehicle - 2
    NN      = NN+2;
    sn_v2   = in(NN+1) + P.SigmaVel*randn(1,1);
    sn_w2   = in(NN+2) + P.SigmaOmg*randn(1,1);
    % Vehicle - 3
    NN      = NN+2;
    sn_v3   = in(NN+1) + P.SigmaVel*randn(1,1);
    sn_w3   = in(NN+2) + P.SigmaOmg*randn(1,1);
    
%%  True States of Surveillance Vehicles
    % Vehicle - 0 true states
    NN      = NN+2;
    x(1)    = in(NN+1);
    y(1)    = in(NN+2);
    psi(1)  = in(NN+3);
    % Vehicle - 1 true states
    NN      = NN+3;
    x(2)    = in(NN+1);
    y(2)    = in(NN+2);
    psi(2)  = in(NN+3);
    % Vehicle - 2 true states
    NN      = NN+3;
    x(3)    = in(NN+1);
    y(3)    = in(NN+2);
    psi(3)  = in(NN+3);
    % Vehicle - 3 true states
    NN      = NN+3;
    x(4)    = in(NN+1);
    y(4)    = in(NN+2);
    psi(4)  = in(NN+3);
    % Vehicle - 4 true states
    NN      = NN+3;
    x(5)    = in(NN+1);
    y(5)    = in(NN+2);
    psi(5)  = in(NN+3);
    % Vehicle - 5 true states
    NN      = NN+3;
    x(6)    = in(NN+1);
    y(6)    = in(NN+2);
    psi(6)  = in(NN+3);
    
%%  True States of Sensor Network Vehicles
    % Vehicle - 0 true states
    NN      = NN+4;
    sn_x(1) = in(NN+1);
    sn_y(1) = in(NN+2);
    sn_psi0 = in(NN+3);
    % Vehicle - 1 true states
    NN      = NN+3;
    sn_x(2) = in(NN+1);
    sn_y(2) = in(NN+2);
    sn_psi1 = in(NN+3);
    % Vehicle - 2 true states
    NN      = NN+3;
    sn_x(3) = in(NN+1);
    sn_y(3) = in(NN+2);
    sn_psi2 = in(NN+3);
    % Vehicle - 3 true states
    NN      = NN+3;
    sn_x(4) = in(NN+1);
    sn_y(4) = in(NN+2);
    sn_psi3 = in(NN+3);
    
%% Time   
    
    NN      = NN+3;
    t       = in(NN+1);
    
%% Range Measurements between Surveillance Vehicles

    rho_veh = -999*ones(P.NumVeh);
    
    for i = 1:1:P.NumVeh
        for j = i+1:1:P.NumVeh
            
            rho = sqrt((x(i)-x(j))^2+(y(i)-y(j))^2) + P.SigmaRange*randn(1,1);
            
            if rho <= P.SensingRange
                rho_veh(i,j) = rho;
            end
        end
    end

%% Range Measurements between Sensing Vehicles

    rho_sen_veh = -999*ones(P.NumSenVeh);
    
    for i = 1:1:P.NumSenVeh
        for j = i+1:1:P.NumSenVeh
            
            rho = sqrt((sn_x(i)-sn_x(j))^2+(sn_y(i)-sn_y(j))^2) + P.SigmaRange*randn(1,1);
            
            if rho <= P.SensingRange
                rho_sen_veh(i,j) = rho;
            end
        end
    end
    
%% Range Measurements between Surveillance and Sensing Vehicles
    
    rho_sen_sur_veh = -999*ones(P.NumVeh,P.NumSenVeh);
    
    for i = 1:1:P.NumVeh
        for j = i+1:1:P.NumSenVeh
            
            rho = sqrt((x(i)-sn_x(j))^2 + (y(i)-sn_y(j))^2) + P.SigmaRange*randn(1,1);
            
            if rho <= P.SensingRange
                rho_sen_sur_veh(i,j) = rho;
            end
        end
    end
    
    
%% Range Measurements of Surveillance Vehicles from Landmarks
     
    rho_lm = -999*ones(P.NumVeh,P.NumLm);
    
    for i = 1:1:P.NumVeh
        for j = 1:1:P.NumLm
            
            rho = sqrt((x(i)-P.LmX(j))^2 + (y(i)-P.LmY(j))^2) + P.SigmaRange*randn(1,1);
            
            if rho <= P.SensingRange
                rho_lm(i,j) = rho;
            end
        end
    end
    
%% Range Measurements of Sensor Vehicles from Landmarks
     
    rho_sen_lm = -999*ones(P.NumSenVeh,P.NumLm);
    
    for i = 1:1:P.NumSenVeh
        for j = 1:1:P.NumLm
            
            rho = sqrt((sn_x(i)-P.LmX(j))^2 + (sn_y(i)-P.LmY(j))^2) + P.SigmaRange*randn(1,1);
            
            if rho <= P.SensingRange
                rho_sen_lm(i,j) = rho;
            end
        end
    end
    
%% Output

    y     = [...
                                           v0;...
                                           w0;...
                                           v1;...
                                           w1;...
                                           v2;...
                                           w2;...
                                           v3;...
                                           w3;...
                                           v4;...
                                           w4;...
                                           v5;...
                                           w5;...
                                        sn_v0;...
                                        sn_w0;...
                                        sn_v1;...
                                        sn_w1;...
                                        sn_v2;...
                                        sn_w2;...
                                        sn_v3;...
                                        sn_w3;...
                        reshape(rho_veh,[],1);...
                    reshape(rho_sen_veh,[],1);...
                reshape(rho_sen_sur_veh,[],1);...
                         reshape(rho_lm,[],1);...
                     reshape(rho_sen_lm,[],1);...
                                            t;...
            ];
end