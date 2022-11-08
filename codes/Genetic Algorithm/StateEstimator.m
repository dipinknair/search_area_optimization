%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%

function out = StateEstimator(in,P)
%% IMU of Surveillance Vehicles
    % Vehicle - 0
    NN = 0;
    v0 = in(NN+1);
    w0 = in(NN+2);
    % Vehicle - 1
    NN = NN+2;
    v1 = in(NN+1);
    w1 = in(NN+2);
    % Vehicle - 2
    NN = NN+2;
    v2 = in(NN+1);
    w2 = in(NN+2);
    % Vehicle - 3
    NN = NN+2;
    v3 = in(NN+1);
    w3 = in(NN+2);
    % Vehicle - 4
    NN = NN+2;
    v4 = in(NN+1);
    w4 = in(NN+2);
    NN = NN+2;
    v5 = in(NN+1);
    w5 = in(NN+2);

%% IMU of Sensor Network Vehicles
    % Vehicle - 0
    NN = NN+2;
    sn_v0 = in(NN+1);
    sn_w0 = in(NN+2);
    % Vehicle - 1
    NN = NN+2;
    sn_v1 = in(NN+1);
    sn_w1 = in(NN+2);
    % Vehicle - 2
    NN = NN+2;
    sn_v2 = in(NN+1);
    sn_w2 = in(NN+2);
    % Vehicle - 3
    NN = NN+2;
    sn_v3 = in(NN+1);
    sn_w3 = in(NN+2);
    
%% Range measurements between Surveillance Vehicles
    NN = NN+2;
    rho_veh = reshape(in(NN+1:NN+P.NumVeh^2),[P.NumVeh,P.NumVeh]);
    
%% Range Measurements between Sensor Network Vehicles
    NN = NN + P.NumVeh^2;
    rho_sen_veh = reshape(in(NN+1:NN+P.NumSenVeh^2),[P.NumSenVeh,P.NumSenVeh]);
    
%% Range Measurements between Sensor Network and Surveillance Vehicles
    NN = NN+P.NumSenVeh^2;
    rho_sen_sur_veh = reshape(in(NN+1:NN+P.NumVeh*P.NumSenVeh),[P.NumVeh,P.NumSenVeh]);
    
%% Range measurements of Surveillance Vehicles from Landmarks
    NN = NN+P.NumVeh*P.NumSenVeh;
    rho_lm = reshape(in(NN+1:NN+P.NumVeh*P.NumLm),[P.NumVeh,P.NumLm]);
    
%% Range measurements of Sensor Network Vehicles from Landmarks
    NN = NN+P.NumVeh*P.NumLm;
    rho_sen_lm = reshape(in(NN+1:NN+P.NumSenVeh*P.NumLm),[P.NumSenVeh,P.NumLm]);
    
%% time
    NN = NN+P.NumSenVeh*P.NumLm;
    t = in(NN+1);
    
%% Initialization
    qu = eye(2*(P.NumVeh+P.NumSenVeh));
    
    for i = 1:1:P.NumVeh+P.NumSenVeh
        qu(2*i-1,2*i-1) = P.SigmaVel^2;
        qu(2*i,2*i) = P.SigmaOmg^2;
    end
    
    persistent xHat Pt tPr
    if t == 0
        xHat = [P.x0;P.x1;P.x2;P.x3;P.x4;P.x5;P.sn_x0;P.sn_x1;P.sn_x2;P.sn_x3] + 0.1*randn(3*(P.NumVeh+P.NumSenVeh),1);
        Pt   = 1*eye(numel(xHat));
        for i = 1:1:P.NumVeh+P.NumSenVeh
            Pt(3*i,3*i) = 0.5;
        end
        tPr = t;
    else
%% Prediction
        Ts = t - tPr;
        step = 10;
        
        for i = 1:1:step
            NN      = 0;
            x0      = xHat(NN+1);
            y0      = xHat(NN+2);
            psi0    = xHat(NN+3);
            NN      = NN+3;
            x1      = xHat(NN+1);
            y1      = xHat(NN+2);
            psi1    = xHat(NN+3);
            NN      = NN+3;
            x2      = xHat(NN+1);
            y2      = xHat(NN+2);
            psi2    = xHat(NN+3);
            NN      = NN+3;
            x3      = xHat(NN+1);
            y3      = xHat(NN+2);
            psi3    = xHat(NN+3);
            NN      = NN+3;
            x4      = xHat(NN+1);
            y4      = xHat(NN+2);
            psi4    = xHat(NN+3);
            NN      = NN+3;
            x5      = xHat(NN+1);
            y5      = xHat(NN+2);
            psi5    = xHat(NN+3);
            NN      = NN+3;
            sn_x0   = xHat(NN+1);
            sn_y0   = xHat(NN+2);
            sn_psi0 = xHat(NN+3);
            NN      = NN+3;
            sn_x1   = xHat(NN+1);
            sn_y1   = xHat(NN+2);
            sn_psi1 = xHat(NN+3);
            NN      = NN+3;
            sn_x2   = xHat(NN+1);
            sn_y2   = xHat(NN+2);
            sn_psi2 = xHat(NN+3);
            NN      = NN+3;
            sn_x3   = xHat(NN+1);
            sn_y3   = xHat(NN+2);
            sn_psi3 = xHat(NN+3);
            
            x0dot   = v0*cos(psi0);
            y0dot   = v0*sin(psi0);
            psi0dot = w0;
            
            x1dot   = v1*cos(psi1);
            y1dot   = v1*sin(psi1);
            psi1dot = w1;
            
            x2dot   = v2*cos(psi2);
            y2dot   = v2*sin(psi2);
            psi2dot = w2;
            
            x3dot   = v3*cos(psi3);
            y3dot   = v3*sin(psi3);
            psi3dot = w3;
            
            x4dot   = v4*cos(psi4);
            y4dot   = v4*sin(psi4);
            psi4dot = w4;
            
            x5dot   = v5*cos(psi5);
            y5dot   = v5*sin(psi5);
            psi5dot = w5;
            
            sn_x0dot   = sn_v0*cos(sn_psi0);
            sn_y0dot   = sn_v0*sin(sn_psi0);
            sn_psi0dot = sn_w0;
            
            sn_x1dot   = sn_v1*cos(sn_psi1);
            sn_y1dot   = sn_v1*sin(sn_psi1);
            sn_psi1dot = sn_w1;
            
            sn_x2dot   = sn_v2*cos(sn_psi2);
            sn_y2dot   = sn_v2*sin(sn_psi2);
            sn_psi2dot = sn_w2;
            
            sn_x3dot   = sn_v3*cos(sn_psi3);
            sn_y3dot   = sn_v3*sin(sn_psi3);
            sn_psi3dot = sn_w3;
            
            ft = [x0dot;y0dot;psi0dot;x1dot;y1dot;psi1dot;x2dot;y2dot;psi2dot;x3dot;y3dot;psi3dot;x4dot;y4dot;psi4dot;x5dot;y5dot;psi5dot;sn_x0dot;sn_y0dot;sn_psi0dot;sn_x1dot;sn_y1dot;sn_psi1dot;sn_x2dot;sn_y2dot;sn_psi2dot;sn_x3dot;sn_y3dot;sn_psi3dot];
            
            xHat = xHat + (Ts/step)*ft;
            
            At = [...
                     0, 0, -v0*sin(psi0), 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,  v0*cos(psi0), 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0, -v1*sin(psi1), 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,  v1*cos(psi1), 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0, -v2*sin(psi2), 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,  v2*cos(psi2), 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0, -v3*sin(psi3), 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,  v3*cos(psi3), 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0, -v4*sin(psi4), 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,  v4*cos(psi4), 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0, -v5*sin(psi5), 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,  v5*cos(psi5), 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0, -sn_v0*sin(sn_psi0), 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,  sn_v0*cos(sn_psi0), 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0, -sn_v1*sin(sn_psi1), 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,  sn_v1*cos(sn_psi1), 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0, -sn_v2*sin(sn_psi2), 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,  sn_v2*cos(sn_psi2), 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0, -sn_v3*sin(sn_psi3);...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,  sn_v3*cos(sn_psi3);...
                     0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,             0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0, 0, 0,                   0;...
                 ];
             
             Bt = [...
                     cos(psi0), 0,         0, 0,         0, 0,         0, 0,         0, 0,         0, 0,            0, 0,            0, 0,            0, 0,            0, 0;...
                     sin(psi0), 0,         0, 0,         0, 0,         0, 0,         0, 0,         0, 0,            0, 0,            0, 0,            0, 0,            0, 0;...
                             0, 1,         0, 0,         0, 0,         0, 0,         0, 0,         0, 0,            0, 0,            0, 0,            0, 0,            0, 0;...
                             0, 0, cos(psi1), 0,         0, 0,         0, 0,         0, 0,         0, 0,            0, 0,            0, 0,            0, 0,            0, 0;...
                             0, 0, sin(psi1), 0,         0, 0,         0, 0,         0, 0,         0, 0,            0, 0,            0, 0,            0, 0,            0, 0;...
                             0, 0,         0, 1,         0, 0,         0, 0,         0, 0,         0, 0,            0, 0,            0, 0,            0, 0,            0, 0;...
                             0, 0,         0, 0, cos(psi2), 0,         0, 0,         0, 0,         0, 0,            0, 0,            0, 0,            0, 0,            0, 0;...
                             0, 0,         0, 0, sin(psi2), 0,         0, 0,         0, 0,         0, 0,            0, 0,            0, 0,            0, 0,            0, 0;...
                             0, 0,         0, 0,         0, 1,         0, 0,         0, 0,         0, 0,            0, 0,            0, 0,            0, 0,            0, 0;...
                             0, 0,         0, 0,         0, 0, cos(psi3), 0,         0, 0,         0, 0,            0, 0,            0, 0,            0, 0,            0, 0;...
                             0, 0,         0, 0,         0, 0, sin(psi3), 0,         0, 0,         0, 0,            0, 0,            0, 0,            0, 0,            0, 0;...
                             0, 0,         0, 0,         0, 0,         0, 1,         0, 0,         0, 0,            0, 0,            0, 0,            0, 0,            0, 0;...
                             0, 0,         0, 0,         0, 0,         0, 0, cos(psi4), 0,         0, 0,            0, 0,            0, 0,            0, 0,            0, 0;...
                             0, 0,         0, 0,         0, 0,         0, 0, sin(psi4), 0,         0, 0,            0, 0,            0, 0,            0, 0,            0, 0;...
                             0, 0,         0, 0,         0, 0,         0, 0,         0, 1,         0, 0,            0, 0,            0, 0,            0, 0,            0, 0;...
                             0, 0,         0, 0,         0, 0,         0, 0,         0, 0, cos(psi5), 0,            0, 0,            0, 0,            0, 0,            0, 0;...
                             0, 0,         0, 0,         0, 0,         0, 0,         0, 0, sin(psi5), 0,            0, 0,            0, 0,            0, 0,            0, 0;...
                             0, 0,         0, 0,         0, 0,         0, 0,         0, 0,         0, 1,            0, 0,            0, 0,            0, 0,            0, 0;...
                             0, 0,         0, 0,         0, 0,         0, 0,         0, 0,         0, 0, cos(sn_psi0), 0,            0, 0,            0, 0,            0, 0;...
                             0, 0,         0, 0,         0, 0,         0, 0,         0, 0,         0, 0, sin(sn_psi0), 0,            0, 0,            0, 0,            0, 0;...
                             0, 0,         0, 0,         0, 0,         0, 0,         0, 0,         0, 0,            0, 1,            0, 0,            0, 0,            0, 0;...
                             0, 0,         0, 0,         0, 0,         0, 0,         0, 0,         0, 0,            0, 0, cos(sn_psi1), 0,            0, 0,            0, 0;...
                             0, 0,         0, 0,         0, 0,         0, 0,         0, 0,         0, 0,            0, 0, sin(sn_psi1), 0,            0, 0,            0, 0;...
                             0, 0,         0, 0,         0, 0,         0, 0,         0, 0,         0, 0,            0, 0,            0, 1,            0, 0,            0, 0;...
                             0, 0,         0, 0,         0, 0,         0, 0,         0, 0,         0, 0,            0, 0,            0, 0, cos(sn_psi2), 0,            0, 0;...
                             0, 0,         0, 0,         0, 0,         0, 0,         0, 0,         0, 0,            0, 0,            0, 0, sin(sn_psi2), 0,            0, 0;...
                             0, 0,         0, 0,         0, 0,         0, 0,         0, 0,         0, 0,            0, 0,            0, 0,            0, 1,            0, 0;...
                             0, 0,         0, 0,         0, 0,         0, 0,         0, 0,         0, 0,            0, 0,            0, 0,            0, 0, cos(sn_psi3), 0;...
                             0, 0,         0, 0,         0, 0,         0, 0,         0, 0,         0, 0,            0, 0,            0, 0,            0, 0, sin(sn_psi3), 0;...
                             0, 0,         0, 0,         0, 0,         0, 0,         0, 0,         0, 0,            0, 0,            0, 0,            0, 0,            0, 1;...
                   ];
               
               Qt = Bt*qu*Bt';
               Pt = Pt + (Ts/step)*(Qt + At*Pt + Pt*At');
        end
%% Measurement update of Surveillance Vehicles from Landmarks
        for i = 1:1:P.NumVeh
            for j = 1:1:P.NumLm
                if rho_lm(i,j) ~= -999
                    x1 = xHat(3*i-2);
                    y1 = xHat(3*i-1);
                    psi1 = xHat(3*i);
                    
                    h10 = sqrt((x1-P.LmX(j))^2+(y1-P.LmY(j))^2);
                    H10 = zeros(1,(P.NumVeh+P.NumSenVeh)*3);
                    H10(1,3*i-2) = (x1-P.LmX(j))/h10;
                    H10(1,3*i-1) = (y1-P.LmY(j))/h10;
                    Rt = P.SigmaRange^2;
                    
                    Lt = Pt*H10'/(Rt+H10*Pt*H10');
                    Pt = (eye(numel(xHat))-Lt*H10)*Pt;
                    xHat = xHat + Lt*(rho_lm(i,j)-h10);
                end
            end
        end
        
%% Measurement update of Sensor Network Vehicles from Landmarks
        for i = 1:1:P.NumSenVeh
            k = i+P.NumVeh;
            for j = 1:1:P.NumLm
                if rho_sen_lm(i,j) ~= -999
                    x1 = xHat(3*k-2);
                    y1 = xHat(3*k-1);
                    psi1 = xHat(3*k);
                    
                    h10 = sqrt((x1-P.LmX(j))^2+(y1-P.LmY(j))^2);
                    H10 = zeros(1,(P.NumVeh+P.NumSenVeh)*3);
                    H10(1,3*k-2) = (x1-P.LmX(j))/h10;
                    H10(1,3*k-1) = (y1-P.LmY(j))/h10;
                    Rt = P.SigmaRange^2;
                    
                    Lt = Pt*H10'/(Rt+H10*Pt*H10');
                    Pt = (eye(numel(xHat))-Lt*H10)*Pt;
                    xHat = xHat + Lt*(rho_sen_lm(i,j)-h10);
                end
            end
        end
%% Measurement updates between Surveillance Vehicles
        for i = 1:1:P.NumVeh
            for j = i+1:1:P.NumVeh
                
                if rho_veh(i,j) ~= -999
                    x1 = xHat(3*i-2);
                    y1 = xHat(3*i-1);
                    psi1 = xHat(3*i);

                    x2 = xHat(3*j-2);
                    y2 = xHat(3*j-1);
                    psi2 = xHat(3*j);

                    h10 = sqrt((x1-x2)^2+(y1-y2)^2);
                    H10 = zeros(1,(P.NumVeh+P.NumSenVeh)*3);
                    H10(1,3*i-2) = (x1-x2)/h10;
                    H10(1,3*i-1) = (y1-y2)/h10;
                    H10(1,3*j-2) = -(x1-x2)/h10;
                    H10(1,3*j-1) = -(y1-y2)/h10;
                    Rt = P.SigmaRange^2;

                    Lt = Pt*H10'/(Rt+H10*Pt*H10');
                    Pt = (eye(numel(xHat))-Lt*H10)*Pt;
                    xHat = xHat + Lt*(rho_veh(i,j)-h10);
                end
                
            end
        end
%% Measurement updates between Sensor Network Vehicles
        for i = 1:1:P.NumSenVeh
            k = i + P.NumVeh;
            for j = i+1:1:P.NumSenVeh
                l = j + P.NumVeh;
                
                if rho_sen_veh(i,j) ~= -999
                    x1 = xHat(3*k-2);
                    y1 = xHat(3*k-1);
                    psi1 = xHat(3*k);

                    x2 = xHat(3*l-2);
                    y2 = xHat(3*l-1);
                    psi2 = xHat(3*l);

                    h10 = sqrt((x1-x2)^2+(y1-y2)^2);
                    H10 = zeros(1,(P.NumVeh+P.NumSenVeh)*3);
                    H10(1,3*k-2) = (x1-x2)/h10;
                    H10(1,3*k-1) = (y1-y2)/h10;
                    H10(1,3*l-2) = -(x1-x2)/h10;
                    H10(1,3*l-1) = -(y1-y2)/h10;
                    Rt = P.SigmaRange^2;

                    Lt = Pt*H10'/(Rt+H10*Pt*H10');
                    Pt = (eye(numel(xHat))-Lt*H10)*Pt;
                    xHat = xHat + Lt*(rho_sen_veh(i,j)-h10);
                end
                
            end
        end
%% Measurement update between Sensor Network and Sureillance Vehicles
%         for i = 1:1:P.NumVeh
%             for j = i+1:1:P.NumSenVeh
%                 k = j + P.NumVeh;
%                 
%                 if rho_sen_sur_veh(i,j) ~= -999
%                     x1 = xHat(3*i-2);
%                     y1 = xHat(3*i-1);
%                     psi1 = xHat(3*i);
% 
%                     x2 = xHat(3*k-2);
%                     y2 = xHat(3*k-1);
%                     psi2 = xHat(3*k);
% 
%                     h10 = sqrt((x1-x2)^2+(y1-y2)^2);
%                     H10 = zeros(1,(P.NumVeh+P.NumSenVeh)*3);
%                     H10(1,3*i-2) = (x1-x2)/h10;
%                     H10(1,3*i-1) = (y1-y2)/h10;
%                     H10(1,3*k-2) = -(x1-x2)/h10;
%                     H10(1,3*k-1) = -(y1-y2)/h10;
%                     Rt = P.SigmaRange^2;
% 
%                     Lt = Pt*H10'/(Rt+H10*Pt*H10');
%                     Pt = (eye(numel(xHat))-Lt*H10)*Pt;
%                     xHat = xHat + Lt*(rho_sen_sur_veh(i,j)-h10);
%                 end
%                 
%             end
%         end

    end

    %% Output
    tPr = t;
    out = [xHat;diag(Pt)];
end