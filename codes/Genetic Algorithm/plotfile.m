%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%

function plotfile
    close all;
    load databag_simplecontrol_iter6.mat
    
    t      = data.Time;
    states = data.Data;
%% True Inertial States of Surveillance Vehicles
    NN = 0;
    x0 = states(:,NN+1);
    y0 = states(:,NN+2);
    psi0 = states(:,NN+3);
    NN = NN+3;
    x1 = states(:,NN+1);
    y1 = states(:,NN+2);
    psi1 = states(:,NN+3);
    NN = NN+3;
    x2 = states(:,NN+1);
    y2 = states(:,NN+2);
    psi2 = states(:,NN+3);
    NN = NN+3;
    x3 = states(:,NN+1);
    y3 = states(:,NN+2);
    psi3 = states(:,NN+3);
    NN = NN+3;
    x4 = states(:,NN+1);
    y4 = states(:,NN+2);
    psi4 = states(:,NN+3);
    NN = NN+3;
    x5 = states(:,NN+1);
    y5 = states(:,NN+2);
    psi5 = states(:,NN+3);
    
%% True Inertial States of Sensor Network Vehicles
    NN = NN+4;
    sn_x0 = states(:,NN+1);
    sn_y0 = states(:,NN+2);
    sn_psi0 = states(:,NN+3);
    NN = NN+3;
    sn_x1 = states(:,NN+1);
    sn_y1 = states(:,NN+2);
    sn_psi1 = states(:,NN+3);
    NN = NN+3;
    sn_x2 = states(:,NN+1);
    sn_y2 = states(:,NN+2);
    sn_psi2 = states(:,NN+3);
    NN = NN+3;
    sn_x3 = states(:,NN+1);
    sn_y3 = states(:,NN+2);
    sn_psi3 = states(:,NN+3);
    
%% Control inputs of Surveillance Vehicles
    NN = NN+4;
    v0 = states(:,NN+1);
    w0 = states(:,NN+2);
    NN = NN+2;
    v1 = states(:,NN+1);
    w1 = states(:,NN+2);
    NN = NN+2;
    v2 = states(:,NN+1);
    w2 = states(:,NN+2);
    NN = NN+2;
    v3 = states(:,NN+1);
    w3 = states(:,NN+2);
    NN = NN+2;
    v4 = states(:,NN+1);
    w4 = states(:,NN+2);
    NN = NN+2;
    v5 = states(:,NN+1);
    w5 = states(:,NN+2);
    
%% Control inputs of Sensor Network Vehicles
    NN = NN+2;
    sn_v0 = states(:,NN+1);
    sn_w0 = states(:,NN+2);
    NN = NN+2;
    sn_v1 = states(:,NN+1);
    sn_w1 = states(:,NN+2);
    NN = NN+2;
    sn_v2 = states(:,NN+1);
    sn_w2 = states(:,NN+2);
    NN = NN+2;
    sn_v3 = states(:,NN+1);
    sn_w3 = states(:,NN+2);
    
%% Estimated States
    NN= NN+2;
    x0Hat = states(:,NN+1);
    y0Hat = states(:,NN+2);
    psi0Hat = states(:,NN+3);
    NN= NN+3;
    x1Hat = states(:,NN+1);
    y1Hat = states(:,NN+2);
    psi1Hat = states(:,NN+3);
    NN= NN+3;
    x2Hat = states(:,NN+1);
    y2Hat = states(:,NN+2);
    psi2Hat = states(:,NN+3);
    NN= NN+3;
    x3Hat = states(:,NN+1);
    y3Hat = states(:,NN+2);
    psi3Hat = states(:,NN+3);
    NN= NN+3;
    x4Hat = states(:,NN+1);
    y4Hat = states(:,NN+2);
    psi4Hat = states(:,NN+3);
    NN= NN+3;
    x5Hat = states(:,NN+1);
    y5Hat = states(:,NN+2);
    psi5Hat = states(:,NN+3);
    
    NN= NN+3;
    sn_x0Hat = states(:,NN+1);
    sn_y0Hat = states(:,NN+2);
    sn_psi0Hat = states(:,NN+3);
    NN= NN+3;
    sn_x1Hat = states(:,NN+1);
    sn_y1Hat = states(:,NN+2);
    sn_psi1Hat = states(:,NN+3);
    NN= NN+3;
    sn_x2Hat = states(:,NN+1);
    sn_y2Hat = states(:,NN+2);
    sn_psi2Hat = states(:,NN+3);
    NN= NN+3;
    sn_x3Hat = states(:,NN+1);
    sn_y3Hat = states(:,NN+2);
    sn_psi3Hat = states(:,NN+3);
    
    NN = NN+3;
    x0_cov = states(:,NN+1);
    y0_cov = states(:,NN+2);
    psi0_cov = states(:,NN+3);
    NN = NN+3;
    x1_cov = states(:,NN+1);
    y1_cov = states(:,NN+2);
    psi1_cov = states(:,NN+3);
    NN = NN+3;
    x2_cov = states(:,NN+1);
    y2_cov = states(:,NN+2);
    psi2_cov = states(:,NN+3);
    NN = NN+3;
    x3_cov = states(:,NN+1);
    y3_cov = states(:,NN+2);
    psi3_cov = states(:,NN+3);
    NN = NN+3;
    x4_cov = states(:,NN+1);
    y4_cov = states(:,NN+2);
    psi4_cov = states(:,NN+3);
    NN = NN+3;
    x5_cov = states(:,NN+1);
    y5_cov = states(:,NN+2);
    psi5_cov = states(:,NN+3);
    
    NN = NN+3;
    sn_x0_cov = states(:,NN+1);
    sn_y0_cov = states(:,NN+2);
    sn_psi0_cov = states(:,NN+3);
    NN = NN+3;
    sn_x1_cov = states(:,NN+1);
    sn_y1_cov = states(:,NN+2);
    sn_psi1_cov = states(:,NN+3);
    NN = NN+3;
    sn_x2_cov = states(:,NN+1);
    sn_y2_cov = states(:,NN+2);
    sn_psi2_cov = states(:,NN+3);
    NN = NN+3;
    sn_x3_cov = states(:,NN+1);
    sn_y3_cov = states(:,NN+2);
    sn_psi3_cov = states(:,NN+3);
        
%% True vs Estimated Trajectory Plot
    figure(1)
    plot(x0,y0,'b',x1,y1,'b',x2,y2,'b',x3,y3,'b',x4,y4,'b',x5,y5,'b',sn_x0,sn_y0,'b',sn_x1,sn_y1,'b',sn_x2,sn_y2,'b',sn_x3,sn_y3,'b','LineWidth',2);
    hold on;
    plot(x0Hat,y0Hat,'r--',x1Hat,y1Hat,'r--',x2Hat,y2Hat,'r--',x3Hat,y3Hat,'r--',x4Hat,y4Hat,'r--',x5Hat,y5Hat,'r--',sn_x0Hat,sn_y0Hat,'r--',sn_x1Hat,sn_y1Hat,'r--',sn_x2Hat,sn_y2Hat,'r--',sn_x3Hat,sn_y3Hat,'r--','LineWidth',1.5);
    %plot(P.LmX,P.LmY,'*','lineWidth',1.5);
    xlabel('X - Pos (m)');
    ylabel('Y - Pos (m)');
  
%% True vs Estimated
    figure(2)
    plot(x0,y0,'b',x0Hat,y0Hat,'r--','LineWidth',2);
    xlabel('X - Pos (m)');
    ylabel('Y - Pos (m)');
    
    figure(3)
    plot(x1,y1,'b',x1Hat,y1Hat,'r--','LineWidth',2);
    xlabel('X - Pos (m)');
    ylabel('Y - Pos (m)');
    
    figure(4)
    plot(x2,y2,'b',x2Hat,y2Hat,'r--','LineWidth',2);
    xlabel('X - Pos (m)');
    ylabel('Y - Pos (m)');
    
    figure(5)
    plot(x3,y3,'b',x3Hat,y3Hat,'r--','LineWidth',2);
    xlabel('X - Pos (m)');
    ylabel('Y - Pos (m)');
    
    figure(6)
    plot(x4,y4,'b',x4Hat,y4Hat,'r--','LineWidth',2);
    xlabel('X - Pos (m)');
    ylabel('Y - Pos (m)');
    
    figure(7)
    plot(x5,y5,'b',x5Hat,y5Hat,'r--','LineWidth',2);
    xlabel('X - Pos (m)');
    ylabel('Y - Pos (m)');
    
    figure(8)
    plot(sn_x0,sn_y0,'b',sn_x0Hat,sn_y0Hat,'r--','LineWidth',2);
    xlabel('X - Pos (m)');
    ylabel('Y - Pos (m)');
    
    figure(9)
    plot(sn_x1,sn_y1,'b',sn_x1Hat,sn_y1Hat,'r--','LineWidth',2);
    xlabel('X - Pos (m)');
    ylabel('Y - Pos (m)');
    
    figure(10)
    plot(sn_x2,sn_y2,'b',sn_x2Hat,sn_y2Hat,'r--','LineWidth',2);
    xlabel('X - Pos (m)');
    ylabel('Y - Pos (m)');
    
    figure(11)
    plot(sn_x3,sn_y3,'b',sn_x3Hat,sn_y3Hat,'r--','LineWidth',2);
    xlabel('X - Pos (m)');
    ylabel('Y - Pos (m)');
    
%% Error Covariance
    figure(12)
    subplot(3,1,1)
    plot(t,x0-x0Hat,'b',t,-3*x0_cov,'r--',t,3*x0_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('X_{err} (m)')
    subplot(3,1,2)
    plot(t,y0-y0Hat,'b',t,-3*y0_cov,'r--',t,3*y0_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('Y_{err} (m)')
    subplot(3,1,3)
    plot(t,psi0-psi0Hat,'b',t,-3*psi0_cov,'r--',t,3*psi0_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('\psi_{err} (m)')
    
    figure(13)
    subplot(3,1,1)
    plot(t,x1-x1Hat,'b',t,-3*x1_cov,'r--',t,3*x1_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('X_{err} (m)')
    subplot(3,1,2)
    plot(t,y1-y1Hat,'b',t,-3*y1_cov,'r--',t,3*y1_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('Y_{err} (m)')
    subplot(3,1,3)
    plot(t,psi1-psi1Hat,'b',t,-3*psi1_cov,'r--',t,3*psi1_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('\psi_{err} (m)')
    
    figure(14)
    subplot(3,1,1)
    plot(t,x2-x2Hat,'b',t,-3*x2_cov,'r--',t,3*x2_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('X_{err} (m)')
    subplot(3,1,2)
    plot(t,y2-y2Hat,'b',t,-3*y2_cov,'r--',t,3*y2_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('Y_{err} (m)')
    subplot(3,1,3)
    plot(t,psi2-psi2Hat,'b',t,-3*psi2_cov,'r--',t,3*psi2_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('\psi_{err} (m)')
    
    figure(15)
    subplot(3,1,1)
    plot(t,x3-x3Hat,'b',t,-3*x3_cov,'r--',t,3*x3_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('X_{err} (m)')
    subplot(3,1,2)
    plot(t,y3-y3Hat,'b',t,-3*y3_cov,'r--',t,3*y3_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('Y_{err} (m)')
    subplot(3,1,3)
    plot(t,psi3-psi3Hat,'b',t,-3*psi3_cov,'r--',t,3*psi3_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('\psi_{err} (m)')
    
    figure(16)
    subplot(3,1,1)
    plot(t,x4-x4Hat,'b',t,-3*x4_cov,'r--',t,3*x4_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('X_{err} (m)')
    subplot(3,1,2)
    plot(t,y4-y4Hat,'b',t,-3*y4_cov,'r--',t,3*y4_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('Y_{err} (m)')
    subplot(3,1,3)
    plot(t,psi4-psi4Hat,'b',t,-3*psi4_cov,'r--',t,3*psi4_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('\psi_{err} (m)')
    
    figure(17)
    subplot(3,1,1)
    plot(t,x5-x5Hat,'b',t,-3*x5_cov,'r--',t,3*x5_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('X_{err} (m)')
    subplot(3,1,2)
    plot(t,y5-y5Hat,'b',t,-3*y5_cov,'r--',t,3*y5_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('Y_{err} (m)')
    subplot(3,1,3)
    plot(t,psi5-psi5Hat,'b',t,-3*psi5_cov,'r--',t,3*psi5_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('\psi_{err} (m)')
    
    figure(18)
    subplot(3,1,1)
    plot(t,sn_x0-sn_x0Hat,'b',t,-3*sn_x0_cov,'r--',t,3*sn_x0_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('X_{err} (m)')
    subplot(3,1,2)
    plot(t,sn_y0-sn_y0Hat,'b',t,-3*sn_y0_cov,'r--',t,3*sn_y0_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('Y_{err} (m)')
    subplot(3,1,3)
    plot(t,sn_psi0-sn_psi0Hat,'b',t,-3*sn_psi0_cov,'r--',t,3*sn_psi0_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('\psi_{err} (m)')
    
    figure(19)
    subplot(3,1,1)
    plot(t,sn_x1-sn_x1Hat,'b',t,-3*sn_x1_cov,'r--',t,3*sn_x1_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('X_{err} (m)')
    subplot(3,1,2)
    plot(t,sn_y1-sn_y1Hat,'b',t,-3*sn_y1_cov,'r--',t,3*sn_y1_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('Y_{err} (m)')
    subplot(3,1,3)
    plot(t,sn_psi1-sn_psi1Hat,'b',t,-3*sn_psi1_cov,'r--',t,3*sn_psi1_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('\psi_{err} (m)')
    
    figure(20)
    subplot(3,1,1)
    plot(t,sn_x2-sn_x2Hat,'b',t,-3*sn_x2_cov,'r--',t,3*sn_x2_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('X_{err} (m)')
    subplot(3,1,2)
    plot(t,sn_y2-sn_y2Hat,'b',t,-3*sn_y2_cov,'r--',t,3*sn_y2_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('Y_{err} (m)')
    subplot(3,1,3)
    plot(t,sn_psi2-sn_psi2Hat,'b',t,-3*sn_psi2_cov,'r--',t,3*sn_psi2_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('\psi_{err} (m)')
    
    figure(21)
    subplot(3,1,1)
    plot(t,sn_x3-sn_x3Hat,'b',t,-3*sn_x3_cov,'r--',t,3*sn_x3_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('X_{err} (m)')
    subplot(3,1,2)
    plot(t,sn_y3-sn_y3Hat,'b',t,-3*sn_y3_cov,'r--',t,3*sn_y3_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('Y_{err} (m)')
    subplot(3,1,3)
    plot(t,sn_psi3-sn_psi3Hat,'b',t,-3*sn_psi3_cov,'r--',t,3*sn_psi3_cov,'r--','LineWidth',2);
    xlabel('t (s)');
    ylabel('\psi_{err} (m)')
    
%%
    load GL.mat
    
    figure(22)
    plot(t,GL.Data)
    
end