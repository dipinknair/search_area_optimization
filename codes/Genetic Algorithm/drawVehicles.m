%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%

function drawVehicles(u,P)
%% Inertial Positions from Surveillance Vehicles
    NN      = 0;
    x0      = u(NN+1);
    y0      = u(NN+2);
    psi0    = u(NN+3);
    NN      = NN+3;
    x1      = u(NN+1);
    y1      = u(NN+2);
    psi1    = u(NN+3);
    NN      = NN+3;
    x2      = u(NN+1);
    y2      = u(NN+2);
    psi2    = u(NN+3);
    NN      = NN+3;
    x3      = u(NN+1);
    y3      = u(NN+2);
    psi3    = u(NN+3);
    NN      = NN+3;
    x4      = u(NN+1);
    y4      = u(NN+2);
    psi4    = u(NN+3);
    NN      = NN+3;
    x5      = u(NN+1);
    y5      = u(NN+2);
    psi5    = u(NN+3);
    NN      = NN+3;
    t       = u(NN+1);
%% Inertial Positions from Sensing Network Vehicles
    NN      = NN+1;
    sn_x0   = u(NN+1);
    sn_y0   = u(NN+2);
    sn_psi0 = u(NN+3);
    NN      = NN+3;
    sn_x1   = u(NN+1);
    sn_y1   = u(NN+2);
    sn_psi1 = u(NN+3);
    NN      = NN+3;
    sn_x2   = u(NN+1);
    sn_y2   = u(NN+2);
    sn_psi2 = u(NN+3);
    NN      = NN+3;
    sn_x3   = u(NN+1);
    sn_y3   = u(NN+2);
    sn_psi3 = u(NN+3);
    %NN    = NN+3;
    %t     = u(NN+1);
 
%%
    persistent vehicle_handle0 trajectory_handle0
    persistent vehicle_handle1 trajectory_handle1
    persistent vehicle_handle2 trajectory_handle2
    persistent vehicle_handle3 trajectory_handle3
    persistent vehicle_handle4 trajectory_handle4
    persistent vehicle_handle5 trajectory_handle5
    
    persistent sn_vehicle_handle0 sn_trajectory_handle0
    persistent sn_vehicle_handle1 sn_trajectory_handle1
    persistent sn_vehicle_handle2 sn_trajectory_handle2
    persistent sn_vehicle_handle3 sn_trajectory_handle3
    %num_wp = numel(P.wp(:,1));
    
    if t == 0
        close all;
        figure(1);
%         if P.control_flag.veh1 == 3
%             for i = 1:1:num_wp
%                 plot(P.wp(i,1),P.wp(i,2),'dk','LineWidth',2);
%                 hold on;
%             end
%         end
        hold on;
        plot(P.map.SearchSpacePolygon);
        plot(P.TotWP(:,1),P.TotWP(:,2),'ko');
        plot(P.LmX,P.LmY,'g*','LineWidth',2);
        vehicle_handle0 = drawVehicle(x0,y0,psi0,[]);
        vehicle_handle1 = drawVehicle(x1,y1,psi1,[]);
        vehicle_handle2 = drawVehicle(x2,y2,psi2,[]);
        vehicle_handle3 = drawVehicle(x3,y3,psi3,[]);
        vehicle_handle4 = drawVehicle(x4,y4,psi4,[]);
        vehicle_handle5 = drawVehicle(x5,y5,psi5,[]);
        
        sn_vehicle_handle0 = drawVehicle_SN(sn_x0,sn_y0,sn_psi0,[]);
        sn_vehicle_handle1 = drawVehicle_SN(sn_x1,sn_y1,sn_psi1,[]);
        sn_vehicle_handle2 = drawVehicle_SN(sn_x2,sn_y2,sn_psi2,[]);
        sn_vehicle_handle3 = drawVehicle_SN(sn_x3,sn_y3,sn_psi3,[]);
        
        trajectory_handle0 = drawPath(x0,y0,[]);
        trajectory_handle1 = drawPath(x1,y1,[]);
        trajectory_handle2 = drawPath(x2,y2,[]);
        trajectory_handle3 = drawPath(x3,y3,[]);
        trajectory_handle4 = drawPath(x4,y4,[]);
        trajectory_handle5 = drawPath(x5,y5,[]);
        
        sn_trajectory_handle0 = drawPath_SN(sn_x0,sn_y0,[]);
        sn_trajectory_handle1 = drawPath_SN(sn_x1,sn_y1,[]);
        sn_trajectory_handle2 = drawPath_SN(sn_x2,sn_y2,[]);
        sn_trajectory_handle3 = drawPath_SN(sn_x3,sn_y3,[]);
        
        xlabel('X');
        ylabel('Y');
        %axis([-5 50 -10 25]);
    else
        drawVehicle(x0,y0,psi0,vehicle_handle0);
        drawVehicle(x1,y1,psi1,vehicle_handle1);
        drawVehicle(x2,y2,psi2,vehicle_handle2);
        drawVehicle(x3,y3,psi3,vehicle_handle3);
        drawVehicle(x4,y4,psi4,vehicle_handle4);
        drawVehicle(x5,y5,psi5,vehicle_handle5);
        
        drawVehicle_SN(sn_x0,sn_y0,sn_psi0,sn_vehicle_handle0);
        drawVehicle_SN(sn_x1,sn_y1,sn_psi1,sn_vehicle_handle1);
        drawVehicle_SN(sn_x2,sn_y2,sn_psi2,sn_vehicle_handle2);
        drawVehicle_SN(sn_x3,sn_y3,sn_psi3,sn_vehicle_handle3);
        
        drawPath(x0,y0,trajectory_handle0);
        drawPath(x1,y1,trajectory_handle1);
        drawPath(x2,y2,trajectory_handle2);
        drawPath(x3,y3,trajectory_handle3);
        drawPath(x4,y4,trajectory_handle4);
        drawPath(x5,y5,trajectory_handle5);
        
        drawPath_SN(sn_x0,sn_y0,sn_trajectory_handle0);
        drawPath_SN(sn_x1,sn_y1,sn_trajectory_handle1);
        drawPath_SN(sn_x2,sn_y2,sn_trajectory_handle2);
        drawPath_SN(sn_x3,sn_y3,sn_trajectory_handle3);
    end
       
end

%%
function handle = drawVehicle(x,y,psi,handle)

    Rot   = [...
           cos(psi) -sin(psi);...
           sin(psi)  cos(psi);...
           ];
    cod   = [...
           2 0  0 2;...
           0 1 -1 0;...
           ];
    codn  = [x;y]+Rot*cod;
    X     = codn(1,:);
    Y     = codn(2,:);
    
    if isempty(handle)
        handle = fill(X,Y,'b');
    else
        set(handle,'XData',X,'YData',Y);
        %drawnow;
    end
    
end

%%
function handle = drawVehicle_SN(x,y,psi,handle)

    Rot   = [...
           cos(psi) -sin(psi);...
           sin(psi)  cos(psi);...
           ];
    cod   = [...
           2 0  0 2;...
           0 1 -1 0;...
           ];
    codn  = [x;y]+Rot*cod;
    X     = codn(1,:);
    Y     = codn(2,:);
    
    if isempty(handle)
        handle = fill(X,Y,'r');
    else
        set(handle,'XData',X,'YData',Y);
        %drawnow;
    end
    
end

%%
function handle = drawPath(x,y,handle)

    
    if isempty(handle)
        handle = plot(x,y,'b--');
    else
        X = [handle.XData x];
        Y = [handle.YData y];
        set(handle,'XData',X,'YData',Y);
    end
    
end

%%
function handle = drawPath_SN(x,y,handle)

    
    if isempty(handle)
        handle = plot(x,y,'r--');
    else
        X = [handle.XData x];
        Y = [handle.YData y];
        set(handle,'XData',X,'YData',Y);
    end
    
end
