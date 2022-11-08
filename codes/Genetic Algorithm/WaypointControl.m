%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%

function out = WaypointControl(in,P)
%% Inertial States of Surveillance Vehicles     
    NN       = 0;
    x(1,1)   = in(NN+1);
    y(1,1)   = in(NN+2);
    psi(1,1) = in(NN+3);
    
    NN       = NN+3;
    x(2,1)   = in(NN+1);
    y(2,1)   = in(NN+2);
    psi(2,1) = in(NN+3);
    
    NN       = NN+3;
    x(3,1)   = in(NN+1);
    y(3,1)   = in(NN+2);
    psi(3,1) = in(NN+3);
    
    NN       = NN+3;
    x(4,1)   = in(NN+1);
    y(4,1)   = in(NN+2);
    psi(4,1) = in(NN+3);
    
    NN       = NN+3;
    x(5,1)   = in(NN+1);
    y(5,1)   = in(NN+2);
    psi(5,1) = in(NN+3);
    
    NN       = NN+3;
    x(6,1)   = in(NN+1);
    y(6,1)   = in(NN+2);
    psi(6,1) = in(NN+3);
    
    NN       = NN+3;
    t        = in(NN+1);
    
%%
    persistent vehDes
    
    if t == 0
        for i = 1:1:P.NumVeh
            vehDes(i).iter = 1;
            veh(i).vel = 1;
            veh(i).omg = 0.1;
        end
    end
%%    
    for i = 1:1:P.NumVeh
        [veh(i).vel,veh(i).omg,vehDes(i).iter] = wpControl(x(i),y(i),psi(i),vehDes(i).iter,i,P);
    end
%%
    v0 = veh(1).vel;
    w0 = veh(1).omg;
    v1 = veh(2).vel;
    w1 = veh(2).omg;
    v2 = veh(3).vel;
    w2 = veh(3).omg;
    v3 = veh(4).vel;
    w3 = veh(4).omg;
    v4 = veh(5).vel;
    w4 = veh(5).omg;
    v5 = veh(6).vel;
    w5 = veh(6).omg;

    
    out = [v0;w0;v1;w1;v2;w2;v3;w3;v4;w4;v5;w5];
    

end

%% wp control

function [v,w,id] = wpControl(x,y,psi,id,vehNum,P)

    xd   = P.WP(vehNum).WP(id,1);
    yd   = P.WP(vehNum).WP(id,2);
    psid = wrapToPi(atan2(yd-y,xd-x));
    
    w = 0.75*wrapToPi(psid-psi);
        
    rp = sqrt((xd-x)^2+(yd-y)^2);
        
    if rp <= 0.1
        v = 0;
        id = id+1;
    else
        v = 1;
    end
        
    if id > 5
        id = 1;
    end
    
end