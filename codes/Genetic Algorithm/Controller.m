%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%

function out = Controller(in,P)
%     
    NN = 0;
    x0 = in(NN+1);
    y0 = in(NN+2);
    psi0 = in(NN+3);
    
    NN = NN+3;
    x1 = in(NN+1);
    y1 = in(NN+2);
    psi1 = in(NN+3);
    
    NN = NN+3;
    x2 = in(NN+1);
    y2 = in(NN+2);
    psi2 = in(NN+3);
    
    NN = NN+3;
    x3 = in(NN+1);
    y3 = in(NN+2);
    psi3 = in(NN+3);
    
    NN = NN+3;
    x4 = in(NN+1);
    y4 = in(NN+2);
    psi4 = in(NN+3);
    
    NN = NN+3;
    x5 = in(NN+1);
    y5 = in(NN+2);
    psi5 = in(NN+3);
    
    NN = NN+3;
    t = in(NN+1);
    
    persistent i0 i1 i2 i3 i4 i5 
    
    if t == 0
        i0 = 1;
        i1 = 1;
        i2 = 1;
        i3 = 1;
        i4 = 1;
        i5 = 1;
    end
    
    %while (i0 <= 5)
        
        x0d = P.WP(1).WP(i0,1); y0d = P.WP(1).WP(i0,2); psi0d = wrapToPi(atan2(y0d-y0,x0d-x0));
        
        w0 = 0.75*wrapToPi(psi0d-psi0);
        
        rp0 = sqrt((x0d-x0)^2+(y0d-y0)^2);
        
        if rp0 <= 0.1
            v0 = 0;
            i0 = i0+1;
        else
            v0 = 1;
        end
        
        if i0 > 5
            i0 = 1;
        end
        
    %end
    
    %while (i1 <= 5)
        x1d = P.WP(2).WP(i1,1); y1d = P.WP(2).WP(i1,2); psi1d = wrapToPi(atan2(y1d-y1,x1d-x1));
        w1 = 0.75*wrapToPi(psi1d-psi1);
        rp1 = sqrt((x1d-x1)^2+(y1d-y1)^2);
        
        if rp1 <= 0.1
            v1 = 0;
            i1 = i1+1;
        else
            v1 = 1;
        end  
        if i1 > 5
            i1 = 1;
        end
    %end
    
    %while (i2 <= 5)
        x2d = P.WP(3).WP(i2,1); y2d = P.WP(3).WP(i2,2); psi2d = wrapToPi(atan2(y2d-y2,x2d-x2));
        w2 = 0.75*wrapToPi(psi2d-psi2);
        rp2 = sqrt((x2d-x2)^2+(y2d-y2)^2);
        
        if rp2 <= 0.1
            v2 = 0;
            i2 = i2+1;
        else
            v2 = 1;
        end  
        if i2 > 5
            i2 = 1;
        end
    %end
    
    %while (i3 <= 5)
        x3d = P.WP(4).WP(i3,1); y3d = P.WP(4).WP(i3,2); psi3d = wrapToPi(atan2(y3d-y3,x3d-x3));
        w3 = 0.75*wrapToPi(psi3d-psi3);
        rp3 = sqrt((x3d-x3)^2+(y3d-y3)^2);
        
        if rp3 <= 0.1
            v3 = 0;
            i3 = i3+1;
        else
            v3 = 1;
        end  
        if i3 > 5
            i3 = 1;
        end
    %end
    
    %while (i4 <= 5)
        x4d = P.WP(5).WP(i4,1); y4d = P.WP(5).WP(i4,2); psi4d = wrapToPi(atan2(y4d-y4,x4d-x4));
        w4 = 0.75*wrapToPi(psi4d-psi4);
        rp4 = sqrt((x4d-x4)^2+(y4d-y4)^2);
        
        if rp4 <= 0.1
            v4 = 0;
            i4 = i4+1;
        else
            v4 = 1;
        end  
        if i4 > 5
            i4 = 1;
        end
    %end
    
    %while (i5 <= 5)
        x5d = P.WP(6).WP(i5,1); y5d = P.WP(6).WP(i5,2); psi5d = wrapToPi(atan2(y5d-y5,x5d-x5));
        w5 = 0.75*wrapToPi(psi5d-psi5);
        rp5 = sqrt((x5d-x5)^2+(y5d-y5)^2);
        
        if rp5 <= 0.1
            v5 = 0;
            i5 = i5+1;
        else
            v5 = 1;
        end  
        if i5 > 5
            i5 = 1;
        end
    %end     

% v0 = 1;
% w0 = 0.1;
% v1 = 1;
% w1 = 0.1;
% v2 = 1;
% w2 = 0.1;
% v3 = 1;
% w3 = 0.1;
% v4 = 1;
% w4 = 0.1;
% v5 = 1;
% w5 = 0.1;

    
    out = [v0;w0;v1;w1;v2;w2;v3;w3;v4;w4;v5;w5];
    

end