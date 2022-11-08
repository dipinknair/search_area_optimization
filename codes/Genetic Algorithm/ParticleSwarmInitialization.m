%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%

function [wp,velXMin,velXMax,velYMin,velYMax] = ParticleSwarmInitialization(map,SD)
    
    bxMin = map.bxMin;
    bxMax = map.bxMax;
    byMin = map.byMin;
    byMax = map.byMax;
    
    env   = map.env;
    
    obst1 = map.obst1;
    obst2 = map.obst2;

    nX = round((bxMax-bxMin)/SD);
    nY = round((byMax-byMin)/SD);
    
    wpX = linspace(bxMin,bxMax,nX);
    wpY = linspace(byMin,byMax,nY);
    %[X,Y] = meshgrid(x1,y1);
    
    %wpX = reshape(X,[],1);
    %wpY = reshape(Y,[],1);
    
    Kx = 0.1;
    Ky = 0.1;
    
    velXMax = Kx*(bxMax-bxMin);
    velXMin = -velXMax;
    velYMax = Ky*(byMax-byMin);
    velYMin = -velYMax;
    
    wp = [];
    
    for i = 1:1:numel(wpX)
        for j = 1:1:numel(wpY)
            
            feasible_check = inpolygon(wpX(i),wpY(j),env(:,1),env(:,2));
            
            if feasible_check == 1
                obs_check = inpolygon(wpX(i),wpY(j),obst1(:,1),obst1(:,2)) || inpolygon(wpX(i),wpY(j),obst2(:,1),obst2(:,2));
                
                if obs_check ~= 1
                    
                    wp = [wp;wpX(i) wpY(j)];
                    
                end
            end
            
        end
    end
    
    
end