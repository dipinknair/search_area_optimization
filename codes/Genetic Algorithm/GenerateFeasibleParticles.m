%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%
function [xVec,yVec] = GenerateFeasibleParticles(P)
    xPos = unifrnd(P.posMinX,P.posMaxX,[P.NumWP,1]);
    yPos = unifrnd(P.posMinY,P.posMaxY,[P.NumWP,1]);
    
    feasible_check = inpolygon(xPos,yPos,P.map.SearchSpacePolygon.Vertices(:,1),P.map.SearchSpacePolygon.Vertices(:,2));
    
    xPos = xPos(feasible_check);
    yPos = yPos(feasible_check);
    
    obstacle1_check = inpolygon(xPos,yPos,P.map.Obstacle1.Vertices(:,1),P.map.Obstacle1.Vertices(:,2));
    
    xPos = xPos(~obstacle1_check);
    yPos = yPos(~obstacle1_check);
    
    obstacle2_check = inpolygon(xPos,yPos,P.map.Obstacle2.Vertices(:,1),P.map.Obstacle2.Vertices(:,2));
    xPos = xPos(~obstacle2_check);
    yPos = yPos(~obstacle2_check);
    
    while (numel(xPos)<P.NumWP)
        xNew = unifrnd(P.posMinX,P.posMaxX,1);
        yNew = unifrnd(P.posMinY,P.posMaxY,1);
        
        feasible_check = inpolygon(xNew,yNew,P.map.SearchSpacePolygon.Vertices(:,1),P.map.SearchSpacePolygon.Vertices(:,2));
        
        if feasible_check
            obstacle1_check = inpolygon(xNew,yNew,P.map.Obstacle1.Vertices(:,1),P.map.Obstacle1.Vertices(:,2));
            obstacle2_check = inpolygon(xNew,yNew,P.map.Obstacle2.Vertices(:,1),P.map.Obstacle2.Vertices(:,2));
            
            if ~obstacle1_check && ~obstacle2_check
                xPos = [xPos;xNew];
                yPos = [yPos;yNew];
            end
            
        end
    end
    
    xVec = xPos;
    yVec = yPos;
end