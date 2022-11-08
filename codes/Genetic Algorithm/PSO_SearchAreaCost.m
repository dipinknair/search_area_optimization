%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%

function J = PSO_SearchAreaCost(Pop,P)
    
    xc = Pop.xPos;
    yc = Pop.yPos;
    
    goodParticles = inpolygon(xc,yc,P.map.SearchSpacePolygon.Vertices(:,1),P.map.SearchSpacePolygon.Vertices(:,2));
    NumGoodParticles = sum(goodParticles);
    NumBadParticles = P.NumWP-NumGoodParticles;
    
    CostEnv = 0;
    if NumBadParticles ~= 0
        CostEnv = NumBadParticles/P.NumWP + P.OptimalityCriteria;
    end
    
    TotalSearchArea = P.map.SearchPolygonArea;
    UAVSearchArea = P.SearchAreaUAV;
    SearchSpacePolygon = P.map.SearchSpacePolygon;
    WPAreaUnion = polyshape([0 0 0 0],[0 0 0 0]);
    
    L = linspace(0,2*pi);
    circX = cos(L);
    circY = sin(L);
    rad = P.SearchRadUAV;
    
    for i = 1:1:P.NumWP
        x_cir = rad*circX + xc(i);
        y_cir = rad*circY + yc(i);
        
        CircleI = polyshape({x_cir},{y_cir});
        WPAreaUnion = union(WPAreaUnion,CircleI);
    end
    
    CoverageArea = area(intersect(WPAreaUnion,SearchSpacePolygon));
    AreaOverlap = UAVSearchArea*P.NumWP - CoverageArea;
    
    CostCoverage = 1 - (CoverageArea/TotalSearchArea);
    CostOverlap  = AreaOverlap/(UAVSearchArea*P.NumWP);
%% Total cost
    CostTotal = P.lam.Env*CostEnv + P.lam.Coverage*CostCoverage + P.lam.Overlap*CostOverlap;
    
    J.CostEnv = CostEnv;
    J.CostCoverage = CostCoverage;
    J.CostOverlap = CostOverlap;
    J.CostTotal = CostTotal; 
    
end