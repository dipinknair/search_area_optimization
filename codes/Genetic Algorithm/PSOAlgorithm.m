%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%

function out = PSOAlgorithm(particleStructure,P)
%%
    particles = repmat(particleStructure,[P.NumGen,1]);
    GlobalBest.Cost = P.GlobalBest;
    w = P.w;
    wDamp = P.wDamp;
    c1 = P.c1;
    c2 = P.c2;
    
%% Initialization
    for i = 1:1:P.NumGen
        [particles(i).xPos,particles(i).yPos] = GenerateFeasibleParticles(P);
        Cost = PSO_SearchAreaCost(particles(i),P);
        particles(i).Cost = Cost.CostTotal;
        
        particles(i).Best.xPos = particles(i).xPos;
        particles(i).Best.yPos = particles(i).yPos;
        particles(i).Best.Cost = particles(i).Cost;
        
        if (particles(i).Best.Cost < GlobalBest.Cost)
            GlobalBest = particles(i).Best;
        end 
        
    end
    BestCosts = zeros(P.maxIter,1);
%% Main PSO Loop
    for j = 1:1:P.maxIter
        clc; fprintf('Iteration %f',j);
        for i = 1:1:P.NumGen
            r1 = rand(P.NumWP,1);
            r2 = rand(P.NumWP,1);

            perXVel = particles(i).Best.xPos - particles(i).xPos;
            perYVel = particles(i).Best.yPos - particles(i).yPos; 

            globXVel = GlobalBest.xPos - particles(i).xPos;
            globYVel = GlobalBest.yPos - particles(i).yPos;

            particles(i).xVel = w*particles(i).xVel + c1*r1.*perXVel + c2*r2.*globXVel;
            particles(i).yVel = w*particles(i).yVel + c1*r1.*perYVel + c2*r2.*globYVel;

            particles(i).xVel = max(particles(i).xVel,P.velMinX);
            particles(i).xVel = min(particles(i).xVel,P.velMaxX);

            particles(i).yVel = max(particles(i).yVel,P.velMinY);
            particles(i).yVel = min(particles(i).yVel,P.velMaxY);

            particles(i).xPos = particles(i).xPos + particles(i).xVel;
            particles(i).yPos = particles(i).yPos + particles(i).yVel;

            Cost = PSO_SearchAreaCost(particles(i),P);
            particles(i).Cost = Cost.CostTotal;

            if (particles(i).Cost < particles(i).Best.Cost)
                particles(i).Best.xPos = particles(i).xPos;
                particles(i).Best.yPos = particles(i).yPos;
                particles(i).Best.Cost = particles(i).Cost;
            end

            if (particles(i).Best.Cost < GlobalBest.Cost)
                GlobalBest = particles(i).Best;
            end
        end
        
        w = w*wDamp;
        
        BestCosts(j) = GlobalBest.Cost;
    end
    
    out.Pop = particles;
    out.BestSoln = GlobalBest;
    out.BestCosts = BestCosts;
    
end