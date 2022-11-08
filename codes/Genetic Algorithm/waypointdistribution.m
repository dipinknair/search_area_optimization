function wpnumber = waypointdistribution(P)  
TotalWPdist=randperm(P.NumWP);
extraWPs = [];

%making distribution WPS equal number
for i= 1:P.NumVeh
    wpnumber(i).veh = TotalWPdist(((i-1)*fix(P.NumWP/P.NumVeh) +1):fix(P.NumWP/P.NumVeh)*i);
    if i == P.NumVeh && mod(P.NumWP,P.NumVeh) ~= 0 
        extraWPs = mod(P.NumWP,P.NumVeh);
        for j = 1:extraWPs
            randomveh = randi(P.NumVeh);
            wpnumber(randomveh).veh
            wpnumber(randomveh).veh(end + 1) = TotalWPdist((fix(P.NumWP/P.NumVeh)*i)+j);
            wpnumber(randomveh).veh
            %wpnumber(:,randomveh) = [wpnumber(:,randomveh);TotalWPdist(P.NumWP+j)];
        end 
    end
end
%for i = 1:P.NumVeh
%  WPXmatrix(i).veh = WPx(wpnumber(i).veh)
%  WPYmatrix(i).veh = WPx(wpnumber(i).veh)
%end
