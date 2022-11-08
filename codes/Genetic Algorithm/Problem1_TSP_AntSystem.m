%%
%Assignment 6
%TSP using Ant System
%Rohith Boyinine
%Noel Jackayya
%%

%%
% clc;
% close all;
% clear all;
%%

%%intialization
function out = Problem1_TSP_AntSystem(city)
n = numel(city(:,1));
%city = rand(n,2)*100;
%city = [20 20; 20 40; 40 40;40 20];
S = 1:n;
if n <= 10
    time = 100;
else if n <= 15
        time = 150;
    else if n <= 20
            time = 200;
        else
            time = 1000;
        end
    end
end

for i = 1:1:n
    for j = 1:1:n
        d(i,j) = sqrt((city(i,1)-city(j,1))^2 + (city(i,2)-city(j,2))^2);
    end
end

alpha = 1;
beta = 5;
rho = 0.5;
m = n;
Q = 100;
e = 5;

for i = 1:1:n
    for j = 1:1:n
        tau(i,j) = 1e-6;
    end
end
%%

%%
for k = 1:1:m
    ant(k,1) = k;
    T(k,:) = ant(k);
    J(k,:) = setdiff(S,T(k,:));
end
T_opt = [];
L_opt = inf;
t = 0;
conv_opt = [];
%%
%%Main Loop
while(t<=time)
    while(numel(J(1,:))>0)
        del_tau = zeros(n);
        for k = 1:1:m
            Sum = 0;
            for j = 1:1:numel(J(k,:))
               Sum = Sum + (tau(ant(k),J(k,j))^alpha) * (1/d(ant(k),J(k,j))^beta); 
            end
            for j = 1:1:numel(J(k,:));
                p(ant(k),j) = ((tau(ant(k),J(k,j))^alpha)*(1/d(ant(k),J(k,j))^beta))/(Sum);
            end
            P = cumsum(p(ant(k),:));
            num = rand;
            q = min(find(P>num));
            del_tau(ant(k,1),J(k,q)) = del_tau(ant(k,1),J(k,q))+Q/d(ant(k,1),J(k,q));
            ant(k,1) = J(k,q);    
        end
        T = [T ant];
        J = [];
        for k = 1:1:m
            J(k,:) = setdiff(S,T(k,:));
        end
    end
    T1 = [T T(:,1)];
    C = zeros(n,1);
    for i = 1:1:numel(T(:,1))
        for j = 1:1:numel(T1(1,:))-1
            C(i) = C(i)+d(T1(i,j),T1(i,j+1));
        end
    end
    
    [q,ind] = min(C);
     if q < L_opt
         L_opt = q;
         T_opt = T1(ind,:);
     end
     
     conv_opt = [conv_opt L_opt];
     
     del_tau_e = Q/L_opt;
     
     for i = 1:1:n
         for j = 1:1:n
             tau(i,j) = (1-rho)*tau(i,j)+del_tau(i,j)+e*del_tau_e;
         end
     end
     %disp(L_opt);
%      disp(T_opt);
     T = [];
     J = [];
     for k = 1:1:m
        ant(k,1) = k;
        T(k,:) = ant(k);
        J(k,:) = setdiff(S,T(k,:));
     end
     t = t+1;
     X = city(T_opt(:),1);
     Y = city(T_opt(:),2);
%      figure(1)
%      plot(X,Y,'r');
%      hold on;
%      plot(city(:,1),city(:,2),'bo');
%      pause(0.1);
     %clf;
     %pause(1);
%      figure(2)
%      plot(t,L_opt,'bo');
%      title('convergence');
%      xlabel('t');
%      ylabel('shortest distance');
%      hold on;
end
%%
% figure(2)
% plot(conv_opt);
% figure(3)
% plot(X,Y,'r');
% hold on;
% plot(city(:,1),city(:,2),'bo');
% title('Shortest Path');
% xlabel('X');
% ylabel('Y');
out.WPorder = [city(T_opt(:),1) city(T_opt(:),2)];
out.tourLength = L_opt;
end
%%
