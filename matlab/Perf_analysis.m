%%
%............Matlab optimisation program, to find the set of parameters
%optimising both of the scenarios (crossing and obstacle world).........%
%%
clc; clear all; close all

plot = 0; print = 0; nb_scenarios = 2; nb_metrics = 4;    
data_label = ["nb_simul","time","orientation","cohesion","velocity",...
            "performance"];
%%
%......................Read the performance csv files ....................%
%%
addpath("crossing/ratio01_5"); 
Reynolds_cross = -1; errmsg = ''; 
while Reynolds_cross < 0
    disp(errmsg); 
    [Reynolds_cross, errmsg] = fopen('Reynolds_performance_cross.csv', 'r'); 
    performances_cross = csvread('Reynolds_performance_cross.csv');
end
fclose('all');

addpath("obstacles/ratio01_5");
Reynolds_obst = -1; errmsg = ''; 
while Reynolds_obst < 0
    disp(errmsg); 
    [Reynolds_obst, errmsg] = fopen('Reynolds_performance_obst.csv', 'r'); 
    performances_obst = csvread('Reynolds_performance_obst.csv');
end
fclose('all');


%%
%..............Compare for both scenarios in the same time................% 
%%
[cross_sum, lines_cross] = analyse(performances_cross, data_label,nb_metrics, plot, print); 
[obst_sum, lines_obst] = analyse(performances_obst, data_label,nb_metrics, plot, print); 

%Sum both scenarios results for instant performance sun
combine = cross_sum + obst_sum;

nb_compr = 5;
[highest_sum, I_sum] = sort(combine, 'descend'); 
fprintf("Highest sum of instant performance combining both scenarios are : \n")
for(i=1:nb_compr)
    fprintf("%d for simulation %d \n",highest_sum(i,1), I_sum(i));
end

figure; 
ns = 1; 
for(i=1:nb_scenarios)
    for(j=1:nb_metrics)
        bx1 = performances_obst(lines_obst(I_sum(1),1) +1:lines_obst(I_sum(1),2), 2+j); %+1 to remove start line
        bx2 = performances_obst(lines_obst(I_sum(2),1) +1:lines_obst(I_sum(2),2), 2+j); 
        bx3 = performances_obst(lines_obst(I_sum(3),1) +1:lines_obst(I_sum(3),2), 2+j); 
        bx4 = performances_obst(lines_obst(I_sum(4),1) +1:lines_obst(I_sum(4),2), 2+j); 
        bx5 = performances_obst(lines_obst(I_sum(5),1) +1:lines_obst(I_sum(5),2), 2+j); 
        bx=[bx1; bx2; bx3; bx4; bx5]; 
        bg = [zeros(length(bx1), 1); ones(length(bx2), 1); ...
            2*ones(length(bx3), 1); 3*ones(length(bx4),1); 4*ones(length(bx5),1)];

        subplot(2,4,ns);
        boxplot(bx, bg);
        xticklabels({'367','967','376','200', '826'});
        xlabel('Simulation n°');    ylabel('Performance');
        title({data_label(j+2)},'FontSize', 20);
        xt = get(gca, 'XTick'); set(gca, 'FontSize', 16);
        yt = get(gcb, 'YTick'); set(gcb, 'FontSize', 19);
        ns = ns+1; 
	end
end


for(i=1:nb_metrics)
    p_obst(:,i) = performances_obst(lines_obst(I_sum(1),1) +1:lines_obst(I_sum(1),2), 2+i);
    p_cross(:,i) = performances_cross(lines_cross(I_sum(1),1) +1:lines_cross(I_sum(1),2), 2+i);
end
figure; 
boxplot(p_obst);
xticklabels({"orientation","cohesion","velocity", "performance"});
xlabel('Metrics','FontSize', 20); ylabel('Performances','FontSize', 20);
title('Bests parameters performance for Obstacle','FontSize', 20);

figure; 
boxplot(p_cross);
xticklabels({"orientation","cohesion","velocity", "performance"});
xlabel('Metrics','FontSize', 20); ylabel('Performances','FontSize', 20);
title('Bests parameters performance for Crossing','FontSize', 20);

fclose('all');