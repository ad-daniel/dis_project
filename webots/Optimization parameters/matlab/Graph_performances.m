clc
clear all 
close all

addpath("../webots/controllers/flock_super")

Reynolds_perf = -1; errmsg = ''; 
data_label = ["nb_simul","time","orientation","cohesion","velocity",...
            "performance"];
nb_metrics = 4;     %TO DO : change so it takes it directly from performances
nb_steps_per_simul = 0; 
nb_simul = 0; 

while Reynolds_perf < 0
    disp(errmsg); 
    [Reynolds_perf, errmsg] = fopen('Reynolds_performance.csv', 'r'); 
    performances = csvread('Reynolds_performance.csv');
    [M,N]=size(performances); 
    nb_simul = max(performances(:,1)); 
    nb_steps_per_simul = sum(performances(:,1)==nb_simul); 
end

perf_boxplot = zeros(nb_simul,M+1);
comp_mean = [ones(nb_simul,1), zeros(nb_simul, nb_metrics)];
comp_std = [ones(nb_simul,1), zeros(nb_simul, nb_metrics)];
simul = 1; c = 1;
    
%Reorganise data so it can realize the boxplot
while(c<=M)
%     somme = sum(performances(c:c+nb_steps_per_simul -1, 1))
%     theorie = nb_steps_per_simul * performances(c,1)
    if(sum(performances(c:c+nb_steps_per_simul -1, 1))==nb_steps_per_simul*performances(c,1))
        perf_boxplot(simul,:)=[performances(c,1), performances(c:c+nb_steps_per_simul -1, 3)'...
                                performances(c:c+nb_steps_per_simul -1, 4)'...
                                performances(c:c+nb_steps_per_simul -1, 5)'...
                                performances(c:c+nb_steps_per_simul -1, 6)'];
        c = c + nb_steps_per_simul;
        simul = simul + 1; 
    else
        fprintf('not enough steps for a simulation\n');
        perf_boxplot(simul,:) = [performances(c,1), zeros(1, nb_metrics*nb_steps_per_simul)]; 
        c = c+1; 
        break; 
    end
end



%Boxplot of every metrics depending on the parameters of the simulation
for(i=2:nb_steps_per_simul:M+1)
    j = floor(i/nb_steps_per_simul)+2; %to obtain indice for data_label ref
    comp_mean(:,j)= mean(perf_boxplot(:,i:i+nb_steps_per_simul -1) ,2);
    comp_std(:,j) = std(perf_boxplot(:,i:i+nb_steps_per_simul -1),0,2);
    
    figure; 
    boxplot(perf_boxplot(:,i:i+nb_steps_per_simul -1)', perf_boxplot(:,1));  %passer en 3 dimensions plus simple?
    ylabel(data_label(j)); 
    xlabel(data_label(2));
end

%Comparison of performances obtained
[best_mean_value, I_mean] = max(comp_mean,[],1);
[best_std, I_std] = min(comp_std,[],1);

for(i=1:4)
    fprintf("Best mean value for %s is simulation %d and smallest std is simulation %d\n",...
            data_label(i+2), I_mean(i), I_std(i));
end

fclose('all');