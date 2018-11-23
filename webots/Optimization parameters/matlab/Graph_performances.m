clc
clear all 
close all

addpath("../webots/controllers/flock_super2")

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
    %nb_simul = max(performances(:,1)); 
    %nb_steps_per_simul = sum(performances(:,1)==nb_simul); 
    nb_steps_per_simul = max(performances(:,1)); 
    nb_simul = sum(performances(:,1)==nb_steps_per_simul); 
end

perf_boxplot = zeros(nb_simul,1+nb_metrics*nb_steps_per_simul);

nb_simul_real = 1; c = 1;
syms k x; set_cumul = symsum(k,k,1,nb_steps_per_simul);    

%Reorganise data so it can realize the boxplot
while(c<=M)
    %fprintf('c value : %d\n',c);
%     somme = sum(performances(c:c+nb_steps_per_simul -1, 1))
%     theorie = nb_steps_per_simul * performances(c,1)
    %if(sum(performances(c:c+nb_steps_per_simul -1, 1))==nb_steps_per_simul*performances(c,1))
      if(sum(performances(c:c+nb_steps_per_simul -1, 1))==set_cumul && performances(c,1) == 1)
          perf_boxplot(nb_simul_real,:)=[performances(c,1), performances(c:c+nb_steps_per_simul -1, 3)'...
                                performances(c:c+nb_steps_per_simul -1, 4)'...
                                performances(c:c+nb_steps_per_simul -1, 5)'...
                                performances(c:c+nb_steps_per_simul -1, 6)'];
        c = c + nb_steps_per_simul;
        nb_simul_real = nb_simul_real + 1; 
      else
        %fprintf('not enough steps for a simulation at line %d\n', c);
        k = 1;
        while(performances(c+k,1)~=1 && k<nb_steps_per_simul)
            k = k+1;
        end
        if(k>nb_steps_per_simul-1)
            fprintf('2 lines will be problematic\n');
            fprintf('line %d, simul n° %d\n', c, nb_simul_real);
        end
        %nb_lines_to_jump = k
        perf_boxplot(nb_simul_real,:) = [performances(c,1), zeros(1, nb_metrics*nb_steps_per_simul)]; 
        nb_simul_real = nb_simul_real + 1; 
        c = c+k; 
    end
end

nb_simul_real = nb_simul_real -1;
% comp_mean = [ones(nb_simul_real,1), zeros(nb_simul_real, nb_metrics)];
% comp_std = [ones(nb_simul_real,1), zeros(nb_simul_real, nb_metrics)];
comp_mean = [zeros(nb_simul_real, nb_metrics)];
comp_std = [zeros(nb_simul_real, nb_metrics)];

%Boxplot of every metrics depending on the parameters of the simulation
for(i=2:nb_steps_per_simul:nb_metrics*nb_steps_per_simul+1)
    j = floor(i/nb_steps_per_simul)+2; %to obtain indice for data_label ref
    comp_mean(:,j-1)= mean(perf_boxplot(:,i:i+nb_steps_per_simul -1) ,2);
    comp_std(:,j-1) = std(perf_boxplot(:,i:i+nb_steps_per_simul -1),0,2);
    
%     figure; 
%     boxplot(perf_boxplot(:,i:i+nb_steps_per_simul -1)');
%     ylabel(data_label(j)); 
%     xlabel('Simulation number');
end

%Comparison of performances obtained
[best_mean_value, I_mean] = max(comp_mean,[],1);

comp_std_nonzero = comp_std; 
k = find(comp_std == 0); 
comp_std_nonzero(k) = 2; 

[best_std, I_std] = min(comp_std_nonzero,[],1);
best_perf_boxplot = zeros(nb_metrics,nb_metrics*nb_steps_per_simul, 2);

for(i=1:4)
    fprintf("Best mean value for %s is simulation %d and smallest std is simulation %d\n",...
    data_label(i+2), I_mean(i), I_std(i));
    best_perf_boxplot(i,:,1) = perf_boxplot(I_mean(i), 2:1+nb_metrics*nb_steps_per_simul);
    best_perf_boxplot(i,:,2) = perf_boxplot(I_std(i), 2:1+nb_metrics*nb_steps_per_simul);
end
%Plot the box plot related to the best performances obtained
figure;
title('Best mean value for');
ns=1;
for(j=1:2)
    for(i=1:4)
        subplot(2,4,ns);
        c = nb_steps_per_simul * (i-1) + 1; 
        %g = sprintf("Sim%d",I_mean(1), "Sim%d",I_mean(2), "Sim%d",I_mean(3), "Sim%d",I_mean(4)];
        boxplot(best_perf_boxplot(:,c:c+nb_steps_per_simul -1,j)');
        if(j==1)
            xticklabels({'S.805','S.903','S.109','S.805'});
        elseif(j==2)
            xticklabels({'S.505','S.714','S.103','S.102'});
        end
        %title(['Subplot' data_label(i+2)]);
        title({data_label(i+2)});
        ns = ns+1; 
    end
%title(['Temperature is ',num2str(c),' C'])
end
 sgtitle('Top : Mean value maximization. Down : Std minimization');


fclose('all');