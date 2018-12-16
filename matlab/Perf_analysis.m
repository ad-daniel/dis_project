clc
clear all 
close all

addpath("../webots/controllers/flock_super")
%addpath("ratio02_0")
%addpath("..")

Reynolds_perf = -1; errmsg = ''; 
data_label = ["nb_simul","time","orientation","cohesion","velocity",...
            "performance"];
nb_metrics = 4;     %TO DO : change so it takes it directly from performances
nb_steps_per_simul = 0; 
nb_simul = 0; 

while Reynolds_perf < 0
    disp(errmsg); 
    [Reynolds_perf, errmsg] = fopen('Reynolds_performance Ratio [5.000,1.000,40.000].csv', 'r'); 
    performances = csvread('Reynolds_performance Ratio [5.000,1.000,40.000].csv');
    [M,N]=size(performances);
    
    %Find line with 1 0 0 0 0 to know when a simulation starts
    nb_steps_per_simul = max(performances(:,1)); 
    nb_max_steps = nb_steps_per_simul; 
    nb_simul = size( find(performances(:,3)==0.0000) ,1);
end


nb_simul_real = 1; c = 1;
syms k x; set_cumul = symsum(k,k,1,nb_steps_per_simul);    count = 0; countB = 0; 

comp_mean = zeros(nb_simul, nb_metrics + 2); 
comp_std = zeros(nb_simul, nb_metrics + 2); 

%Reorganise data so it can realize the boxplot
while(c<M)
    if(performances(c,1) == 1)
        nb_line = 1; 
        while((performances(c+nb_line) ~= 1) && (nb_line < nb_max_steps))
            nb_line = nb_line + 1 ; 
        end
        
       if(c + nb_line -1 <= M)
           cf = c + nb_line -1 ;
       else
           cf = M; 
       end
    elseif(performances(c,1) ~= 1)
        cf = c; 
            while(performances(cf,1) ~= 1 && (cf<M))
                cf = cf + 1; 
            end
        if(cf>=M)
            cf = M; 
        end
        
        cf = cf-1;
    end
    
    comp_mean(nb_simul_real, 1:2) = [c, cf];
    comp_std(nb_simul_real, 1:2) = [c, cf];
        
    for(i=1:nb_metrics)
        comp_mean(nb_simul_real, 2+i) = mean(performances(c:cf,2+i));
        comp_std(nb_simul_real, 2+i) = std(performances(c:cf,2+i), 0,1);
    end
    
    c = cf +1;
    nb_simul_real = nb_simul_real + 1; 
end

nb_simul_real = nb_simul_real -1;
[best_mean_value, I_mean] = max(comp_mean(:,3:end),[],1);

%%
%............Best mean value of orientation, cohesion and velocity........%
%%
comp_std_nonzero = comp_std; 
k = find(comp_std == 0); 
comp_std_nonzero(k) = 2; 

[best_std, I_std] = min(comp_std_nonzero(:, 3:end),[],1);
%best_perf_boxplot = zeros(nb_metrics,nb_metrics*nb_steps_per_simul, 2);


for(i=1:4)
    fprintf("Best mean value for %s is simulation %d and smallest std is simulation %d\n",...
    data_label(i+2), I_mean(i), I_std(i));
end

for(j=1:4)
    start(j,:) = [comp_mean(I_mean(j),1), comp_std(I_std(j),1)];
    stop(j,:) = [comp_mean(I_mean(j),2), comp_std(I_std(j),2)];
end

%%
%..........Best compromise between orientation, cohesion and velocity ......%
%Normalization of the mean value, so compromised_mean between 0 to 3, as
%sum of 3 normalized metrics.
%%
%Plot the box plot related to the best performances obtained
figure;
ns=1;
for(j=1:2)
    for(i=1:4)
        subplot(2,4,ns);
        %start at line + 1 to avoid 0 of the start line
        x1 = performances(start(1,j)+1:stop(1,j), 2+i);
        x2 = performances(start(2,j)+1:stop(2,j), 2+i);
        x3 = performances(start(3,j)+1:stop(3,j), 2+i);
        x4 = performances(start(4,j)+1:stop(4,j), 2+i);
        x=[x1; x2; x3; x4]; 
        g = [zeros(length(x1), 1); ones(length(x2), 1); ...
            2*ones(length(x3), 1); 3*ones(length(x4),1)];
        boxplot(x, g);
        
        if(j==1) 
            xticklabels({'23','3','895','895'});
        elseif(j==2)
            xticklabels({'23','55','11','1'});
        end
        xlabel('Simulation n�');    ylabel('Performance');
        title({data_label(i+2)},'FontSize', 20);
        xt = get(gca, 'XTick'); set(gca, 'FontSize', 19)
        yt = get(gcb, 'YTick'); set(gcb, 'FontSize', 19)
        ns = ns+1; 
    end
end
sgtitle('Boxplots of the best performances for our metrics, during the time of simulation');


nb_compr = 5;
compromise_mean = sum(comp_mean(:,3:5) ./ best_mean_value(1:3),2); %only take orient, cohesion, velocity
[compr_mean_sorted, I_sorted] = sort(compromise_mean, 'descend'); 
best_compromise_mean = [compr_mean_sorted(1:nb_compr,1), I_sorted(1:nb_compr,1)]; 

fprintf("\n Best compromise means are : \n");
for(j=1:nb_compr)
    bstart(j,1) = [comp_mean(I_sorted(j),1)];
    bstop(j,1) = [comp_mean(I_sorted(j),2)];
    fprintf("%d for simulation %d \n",best_compromise_mean(j), I_sorted(j));
end

figure; 
for(i=1:4)
    subplot(1,4,i);
    bx1 = performances(bstart(1,1)+1:bstop(1,1), 2+i);
    bx2 = performances(bstart(2,1)+1:bstop(2,1), 2+i);
    bx3 = performances(bstart(3,1)+1:bstop(3,1), 2+i);
    bx4 = performances(bstart(4,1)+1:bstop(4,1), 2+i);
    bx5 = performances(bstart(5,1)+1:bstop(5,1), 2+i);
    bx=[bx1; bx2; bx3; bx4; bx5]; 
    bg = [zeros(length(bx1), 1); ones(length(bx2), 1); ...
        2*ones(length(bx3), 1); 3*ones(length(bx4),1); 4*ones(length(bx5),1)];
    boxplot(bx, bg);
    xticklabels({'895','860','987','400', '890'});
    xlabel('Simulation n�');    ylabel('Performance');
    title({data_label(i+2)},'FontSize', 20);
    xt = get(gca, 'XTick'); set(gca, 'FontSize', 16)
    yt = get(gcb, 'YTick'); set(gcb, 'FontSize', 19)
end
%sgtitle('Boxplots of the best COMPROMISES performances for our metrics, during the time of simulation'); 
%%
fclose('all');