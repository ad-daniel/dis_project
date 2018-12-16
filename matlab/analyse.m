function [norm_sum, lines_to_read] = analyse(performances, data_label,nb_metrics, plot, print)
%%
% Analyse datas in order to calculate the best set of parameters 
%%
[M,N]=size(performances);

%Find simulation start line
nb_steps_per_simul = max(performances(:,1)); 
nb_max_steps = nb_steps_per_simul; 
nb_simul = size( find(performances(:,3)==0.0000) ,1);
syms k x; set_cumul = symsum(k,k,1,nb_steps_per_simul);    count = 0; countB = 0; 

comp_mean = zeros(nb_simul, nb_metrics + 2); 
comp_std = zeros(nb_simul, nb_metrics + 2); 


%Reorganise data so it can realize the boxplot
nb_simul_real = 1; c = 1;
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
    
    min_sum = min(performances(c:cf,6), [],1); 
    max_sum = max(performances(c:cf,6), [],1); 
    sum_perf_inst(nb_simul_real, 1) = sum(performances(c:cf,6), 1); 
    norm_sum(nb_simul_real,1) = (sum(performances(c:cf,6), 1) - min_sum)/(max_sum - min_sum);
    
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

if(print)
    for(i=1:4)
        fprintf("Best mean value for %s is simulation %d and smallest std is simulation %d\n",...
        data_label(i+2), I_mean(i), I_std(i));
    end
end

for(j=1:4)
    start(j,:) = [comp_mean(I_mean(j),1), comp_std(I_std(j),1)];
    stop(j,:) = [comp_mean(I_mean(j),2), comp_std(I_std(j),2)];
end


%Plot the box plot related to the best performances obtained
if(plot) figure; end
ns=1;
for(j=1:2)
    for(i=1:4)
        %start at line + 1 to avoid 0 of the start line
        x1 = performances(start(1,j)+1:stop(1,j), 2+i);
        x2 = performances(start(2,j)+1:stop(2,j), 2+i);
        x3 = performances(start(3,j)+1:stop(3,j), 2+i);
        x4 = performances(start(4,j)+1:stop(4,j), 2+i);
        x=[x1; x2; x3; x4]; 
        g = [zeros(length(x1), 1); ones(length(x2), 1); ...
            2*ones(length(x3), 1); 3*ones(length(x4),1)];
        
        if(plot)
            subplot(2,4,ns);
            boxplot(x, g);

            if(j==1) 
                xticklabels({'479','24','774','994'});
            elseif(j==2)
                xticklabels({'146','491','32','11'});
            end
            xlabel('Simulation n°');    ylabel('Performance');
            title({data_label(i+2)},'FontSize', 20);
            xt = get(gca, 'XTick'); set(gca, 'FontSize', 19)
            yt = get(gcb, 'YTick'); set(gcb, 'FontSize', 19)
        end
        ns = ns+1; 
    end
end
if(plot) sgtitle('Boxplots of the best performances for our metrics, during the time of simulation'); end

%%
%..........Best compromise between orientation, cohesion and velocity ......%
%Normalization of the mean value, so compromised_mean between 0 to 3, as
%sum of 3 normalized metrics.
%%

nb_compr = 5;
compromise_mean = sum(comp_mean(:,3:5) ./ best_mean_value(1:3),2); %only take orient, cohesion, velocity
[compr_mean_sorted, I_sorted] = sort(compromise_mean, 'descend'); 
best_compromise_mean = [compr_mean_sorted(1:nb_compr,1), I_sorted(1:nb_compr,1)]; 

if(print) fprintf("\n Best compromise means are : \n"); end
for(j=1:nb_compr)
    bstart(j,1) = [comp_mean(I_sorted(j),1)];
    bstop(j,1) = [comp_mean(I_sorted(j),2)];
    if(print)
        fprintf("%d for simulation %d \n",best_compromise_mean(j), I_sorted(j));
    end
end


if(plot) figure; end
for(i=1:nb_metrics)
    bx1 = performances(bstart(1,1)+1:bstop(1,1), 2+i);
    bx2 = performances(bstart(2,1)+1:bstop(2,1), 2+i);
    bx3 = performances(bstart(3,1)+1:bstop(3,1), 2+i);
    bx4 = performances(bstart(4,1)+1:bstop(4,1), 2+i);
    bx5 = performances(bstart(5,1)+1:bstop(5,1), 2+i);
    bx=[bx1; bx2; bx3; bx4; bx5]; 
    bg = [zeros(length(bx1), 1); ones(length(bx2), 1); ...
        2*ones(length(bx3), 1); 3*ones(length(bx4),1); 4*ones(length(bx5),1)];
    
    if(plot)
        subplot(1,4,i);
        boxplot(bx, bg);
        xticklabels({'994','84','479','774', '588'});
        xlabel('Simulation n°');    ylabel('Performance');
        title({data_label(i+2)},'FontSize', 20);
        xt = get(gca, 'XTick'); set(gca, 'FontSize', 16)
        yt = get(gcb, 'YTick'); set(gcb, 'FontSize', 19)
    end
end
%sgtitle('Boxplots of the best COMPROMISES performances for our metrics, during the time of simulation'); 
%%
%............... Best sum of instant performances obtained ...............%
%Normalization of the mean value, so compromised_mean between 0 to 3, as
%sum of 3 normalized metrics.
%%

[highest_sum, I_highest_sum] = sort(sum_perf_inst, 'descend'); 
if(print) 
    fprintf("\n"); fprintf("Highest sum of instant performance are : \n");
    for(i=1:nb_compr)
        fprintf("%d for simulation %d \n",highest_sum(i,1), I_highest_sum(i));
    end
    fprintf("\n");
end

lines_to_read = [comp_mean(:, 1:2)];
end
%%