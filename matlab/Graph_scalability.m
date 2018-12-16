clc
clear all
close all

%Crossing

nb_robots = [2,3,4,5]';
nb_metrics = 4; x = nb_robots; 
perf_cross = [0.788193, 0.887238, 0.068584, 0.057606; ...
            0.979235, 0.928523, 0.134568, 0.125135; ... 
            0.914534, 0.883461, 0.113588, 0.094751;...
            0.957077, 0.899799, 0.141946, 0.125259]; 
figure; 

plot(x,perf_cross(:,1),'-o',x,perf_cross(:,2),'--o',x,perf_cross(:,3),'-ro', x, perf_cross(:,4), '-.o');
legend({"orientation","cohesion","velocity", "performance"});
%xticklabels({nb_robots});
xticks([nb_robots]);
xlabel('N° of robots','FontSize', 20); ylabel('Performances','FontSize', 20);
title('Scalability for crossing world','FontSize', 20);
grid on; 

%Obstacle
nb_robots = [2,3,4,5,6,7,8]'; x = nb_robots;
perf_obst = [0.988339, 0.940684, 0.124158, 0.116267; ....
            0.925399, 0.889604, 0.129261, 0.111570; ...
            0.934368, 0.827728, 0.150277, 0.111118; 
            0.960811, 0.885561, 0.139252, 0.12136; ...
            0.885749, 0.854659, 0.122261, 0.097712; ...
            0.793987, 0.782480, 0.119621, 0.083951; ...
            0.930565, 0.725797, 0.126715, 0.086392];
figure; 
plot(x,perf_obst(:,1),'-o',x,perf_obst(:,2),'--o',x,perf_obst(:,3),'-ro', x, perf_obst(:,4), '-.o');
legend({"orientation","cohesion","velocity", "performance"});
xticks([nb_robots]);
xlabel('N° of robots','FontSize', 20); ylabel('Performances','FontSize', 20);
title('Scalability for obstacle world','FontSize', 20);
grid on; 