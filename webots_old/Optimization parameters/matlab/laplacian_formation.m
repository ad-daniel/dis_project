function laplacian_formation


    clear all;
    close all;
    %clc;
    deltaT = 0.08;
    threshold = 0.05;
    
    VGx = 0.0;  % goal velocity X
    VGy = 0.0; % goal velocity Y

    I = [-1  0  0  0 
          1 -1  0  0
          0  1 -1 -1
          0  0  1  0
          0  0  0  1];

    W = 0.5*(1.0) * eye(size(I,2)); % weight of edges in the graph

    x=[0,1,-1,0,1]'; %Initial positions X
    y=[4,3,2,1,0]'; %Initial positions Y


    bx = [0,2,6,8,8]';  % The desired X biases of the agents from the center of formation
    by = [4,2,2,4,0]'; % The desired Y biases of the agents from the center of formation
    %bx = [0,0,0,0,0]';  % The desired X biases of the agents from the center of formation
    %by = [0,0,0,0,0]'; % The desired Y biases of the agents from the center of formation

    L=I*W*I'; % computing the Laplace matrix  


 
    X= [x y];
    
    b = [bx by];

    hold on;
    grid on;
    %axis([-5,5,-5,5]);


    
    step=0;
    X_next = X;
    converged = 0;
    while (~converged)
        X = X_next;
        X_next = X + (-1)*L*(X-b) * deltaT;  % Laplacian feedback control

        X_next(:,1) = X_next(:,1) + deltaT*VGx; % constant velocity twards X-axis
        X_next(:,2) = X_next(:,2) + deltaT*VGy; % constant velocity twards Y-axis

        plot_agents(deltaT, step, X_next);
        step = step + 1;
        %if (max(abs(X_next-X))<threshold)
        %    converged =1;
        %end
        pause(0.1);
    end
    display('final position of agents');
    display(X);

    display('execution time:');
    display((step-2)*deltaT);
    
end


function plot_agents(stepsize, step, X)   
    plot(X(1,1),X(1,2),'vr','LineWidth',2);
    plot(X(2,1),X(2,2),'vg','LineWidth',2);
    plot(X(3,1),X(3,2),'vk','LineWidth',2);
    plot(X(4,1),X(4,2),'vb','LineWidth',2);
    plot(X(5,1),X(5,2),'vc','LineWidth',2);
    
        str1 = strcat('Time: ' , num2str((step-1)*stepsize,'%6.4g'), ' s');
    delete(findall(gcf,'Tag','Timetext'));
    annotationPos = [.12 0 0.6 0.3];
    htxtbox = annotation('textbox',annotationPos, ...
    'String'     ,str1, ...
    'FontSize'   ,14, ...
    'FitBoxToText', 'on', ...
    'EdgeColor', 'none', ...
    'FontName'   , 'Times', ...    
    'color','b', ...
    'Tag' , 'Timetext');

end

