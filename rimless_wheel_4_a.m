clc
close all
clear all

%%
set(groot, 'defaultFigureColor', 'w');  
set(groot, 'defaultAxesFontSize', 12); 
set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
blue = [0.000, 0.447, 0.741];   
orange = [0.851, 0.325, 0.098]; 
yellow = [0.929, 0.694, 0.125];
green = [0.466, 0.674, 0.188];
purple = [0.494, 0.184, 0.556];

%%

% Parameters
g = 9.81;          % gravity (m/s^2)
l = 1.0;           % leg length (m) %%MODIFY HERE%% 
alpha = pi/8;      % half inter-leg angle (rad) %%MODIFY HERE%% 
gamma = 0.08;      % slope angle (rad) %%MODIFY HERE%% 


%% FIRST PART - ANALYSIS WITH DIFFERENT INITIAL VELOCITIES VALUES

% Initial conditions
thetadot0_list = [-10, -7, -5, -2, -1.5, -1, -0.5, 0.5, 0.95, 1, 1.5, 2, 2.5, 5, 7, 10 ];


for i = 1 : length(thetadot0_list)
    thetadot0 = thetadot0_list(i);
    
    if (thetadot0 >= 0)
        theta0_fixed = gamma-alpha;
    else
        theta0_fixed = gamma+alpha;
    end
    
    double_support = 0;
    
    y0 = [theta0_fixed; thetadot0];
    
    % Simulation settings
    t0 = 0; %initial time
    tf = 50; %final time
    dt = 0.01; %max step time
    
    % Time/state storage
    T = [];
    Y = [];
    
    while t0 < tf
        options = odeset('Events', @(t, y) impact_event(t, y, alpha,gamma), 'MaxStep', dt);
        [t, y, te, ye, ie] = ode45(@(t, y) dynamics(t, y, g, l, double_support), [t0 tf], y0, options);
        
        T = [T; t];
        Y = [Y; y];
    
        if ~isempty(te)
            [y0,double_support] = impact_map(ye, alpha,g,l); % apply impact map
            t0 = te;
        else
            break;
        end
    end
    
    % Plot results
    figure
    subplot(2,1,1)
    plot(T, Y(:,1), 'Color', blue, 'DisplayName', '${\theta}(t)$(rad)');
    hold on;
    plot(T, Y(:,2),'Color', orange, 'DisplayName', '$\dot{\theta}(t)$(rad/s)');
    xlabel('$t$ [s]');
    ylabel('State');
    %xlim([0,5]);
    title('Rimless Wheel Dynamics');
    legend show;
    grid on;

    subplot(2,1,2)
    plot(Y(:,1), Y(:,2), 'Color', blue, 'DisplayName', '${\theta}(t)$(rad)');
    hold on
    plot(Y(1,1), Y(1,2), 'Color', orange, 'Marker','*','LineWidth',5,'DisplayName','Initial point');
    xlabel('${\theta}(t)$(rad)');
    ylabel('$\dot{\theta}(t)$(rad/s)');
    title('Rimless Wheel Limit Cycle');
    legend show;
    grid on;

    sgtitle(sprintf('Simulation for $\\dot{\\theta}_0$ = %.2f rad/s', thetadot0), 'Interpreter', 'latex');

    v_end = abs(Y(end,2));
    if v_end < 0.05
       result = '→ Equilibrium'; 
    else
        result = '→ Limit Cycle';

    end

fprintf('thetadot0 = %.2f %s (|thetadot| = %.4f)\n', thetadot0, result, v_end);

end

%% SECOND PART - BASINS OF ATTRACTION
theta0_vals = linspace(gamma - alpha - 0.5, gamma + alpha + 0.5, 100);
thetadot0_vals = linspace(-10, 10, 100);


basin = zeros(length(theta0_vals), length(thetadot0_vals)); % 0 = ciclo, 1 = equilibrio


for i = 1:length(theta0_vals)
    for j = 1:length(thetadot0_vals)
        theta0 = theta0_vals(i);
        thetadot0 = thetadot0_vals(j);
        y0 = [theta0; thetadot0];

        t0 = 0;
        tf = 50;
        dt = 0.01;
        double_support = 0;

        while t0 < tf
            options = odeset('Events', @(t, y) impact_event(t, y, alpha, gamma), 'MaxStep', dt);
            [t, y, te, ye, ~] = ode45(@(t, y) dynamics(t, y, g, l, double_support), [t0 tf], y0, options);

            if isempty(te)
                break;
            end

            [y0, double_support] = impact_map(ye, alpha, g, l);
            t0 = te;

            if double_support
                break;
            end
        end

        if double_support
            basin(i,j) = 1;  % equilibrio
        else
            basin(i,j) = 0;  % limite ciclico
        end
    end
end


figure;
imagesc(thetadot0_vals, theta0_vals, basin);  % asse x = velocità, asse y = posizione
xlabel('$\dot{\theta}_0$ (rad/s)', 'Interpreter', 'latex');
ylabel('$\theta_0$ (rad)', 'Interpreter', 'latex');
title('Basins of Attraction', 'Interpreter', 'latex');
colormap([1 0.4 0.4; 0.4 0.7 1]);  % rosso = ciclo, blu = equilibrio
colorbar('Ticks',[0,1],'TickLabels',{'Limit Cycle','Equilibrium'});
axis xy; grid on;

%%

thetadot0_vals = linspace(-10, 10, 1000);
result_1D = zeros(size(thetadot0_vals));

for i = 1:length(thetadot0_vals)
    thetadot0 = thetadot0_vals(i);


    if (thetadot0 >= 0)
        theta0_fixed = gamma-alpha;
    else
        theta0_fixed = gamma+alpha;
    end

    y0 = [theta0_fixed; thetadot0];
    
    t0 = 0;
    double_support = 0;
    
    while t0 < tf
        options = odeset('Events', @(t, y) impact_event(t, y, alpha, gamma), 'MaxStep', dt);
        [~, y, te, ye, ~] = ode45(@(t, y) dynamics(t, y, g, l, double_support), [t0 tf], y0, options);
        
        if isempty(te)
            break;
        end
        
        [y0, double_support] = impact_map(ye, alpha, g, l);
        t0 = te;

        if double_support
            break;
        end
    end
    
    % Salva il risultato
    if double_support
        result_1D(i) = y0(1);  % equilibrio
    else
        result_1D(i) = 0;  % ciclo
    end
end

%%
sum_ = gamma + alpha;
diff_ = gamma - alpha;

label_sum  = sprintf('$\\gamma + \\alpha = %.4f$', sum_);
label_diff = sprintf('$\\gamma - \\alpha = %.4f$', diff_);

figure;
plot(thetadot0_vals, result_1D, 'b', 'LineWidth', 1.5);
xlabel('$\dot{\theta}_0$ (rad/s)', 'Interpreter', 'latex', 'FontSize', 12);
ylabel('$\theta_{eq}$ (rad)', 'Interpreter', 'latex', 'FontSize', 12);
title('1D Basin of attraction', 'Interpreter', 'latex');

yline(sum_, '--', label_sum, 'Interpreter', 'latex', 'Color', orange, 'LineWidth', 1);
yline(diff_, '--', label_diff, 'Interpreter', 'latex', 'Color', green, 'LineWidth', 1);

grid on;







%%
function dydt = dynamics(~, y, g, l, ds)
    theta = y(1);
    thetadot = y(2);
    if (~ds)
        dtheta = thetadot;
        dthetadot = (g/l) * sin(theta);
    else
        dtheta = 0;
        dthetadot = 0;
    end
    dydt = [dtheta; dthetadot];
end

function [value, isterminal, direction] = impact_event(~, y, alpha,gamma)
    
    value = [y(1)-alpha-gamma; y(1)-gamma+alpha];% Trigger when theta = gamma+alpha
                                     %Trigger when theta = gamma-alpha
    isterminal = [1;1];         % Stop the integration
    direction = [1;-1];          % Detect only when increasing
end

function [yplus,ds] = impact_map(y_minus, alpha,g,l)%minus: before impact time; plus: after impact time
    if (y_minus(2)>=0)
        theta_plus = y_minus(1)-2*alpha;
    else
        theta_plus = y_minus(1)+2*alpha;
    end
    thetadot_plus = cos(2*alpha) * y_minus(2);
    if (thetadot_plus < 0.01*sqrt(g/l) && thetadot_plus >-0.01*sqrt(g/l)) 
        thetadot_plus = 0;
        ds = 1;
    else
        ds = 0;
    end
    yplus = [theta_plus; thetadot_plus];
end