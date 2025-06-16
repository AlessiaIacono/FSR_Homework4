clc;
clear;
close all;

%% 
set(groot, 'defaultFigureColor', 'w');  
set(groot, 'defaultAxesFontSize', 12); 
set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
blue = [0.000, 0.447, 0.741];   
orange = [0.851, 0.325, 0.098]; 
green = [0.466, 0.674, 0.188];
red = [0.850, 0.325, 0.098];

%% CONSTANT PARAMETERS
g = 9.81;             % gravity
thetadot0 = 0.95;     % initial angular velocity 
tf = 50;
dt = 0.01;

%% VARIATION OF LEG LENGTH 
alpha = pi/8;
gamma = 0.08;

l_values = 0.5:0.05:2.0;
theta_eq = zeros(size(l_values));

for i = 1:length(l_values)
    l = l_values(i);
    double_support = 0;

    if thetadot0 >= 0
        theta0 = gamma - alpha;
    else
        theta0 = gamma + alpha;
    end
    y0 = [theta0; thetadot0];

    t0 = 0;
    while t0 < tf
        options = odeset('Events', @(t,y) impact_event(t, y, alpha, gamma), 'MaxStep', dt);
        [~, y, te, ye, ~] = ode45(@(t,y) dynamics(t, y, g, l, double_support), [t0 tf], y0, options);

        if isempty(te)
            break;
        end
        [y0, double_support] = impact_map(ye, alpha, g, l);
        t0 = te;
        if double_support
            break;
        end
    end

    theta_eq(i) = double_support * y0(1);  % if double support, store theta; else 0
end

figure;
plot(l_values, theta_eq, 'o-', 'LineWidth', 1.5, 'Color', blue);
xlabel('$l$ (m)', 'Interpreter','latex');
ylabel('$\theta_{eq}$ (rad)', 'Interpreter','latex');
title('Variation of leg length $l$', 'Interpreter','latex');
grid on;
hold on;
yline(gamma + alpha, '--','DisplayName', '$\gamma + \alpha$', 'Interpreter','latex', 'Color', red, 'LineWidth', 1);
yline(gamma - alpha, '--','DisplayName', '$\gamma - \alpha$', 'Interpreter','latex', 'Color', green, 'LineWidth', 1);
legend show;

%% VARIATION OF ALPHA 
l = 1;
alpha_vals = linspace(pi/20, pi/4, 20);
theta_eq = zeros(size(alpha_vals));

for i = 1:length(alpha_vals)
    alpha = alpha_vals(i);
    double_support = 0;

    if thetadot0 >= 0
        theta0 = gamma - alpha;
    else
        theta0 = gamma + alpha;
    end
    y0 = [theta0; thetadot0];

    t0 = 0;
    while t0 < tf
        options = odeset('Events', @(t,y) impact_event(t, y, alpha, gamma), 'MaxStep', dt);
        [~, y, te, ye, ~] = ode45(@(t,y) dynamics(t, y, g, l, double_support), [t0 tf], y0, options);

        if isempty(te)
            break;
        end
        [y0, double_support] = impact_map(ye, alpha, g, l);
        t0 = te;
        if double_support
            break;
        end
    end

    theta_eq(i) = double_support * y0(1);
end

figure;
plot(alpha_vals, theta_eq, 'o-', 'LineWidth', 1.5, 'Color', blue);
xlabel('$\alpha$ (rad)', 'Interpreter','latex');
ylabel('$\theta_{eq}$ (rad)', 'Interpreter','latex');
title('Variation of inter-leg angle $\alpha$', 'Interpreter','latex');
grid on;
hold on;
plot(alpha_vals, gamma + alpha_vals, '--', 'LineWidth', 1.0, 'DisplayName', '$\gamma + \alpha$', 'Color',red);
plot(alpha_vals, gamma - alpha_vals, '--', 'LineWidth', 1.0, 'DisplayName', '$\gamma - \alpha$', 'Color', green);
legend show;



%% VARIATION OF GAMMA 
alpha = pi/8;
gamma_vals = 0.01:0.01:0.15;
theta_eq = zeros(size(gamma_vals));

for i = 1:length(gamma_vals)
    gamma = gamma_vals(i);
    double_support = 0;

    if thetadot0 >= 0
        theta0 = gamma - alpha;
    else
        theta0 = gamma + alpha;
    end
    y0 = [theta0; thetadot0];

    t0 = 0;
    while t0 < tf
        options = odeset('Events', @(t,y) impact_event(t, y, alpha, gamma), 'MaxStep', dt);
        [~, y, te, ye, ~] = ode45(@(t,y) dynamics(t, y, g, l, double_support), [t0 tf], y0, options);

        if isempty(te)
            break;
        end
        [y0, double_support] = impact_map(ye, alpha, g, l);
        t0 = te;
        if double_support
            break;
        end
    end

    theta_eq(i) = double_support * y0(1);
end

figure;
plot(gamma_vals, theta_eq, 'o-', 'LineWidth', 1.5, 'Color', blue);
xlabel('$\gamma$ (rad)', 'Interpreter','latex');
ylabel('$\theta_{eq}$ (rad)', 'Interpreter','latex');
title('Variation of slope angle $\gamma$', 'Interpreter','latex');
grid on;
hold on;
plot(gamma_vals, gamma_vals + alpha, '--', 'LineWidth', 1.0, 'DisplayName', '$\gamma + \alpha$', 'Color', red);
plot(gamma_vals, gamma_vals - alpha, '--', 'LineWidth', 1.0, 'DisplayName', '$\gamma - \alpha$', 'Color', green);
legend show;

%% EXAMPLES
% Length
l_values = [1.0, 1.5];  % one equilibrium, one limit cycle
alpha = pi/8;
gamma = 0.08;
thetadot0 = 0.95;

for i = 1:2
    l = l_values(i);
    if thetadot0 >= 0
        theta0 = gamma - alpha;
    else
        theta0 = gamma + alpha;
    end
    y0 = [theta0; thetadot0];
    t0 = 0;
    tf = 50;
    dt = 0.01;
    double_support = 0;
    
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

    figure;
    sgtitle(['Leg length $l$ = ', num2str(l)], 'Interpreter','latex');
    subplot(2,1,1);
    plot(T, Y(:,1), 'Color', blue, 'DisplayName', '$\theta(t)$');
    hold on;
    plot(T, Y(:,2), 'Color', orange, 'DisplayName', '$\dot{\theta}(t)$');
    xlabel('$t$ [s]', 'Interpreter','latex');
    ylabel('State', 'Interpreter','latex');
    grid on;
    title('Rimless Wheel Dynamics', 'Interpreter','latex');
    legend('Interpreter','latex');

    subplot(2,1,2);
    plot(Y(:,1), Y(:,2), 'Color', blue);
    hold on;
    plot(Y(1,1), Y(1,2), 'o', 'Color', orange, 'MarkerSize', 8, 'DisplayName', 'Initial point');
    xlabel('$\theta$ (rad)', 'Interpreter','latex');
    ylabel('$\dot{\theta}$ (rad/s)', 'Interpreter','latex');
    grid on;
    title('Rimless Wheel Limit Cycle','Interpreter','latex');
    legend('Interpreter','latex');
end

%Alpha 
l = 1;
alpha_values = [0.3, 0.7];
gamma = 0.08;

for i = 1:2
    alpha = alpha_values(i);
    if thetadot0 >= 0
        theta0 = gamma - alpha;
    else
        theta0 = gamma + alpha;
    end
    y0 = [theta0; thetadot0];
    t0 = 0; tf = 50; dt = 0.01;
    double_support = 0; T = []; Y = [];

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

    figure;
    sgtitle(['Inter-leg angle $\alpha$ = ', num2str(alpha, '%.2f')], 'Interpreter','latex');
    subplot(2,1,1);
    plot(T, Y(:,1), 'Color', blue, 'DisplayName', '$\theta(t)$');
    hold on;
    plot(T, Y(:,2), 'Color', orange, 'DisplayName', '$\dot{\theta}(t)$');
    xlabel('$t$ [s]','Interpreter','latex'); ylabel('State','Interpreter','latex');
    title('Rimless Wheel Dynamics', 'Interpreter','latex');
    legend('Interpreter','latex'); grid on;

    subplot(2,1,2);
    plot(Y(:,1), Y(:,2), 'Color', blue);
    hold on; plot(Y(1,1), Y(1,2), 'o', 'Color', orange, 'MarkerSize', 8, 'DisplayName', 'Initial point');
    xlabel('$\theta$ (rad)','Interpreter','latex'); ylabel('$\dot{\theta}$ (rad/s)','Interpreter','latex');
    title('Rimless Wheel Limit Cycle','Interpreter','latex'); legend('Interpreter','latex'); grid on;
end

% Gamma 
l = 1;
alpha = pi/8;
gamma_values = [0.05, 0.12];

for i = 1:2
    gamma = gamma_values(i);
    if thetadot0 >= 0
        theta0 = gamma - alpha;
    else
        theta0 = gamma + alpha;
    end
    y0 = [theta0; thetadot0];
    t0 = 0; tf = 50; dt = 0.01;
    double_support = 0; T = []; Y = [];

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

    figure;
    sgtitle(['Slope angle $\gamma$ = ', num2str(gamma, '%.2f')], 'Interpreter','latex');
    subplot(2,1,1);
    plot(T, Y(:,1), 'Color', blue, 'DisplayName', '$\theta(t)$');
    hold on;
    plot(T, Y(:,2), 'Color', orange, 'DisplayName', '$\dot{\theta}(t)$');
    xlabel('$t$ [s]','Interpreter','latex'); ylabel('State','Interpreter','latex');
    title(['Inter-leg angle $\alpha$ = ', num2str(alpha, '%.2f')], 'Interpreter','latex');
    legend('Interpreter','latex'); grid on;

    subplot(2,1,2);
    plot(Y(:,1), Y(:,2), 'Color', blue);
    hold on; plot(Y(1,1), Y(1,2), 'o', 'Color', orange, 'MarkerSize', 8, 'DisplayName', 'Initial point');
    xlabel('$\theta$ (rad)','Interpreter','latex'); ylabel('$\dot{\theta}$ (rad/s)','Interpreter','latex');
    title('Rimless Wheel Limit Cycle','Interpreter','latex'); legend('Interpreter','latex'); grid on;
end
%% 
function dydt = dynamics(~, y, g, l, ds)
    theta = y(1);
    thetadot = y(2);
    if ~ds
        dtheta = thetadot;
        dthetadot = (g/l)*sin(theta);
    else
        dtheta = 0;
        dthetadot = 0;
    end
    dydt = [dtheta; dthetadot];
end

function [value, isterminal, direction] = impact_event(~, y, alpha, gamma)
    value = [y(1)-alpha-gamma; y(1)-gamma+alpha];
    isterminal = [1;1];
    direction = [1;-1];
end

function [yplus, ds] = impact_map(y_minus, alpha, g, l)
    if y_minus(2) >= 0
        theta_plus = y_minus(1) - 2*alpha;
    else
        theta_plus = y_minus(1) + 2*alpha;
    end
    thetadot_plus = cos(2*alpha) * y_minus(2);
    if abs(thetadot_plus) < 0.01 * sqrt(g/l)
        thetadot_plus = 0;
        ds = 1;
    else
        ds = 0;
    end
    yplus = [theta_plus; thetadot_plus];
end
