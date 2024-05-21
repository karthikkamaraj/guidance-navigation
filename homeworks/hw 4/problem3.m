function pursuit_simulation_problem_3()
    % Define initial positions and speeds
    xE0 = 100; % Evader initial x position
    yE0 = 0;   % Evader initial y position
    V_E = 5;   % Evader constant speed
    VP = 6;    % Pursuer speed
    
    lambda_values = [0.25, 0.5, 0.75, 0.9, 1, 2, 5, 50]; % Values of lambda to simulate
    
    % Define simulation time span
    t_span = [0, 100];
    
    % Iterate over different lambda values
    for lambda = lambda_values
        % Define initial state vector for PP
        initial_state_PP = [xE0, yE0, 0, 0, atan2(yE0, xE0), hypot(xE0, yE0)];
        
        % Set options for the ODE solver
        options = odeset('Events', @(t, y) eventFunc(t, y), 'RelTol', 1e-6, 'AbsTol', 1e-6);
        
        % Solve PP differential equations using ode45
        [~, state_PP] = ode45(@(t, y) PP_ode(t, y, V_E, VP, lambda), t_span, initial_state_PP, options);
        
        % Extract PP states
        xP_PP = state_PP(:, 3);
        yP_PP = state_PP(:, 4);
        
        % Plot trajectories and R vs time for PP for lambda < 1
        if lambda < 1
            figure;
            plot(xP_PP, yP_PP);
            xlabel('x position');
            ylabel('y position');
            title(['Trajectory for PP, Lambda = ' num2str(lambda)]);
            
            figure;
            plot(state_PP(:, end));
            xlabel('Time (s)');
            ylabel('Range R');
            title(['Range R vs Time for PP, Lambda = ' num2str(lambda)]);
            continue;
        end
        
        % For lambda >= 1, compare CBP and PP trajectories and R vs time
        % Solve CBP and obtain initial heading theta by solving an equation
        initial_theta = atan2(yE0 - yP_PP(1), xE0 - xP_PP(1));
        
        % Define initial state vector for CBP
        initial_state_CBP = [xE0, yE0, xP_PP(1), yP_PP(1), atan2(yE0 - yP_PP(1), xE0 - xP_PP(1)), hypot(xE0 - xP_PP(1), yE0 - yP_PP(1)))];
        
        % Solve CBP differential equations using ode45
        [~, state_CBP] = ode45(@(t, y) CBP_ode(t, y, V_E, VP), t_span, initial_state_CBP, options);
        
        % Extract CBP states
        xP_CBP = state_CBP(:, 3);
        yP_CBP = state_CBP(:, 4);
        
        % Plot trajectories and R vs time for CBP and PP for lambda >= 1
        figure;
        plot(xP_CBP, yP_CBP, 'r', xP_PP, yP_PP, 'b');
        xlabel('x position');
        ylabel('y position');
        legend('CBP', 'PP');
        title(['Trajectories for CBP and PP, Lambda = ' num2str(lambda)]);
        
        figure;
        plot(state_CBP(:, end), 'r', state_PP(:, end), 'b');
        xlabel('Time (s)');
        ylabel('Range R');
        legend('CBP', 'PP');
        title(['Range R vs Time for CBP and PP, Lambda = ' num2str(lambda)]);
    end
end

function dydt = PP_ode(~, y, V_E, VP, lambda)
    % Extract states for PP
    xE = y(1);
    yE = y(2);
    xP = y(3);
    yP = y(4);
    beta = y(5);
    R = y(6);
    
    % Differential equations for PP
    dydt = zeros(6, 1);
    dydt(1) = 0;
    dydt(2) = V_E;
    dydt(3) = VP * cos(beta * (1 + lambda));
    dydt(4) = VP * sin(beta * (1 + lambda));
    dydt(5) = (VP * sin(beta * (1 + lambda)) + V_E * cos(beta)) / R;
    dydt(6) = V_E * sin(beta) - VP * cos(beta * (1 + lambda));
end

function [value, isterminal, direction] = eventFunc(~, y)
    % Event function to stop simulation when R = 0
    value = y(6); % R
    isterminal = 1; % Stop the integration
    direction = -1; % Negative direction
end

function dydt = CBP_ode(~, y, V_E, VP)
    % Extract states for CBP
    xE = y(1);
    yE = y(2);
    xP = y(3);
    yP = y(4);
    beta = atan2(yE - yP, xE - xP);
    R = hypot(xE - xP, yE - yP);
    
    % Differential equations for CBP
    dydt = zeros(6, 1);
    dydt(1) = 0;
    dydt(2) = V_E;
    dydt(3) = VP * cos(beta);
    dydt(4) = VP * sin(beta);
    dydt(5) = (VP * sin(beta) + V_E * cos(beta)) / R;
    dydt(6) = V_E * sin(beta) - VP * cos(beta);
end
