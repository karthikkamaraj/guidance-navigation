function pursuit_simulation_problem_5()
    % Define initial positions and speeds
    xE0 = 100; % Evader initial x position
    yE0 = 0;   % Evader initial y position
    V_E = 5;   % Evader constant speed
    VP = 6;    % Pursuer speed
    lambda_values = [1.5, 2, 2.5]; % Values of lambda to simulate
    
    % Define simulation time span
    t_span = [0, 100];
    
    % Iterate over different lambda values
    for lambda = lambda_values
        % Define initial state vector for CBP and PP
        initial_state_CBP = [xE0, yE0, xE0, yE0, 0, hypot(xE0, yE0)];
        initial_state_PP = initial_state_CBP; % Same initial state for PP
        
        % Arrays to store states over time
        states_CBP = [];
        states_PP = [];
        
        % Time vector
        t = linspace(t_span(1), t_span(2), 10000);
        
        % Calculate theta_E at each time step based on the modified initial bearing
        theta_E = pi/2 + cos(t);
        
        % Loop through time steps
        for i = 1:length(t)
            % Solve CBP differential equations using ode45
            [~, state_CBP] = ode45(@(t, y) CBP_ode(t, y, V_E, VP, theta_E(i)), [t(i), t(i) + 0.01], initial_state_CBP);
            
            % Solve PP differential equations using ode45
            [~, state_PP] = ode45(@(t, y) PP_ode(t, y, V_E, VP, lambda, theta_E(i)), [t(i), t(i) + 0.01], initial_state_PP);
            
            % Store final state for CBP and PP at this time step
            states_CBP = [states_CBP; state_CBP(end, :)];
            states_PP = [states_PP; state_PP(end, :)];
            
            % Update initial state for the next time step
            initial_state_CBP = state_CBP(end, :);
            initial_state_PP = state_PP(end, :);
        end
        
        % Extract CBP and PP states
        xP_CBP = states_CBP(:, 3);
        yP_CBP = states_CBP(:, 4);
        xP_PP = states_PP(:, 3);
        yP_PP = states_PP(:, 4);
        R_CBP = states_CBP(:, 6);
        R_PP = states_PP(:, 6);
        
        % Plot trajectories for CBP and PP
        figure;
        plot(xP_CBP, yP_CBP, 'r', xP_PP, yP_PP, 'b');
        xlabel('x position');
        ylabel('y position');
        legend('CBP', 'PP');
        title(['Trajectories for CBP and PP, Lambda = ' num2str(lambda)]);
        
        % Plot R vs time for CBP and PP
        figure;
        plot(R_CBP, 'r', R_PP, 'b');
        xlabel('Time steps');
        ylabel('Range R');
        legend('CBP', 'PP');
        title(['Range R vs Time for CBP and PP, Lambda = ' num2str(lambda)]);
    end
end

function dydt = CBP_ode(~, y, V_E, VP, theta_E)
    % Extract states for CBP
    xE = y(1);
    yE = y(2);
    xP = y(3);
    yP = y(4);
    beta = atan2(yE - yP, xE - xP);
    R = hypot(xE - xP, yE - yP);
    
    % Differential equations for CBP
    dydt = zeros(6, 1);
    dydt(1) = V_E * cos(theta_E);
    dydt(2) = V_E * sin(theta_E);
    dydt(3) = VP * cos(beta);
    dydt(4) = VP * sin(beta);
    dydt(5) = 0; % Constant bearing pursuit, so no change in beta
    dydt(6) = hypot(xE - xP, yE - yP); % Range R
end

function dydt = PP_ode(~, y, V_E, VP, lambda, theta_E)
    % Extract states for PP
    xE = y(1);
    yE = y(2);
    xP = y(3);
    yP = y(4);
    beta = atan2(yE - yP, xE - xP);
    R = hypot(xE - xP, yE - yP);
    
    % Differential equations for PP
    dydt = zeros(6, 1);
    dydt(1) = V_E * cos(theta_E);
    dydt(2) = V_E * sin(theta_E);
    dydt(3) = VP * cos(beta * (1 + lambda));
    dydt(4) = VP * sin(beta * (1 + lambda));
    dydt(5) = 0; % Constant bearing pursuit, so no change in beta
    dydt(6) = hypot(xE - xP, yE - yP); % Range R
end
