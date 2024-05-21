function pursuit_simulation_problem_6()
    % Define initial positions and speeds
    xE0 = 100; % Evader initial x position
    yE0 = 0;   % Evader initial y position
    V_E = 5;   % Evader constant speed
    VP = 6;    % Pursuer speed
    lambda_values = [1.5, 2, 2.5]; % Values of lambda to simulate
    Ts = 0.01; % Sampling period
    stop_condition = 0.5; % Stop condition for R
    
    % Define simulation time span
    t_span = [0, 100];
    
    % Time vector
    t = t_span(1):Ts:t_span(2);
    
    % Calculate theta_E at each time step based on the modified initial bearing
    theta_E = pi/2 + cos(t);
    
    % Iterate over different lambda values
    for lambda = lambda_values
        % Arrays to store states over time
        states_DP = zeros(length(t), 6);
        states_PP = zeros(length(t), 6);
        
        % Loop through time steps
        for i = 1:length(t)
            % Generate noisy bearing measurement
            beta_k = calculate_beta_k(t(i));
            w_k = randn() * 0.25;
            u = beta_k + w_k;
            
            % Solve DP differential equations using a single time step
            [~, state_DP] = ode45(@(t, y) DP_ode(t, y, V_E, VP, u), [t(i), t(i) + Ts], states_DP(i, :));
            
            % Solve PP differential equations using a single time step
            [~, state_PP] = ode45(@(t, y) PP_ode(t, y, V_E, VP, lambda, u), [t(i), t(i) + Ts], states_PP(i, :));
            
            % Store final state for DP and PP at this time step
            states_DP(i, :) = state_DP(end, :);
            states_PP(i, :) = state_PP(end, :);
            
            % Check stop condition for R and break loop if met
            if state_DP(end, 6) <= stop_condition || state_PP(end, 6) <= stop_condition
                break;
            end
        end
        
        % Extract DP and PP states
        R_DP = states_DP(:, 6);
        R_PP = states_PP(:, 6);
        
        % Plot R vs time for DP and PP
        figure;
        plot(R_DP, 'r', R_PP, 'b');
        xlabel('Time steps');
        ylabel('Range R');
        legend('DP', 'PP');
        title(['Range R vs Time for DP and PP, Lambda = ' num2str(lambda)]);
    end
end

function beta_k = calculate_beta_k(t)
    % Calculate beta_k for noisy bearing measurement
    beta_k = atan2(100 * sin(0.1 * t), 100 * cos(0.1 * t));
end

function dydt = DP_ode(~, y, V_E, VP, u)
    % Extract states for DP
    xE = 100; % Evader x position
    yE = 0;   % Evader y position
    xP = y(1);
    yP = y(2);
    beta = y(5);
    R = y(6);
    
    % Differential equations for DP
    dydt = zeros(6, 1);
    dydt(1) = VP * cos(u);
    dydt(2) = VP * sin(u);
    dydt(3) = V_E * cos(beta);
    dydt(4) = V_E * sin(beta);
    dydt(5) = (VP * sin(u) + V_E * cos(beta)) / R;
    dydt(6) = V_E * sin(beta) - VP * cos(u);
end

function dydt = PP_ode(~, y, V_E, VP, lambda, u)
    % Extract states for PP
    xE = 100; % Evader x position
    yE = 0;   % Evader y position
    xP = y(1);
    yP = y(2);
    beta = y(5);
    R = y(6);
    
    % Differential equations for PP
    dydt = zeros(6, 1);
    dydt(1) = VP * cos(u);
    dydt(2) = VP * sin(u);
    dydt(3) = V_E * cos(beta * (1 + lambda));
    dydt(4) = V_E * sin(beta * (1 + lambda));
    dydt(5) = (VP * sin(u) + V_E * cos(beta)) / R;
    dydt(6) = V_E * sin(beta) - VP * cos(u);
end
