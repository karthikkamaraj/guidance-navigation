function pursuit_simulation_problem_2()
    % Define initial positions and speeds
    xE0 = 100; % Evader initial x position
    yE0 = 0;   % Evader initial y position
    V_E = 5;   % Evader constant speed
    VP_values = [6, 8, 11]; % Pursuer speeds to simulate
    
    % Iterate over different VP values
    for VP = VP_values
        % Define simulation time span
        t_span = [0, 100];
        
        % Define initial state vector for DP
        initial_state_DP = [xE0, yE0, 0, 0, atan2(yE0, xE0), hypot(xE0, yE0)];
        
        % Solve DP differential equations using ode45
        [~, state_DP] = ode45(@(t, y) DP_ode(t, y, V_E, VP), t_span, initial_state_DP);
        
        % Extract DP states
        xP_DP = state_DP(:, 3);
        yP_DP = state_DP(:, 4);
        
        % Solve CBP and obtain initial heading theta by solving an equation
        initial_theta = atan2(yE0 - yP_DP(1), xE0 - xP_DP(1));
        
        % Define initial state vector for CBP
        initial_state_CBP = [xE0, yE0, xP_DP(1), yP_DP(1), atan2(yE0 - yP_DP(1), xE0 - xP_DP(1)), hypot(xE0 - xP_DP(1), yE0 - yP_DP(1)))];
        
        % Set options for the ODE solver
        options = odeset('Events', @(t, y) eventFunc(t, y), 'RelTol', 1e-6, 'AbsTol', 1e-6);
        
        % Solve CBP differential equations using ode45
        [~, state_CBP] = ode45(@(t, y) CBP_ode(t, y, V_E, VP), t_span, initial_state_CBP, options);
        
        % Extract CBP states
        xP_CBP = state_CBP(:, 3);
        yP_CBP = state_CBP(:, 4);
        
        % Plot trajectories and R vs time for DP and CBP
        figure;
        subplot(3, 1, 1);
        plot(xP_DP, yP_DP, 'r', xP_CBP, yP_CBP, 'b');
        xlabel('x position');
        ylabel('y position');
        legend('DP', 'CBP');
        title(['Trajectories for V_P = ' num2str(VP)]);
        
        % Plot R vs time for DP and CBP
        subplot(3, 1, 2);
        plot(state_DP(:, end), 'r');
        hold on;
        plot(state_CBP(:, end), 'b');
        hold off;
        xlabel('Time (s)');
        ylabel('Range R');
        legend('DP', 'CBP');
        title(['Range R vs Time for V_P = ' num2str(VP)]);
        
        % Calculate times of capture for DP and CBP
        Tc_DP = interp1(state_DP(:, end), t_span, 0);
        Tc_CBP = interp1(state_CBP(:, end), t_span, 0);
        
        disp(['For V_P = ' num2str(VP) ', Time of Capture (DP) = ' num2str(Tc_DP) ', Time of Capture (CBP) = ' num2str(Tc_CBP)]);
    end
end

function dydt = DP_ode(~, y, V_E, VP)
    % Extract states for DP
    xE = y(1);
    yE = y(2);
    xP = y(3);
    yP = y(4);
    beta = y(5);
    R = y(6);
    
    % Differential equations for DP
    dydt = zeros(6, 1);
    dydt(1) = 0;
    dydt(2) = V_E;
    dydt(3) = VP * cos(beta);
    dydt(4) = VP * sin(beta);
    dydt(5) = 0; % Constant bearing pursuit
    dydt(6) = hypot(xE - xP, yE - yP); % Range R
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
    dydt(5) = 0; % Constant bearing pursuit
    dydt(6) = -VP * cos(atan2(yE - yP, xE - xP)); % Range R
end

function [value, isterminal, direction] = eventFunc(~, y)
    % Event function to stop simulation when R = 0
    value = y(6); % R
    isterminal = 1; % Stop the integration
    direction = -1; % Negative direction
end

