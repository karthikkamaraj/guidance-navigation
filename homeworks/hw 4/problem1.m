function pursuit_simulation()
    % Define initial positions and speeds
    xE0 = 100; % Evader initial x position
    yE0 = 0;   % Evader initial y position
    xP0 = 0;   % Pursuer initial x position
    yP0 = 0;   % Pursuer initial y position
    V_E = 5;   % Evader constant speed
    VP_values = [6, 8, 11]; % Pursuer speeds to simulate
    
    % Iterate over different VP values
    for VP = VP_values
        % Define simulation time span
        t_span = [0, 100];
        
        % Define initial state vector
        initial_state = [xE0, yE0, xP0, yP0, atan2(yE0, xE0 - xP0), hypot(xE0 - xP0, yE0 - yP0)];
        
        % Set options for the ODE solver
        options = odeset('Events', @eventFunc, 'RelTol', 1e-6, 'AbsTol', 1e-6);
        
        % Solve the differential equations using ode45
        [t, state] = ode45(@(t, y) pursuitODE(t, y, V_E, VP), t_span, initial_state, options);
        
        % Extract states
        xE = state(:, 1);
        yE = state(:, 2);
        xP = state(:, 3);
        yP = state(:, 4);
        beta = state(:, 5);
        R = state(:, 6);
        
        % Plot trajectories of E and P
        figure;
        plot(xE, yE, 'b', xP, yP, 'r');
        xlabel('x position');
        ylabel('y position');
        legend('Evader', 'Pursuer');
        title(['Trajectories for V_P = ' num2str(VP)]);
        
        % Plot R vs time
        figure;
        plot(t, R);
        xlabel('Time (s)');
        ylabel('Range R');
        title(['Range R vs Time for V_P = ' num2str(VP)]);
        
        % Calculate theoretical time of capture Tc
        Tc = interp1(R, t, 0); % Interpolate time when R = 0
        
        disp(['For V_P = ' num2str(VP) ', theoretical time of capture Tc = ' num2str(Tc)]);
    end
end

function dydt = pursuitODE(~, y, V_E, VP)
    % Extract states
    xE = y(1);
    yE = y(2);
    xP = y(3);
    yP = y(4);
    beta = y(5);
    R = y(6);
    
    % Differential equations
    dydt = zeros(6, 1);
    dydt(1) = 0;
    dydt(2) = V_E;
    dydt(3) = VP * cos(beta);
    dydt(4) = VP * sin(beta);
    dydt(5) = (V_E / R) * cos(beta);
    dydt(6) = V_E * cos(beta - pi/2) - VP;
end

function [value, isterminal, direction] = eventFunc(~, y)
    % Event function to stop simulation when R = 0
    value = y(6); % R
    isterminal = 1; % Stop the integration
    direction = -1; % Negative direction
end
