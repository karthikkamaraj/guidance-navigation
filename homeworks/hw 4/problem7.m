function solve_problem_7()
    % Define parameters
    Ts = 0.001; % Time step
    tf_bar = 10; % Final time
    T = 1; % Time constant T
    Lambdas = [3, 4, 5]; % Values of Lambda to simulate

    % Time vector
    t = 0:Ts:tf_bar;

    % Iterate over different Lambda values
    for Lambda = Lambdas
        % Solve the adjoint problem
        G = zeros(size(t));
        g_final = 0;
        
        % Calculate G(tf_bar, t) using backward recursion
        for i = length(t):-1:2
            G(i - 1) = G(i) * exp(-Lambda * Ts) + g_final * (1 - exp(-Lambda * Ts));
            g_final = G(i - 1);
        end

        % Calculate -TG(tf_bar, t)
        TG = -T * G;

        % Plot -TG(tf_bar, t) versus (tf_bar - t)/T
        figure;
        plot((tf_bar - t)/T, TG);
        xlabel('(tf - t)/T');
        ylabel('-TG(tf, t)');
        title(['Lambda = ' num2str(Lambda)]);
    end
end
