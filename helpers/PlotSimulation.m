function PlotSimulation(time, aircraft_state_array, control_input_array, col)
    % This function is used to plot the results of a sim once its
    % completed.
    % Takes an input of length n vector which holds the time corresponding
    % to state variables.
    % 12 by n array of a/c state. 4 by n array of control inputs. And
    % string of col which indicates plotting option used for every plot.
    % i.e. col = 'b-'.

    % Func should create a total of 6 figures. 
        % 4 figs with each with 3 subplots showing position, euler angles,
        % inertial velocity, and angular velocity.
        % 1 figure with 4 subplots for each control input var.
        % 3D path plot of aircraft with positive height upward in the fig.
        % A/C should start a green and finish at red with a marker
    % Func should be called multiple times without clearing the figures,
    % thus using hold on.

    % Extract states

    xE = aircraft_state_array(1, :);
    yE = aircraft_state_array(2, :);
    zE = aircraft_state_array(3, :);

    phi = aircraft_state_array(4, :);
    theta = aircraft_state_array(5, :);
    psi = aircraft_state_array(6, :);

    uE = aircraft_state_array(7, :);
    vE = aircraft_state_array(8, :);
    wE = aircraft_state_array(9, :);

    p = aircraft_state_array(10, :);
    q = aircraft_state_array(11, :);
    r = aircraft_state_array(12, :);

    % Extract control inputs
    del_e = control_input_array(1, :);
    del_a = control_input_array(2, :);
    del_r = control_input_array(3, :);
    del_t = control_input_array(4, :);

    % Function to calculate ±5% y-limits
    calculateYLimits2 = @(data) [min(data) - 0.05*(max(data) - min(data)), max(data) + 0.05*(max(data) - min(data))] + [-0.1, 0.1];
    calculateYLimits = @(data) (min(data) == max(data)) * [min(data)-0.1, max(data)+0.1] + ~(min(data) == max(data)) * [min(data), max(data)];


    % Position plot
    figure(1);
    subplot(3,1,1);
    plot(time, xE, col); hold on;
    title('Position in xE (East)');
    xlabel('Time (s)');
    ylabel('xE (m)');
    ylim(calculateYLimits(xE));

    subplot(3,1,2);
    plot(time, yE, col); hold on;
    title('Position in yE (North)');
    xlabel('Time (s)');
    ylabel('yE (m)');
    ylim(calculateYLimits(yE));

    subplot(3,1,3);
    plot(time, zE, col); hold on;
    title('Position in zE (Down)');
    xlabel('Time (s)');
    ylabel('zE (m)');
    ylim(calculateYLimits(zE));

    % Euler angles plot
    figure(2);
    hold on;
    subplot(3,1,1);
    plot(time, phi, col); hold on;
    title('Roll angle (phi)');
    xlabel('Time (s)');
    ylabel('Phi (rad)');
    ylim(calculateYLimits(phi));

    subplot(3,1,2);
    plot(time, theta, col); hold on;
    title('Pitch angle (theta)');
    xlabel('Time (s)');
    ylabel('Theta (rad)');
    ylim(calculateYLimits(theta));

    subplot(3,1,3);
    plot(time, psi, col); hold on;
    title('Yaw angle (psi)');
    xlabel('Time (s)');
    ylabel('Psi (rad)');
    ylim(calculateYLimits(psi));

    % Inertial velocity plot
    figure(3);
    subplot(3,1,1);
    plot(time, uE, col); hold on;
    title('Inertial Velocity in xE (uE)');
    xlabel('Time (s)');
    ylabel('uE (m/s)');
    ylim(calculateYLimits(uE));

    subplot(3,1,2);
    plot(time, vE, col); hold on;
    title('Inertial Velocity in yE (vE)');
    xlabel('Time (s)');
    ylabel('vE (m/s)');
    ylim(calculateYLimits(vE));

    subplot(3,1,3);
    plot(time, wE, col); hold on;
    title('Inertial Velocity in zE (wE)');
    xlabel('Time (s)');
    ylabel('wE (m/s)');
    ylim(calculateYLimits(wE));

    % Angular velocity plot
    figure(4);
    subplot(3,1,1);
    plot(time, p, col); hold on;
    title('Angular Velocity p (Roll rate)');
    xlabel('Time (s)');
    ylabel('p (rad/s)');
    ylim(calculateYLimits(p));

    subplot(3,1,2);
    plot(time, q, col); hold on;
    title('Angular Velocity q (Pitch rate)');
    xlabel('Time (s)');
    ylabel('q (rad/s)');
    ylim(calculateYLimits(q));

    subplot(3,1,3);
    plot(time, r, col); hold on;
    title('Angular Velocity r (Yaw rate)');
    xlabel('Time (s)');
    ylabel('r (rad/s)');
    ylim(calculateYLimits(r));

    % Control inputs plot
    figure(5);
    subplot(4,1,1);
    plot(time, del_e, col); hold on;
    title('Elevator deflection (del_e)');
    xlabel('Time (s)');
    ylabel('del_e (rad)');
    ylim(calculateYLimits2(del_e));
    
    subplot(4,1,2);
    plot(time, del_a, col); hold on;
    title('Aileron deflection (del_a)');
    xlabel('Time (s)');
    ylabel('del_a (rad)');
    ylim(calculateYLimits2(del_a));
    
    subplot(4,1,3);
    plot(time, del_r, col); hold on;
    title('Rudder deflection (del_r)');
    xlabel('Time (s)');
    ylabel('del_r (rad)');
    ylim(calculateYLimits2(del_r));
    
    subplot(4,1,4);
    plot(time, del_t, col); hold on;
    title('Throttle (del_t)');
    xlabel('Time (s)');
    ylabel('Throttle (%)');
    ylim(calculateYLimits2(del_t));

    % 3D path plot
    figure(6);
    plot3(xE, yE, -zE, col); hold on; % -zE is used for altitude (positive upwards)
    % xlim(calculateYLimits(xE));
    % ylim(calculateYLimits(yE));
    % zlim(calculateYLimits(-zE));
    xlabel('xE (m)'); ylabel('yE (m)'); zlabel('Altitude (m)');
    title('3D Flight Path');
    grid on;
    plot3(xE(1), yE(1), -zE(1), 'go'); % Start point (green)
    plot3(xE(end), yE(end), -zE(end), 'ro'); % End point (red)

end
