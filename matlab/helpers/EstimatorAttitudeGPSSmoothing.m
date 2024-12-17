function [aircraft_state_est, wind_inertial_est, gps_est, P_est_est, innov, gps_globals] = EstimatorAttitudeGPSSmoothing(time, gps_sensor, inertial_sensors, sensor_params, gps_globals)
    %
    % gps_sensor = [pn; pe; ph; Vg; chi]
    %
    % inertial_sensors = [y_accel; y_gyro; y_pressure; y_dyn_pressure];
    %

    % persistent phat
    % persistent qhat
    % persistent rhat

    % persistent press_stat
    % persistent press_dyn

    % persistent pn_hat
    % persistent pe_hat
    % persistent chi_hat2

    % persistent s_x
    % persistent s_y
    % persistent s_z
    % persistent chi_hat

    % persistent phi_hat
    % persistent theta_hat
    % persistent P_est

    % persistent xhat_gps
    % persistent P_gps

    phat = gps_globals.phat;
    qhat = gps_globals.qhat;
    rhat = gps_globals.rhat;
    press_stat = gps_globals.press_stat;
    press_dyn = gps_globals.press_dyn;
    phi_hat = gps_globals.phi_hat;
    theta_hat = gps_globals.theta_hat;
    P_est = gps_globals.P_est;
    xhat_gps = gps_globals.xhat_gps;
    P_gps = gps_globals.P_gps;

    h_ground = sensor_params.h_ground;
    density = stdatmo(h_ground);

    Ts_imu = sensor_params.Ts_imu; % <================ This filter does not support multiple update rates
    Ts_gps = sensor_params.Ts_gps; % <================ so these better be the same!
    g = sensor_params.g;

    %%%%%%%%%%%%%%%%%%%%%%
    %%% angular velocity
    %%%%%%%%%%%%%%%%%%%%%%
    a_omega = 1000;
    alpha_omega = exp(-a_omega * Ts_imu);

    if (isempty(phat))
        phat = inertial_sensors(4);
    else
        phat = LowPassFilter(phat, inertial_sensors(4), alpha_omega);
    end

    if (isempty(qhat))
        qhat = inertial_sensors(5);
    else
        qhat = LowPassFilter(qhat, inertial_sensors(5), alpha_omega);
    end

    if (isempty(rhat))
        rhat = inertial_sensors(6);
    else
        rhat = LowPassFilter(rhat, inertial_sensors(6), alpha_omega);
    end

    %%%%%%%%%%%%%%%
    %%% height
    %%%%%%%%%%%%%%%

    a_h = 10;
    alpha_h = exp(-a_h * Ts_imu);

    if (isempty(press_stat))
        press_stat = inertial_sensors(7);
    else
        press_stat = LowPassFilter(press_stat, inertial_sensors(7), alpha_h);
    end

    hhat = h_ground + press_stat / (density * g);

    %%%%%%%%%%%%%%%%
    %%% airspeed
    %%%%%%%%%%%%%%%%

    a_Va = 30;
    alpha_Va = exp(-a_Va * Ts_imu);

    if (isempty(press_dyn))
        press_dyn = inertial_sensors(8);
    else
        press_dyn = LowPassFilter(press_dyn, inertial_sensors(8), alpha_Va);
    end

    Va = sqrt(2 * press_dyn / density);

    %%%%%%%%%%%%%%%%%%%%%
    %%% orientation
    %%%%%%%%%%%%%%%%%%%%
    Q = .01 * ((pi / 180) ^ 2) * eye(2);
    R = sensor_params.sig_accel * sensor_params.sig_accel * eye(3);

    if (isempty(phi_hat))
        phi_hat = 0;
        theta_hat = 0;
        P_est = ((30 * pi / 180) ^ 2) * eye(2);
    else
        %%% Propagate
        [xdot, A] = AttitudeFilterUpdate(phi_hat, theta_hat, phat, qhat, rhat);
        phi_hat = phi_hat + xdot(1) * Ts_imu;
        theta_hat = theta_hat + xdot(2) * Ts_imu;
        P_est = P_est + Ts_imu * (A * P_est + P_est * A' + Q);

        %%% Measurement update
        [zhat, H] = AttitudeFilterMeasurement(phi_hat, theta_hat, phat, qhat, rhat, Va, g);
        L = P_est * H' * inv(R + H * P_est * H');
        P_est = (eye(2) - L * H) * P_est;
        xhat = [phi_hat; theta_hat] + L * (wrapToPi(inertial_sensors(1:3, 1) - zhat));
        phi_hat = xhat(1);
        theta_hat = xhat(2);
    end

    % %%%%%%%%%%%%%%%%%%%%
    % %%% position (gps)
    % %%%%%%%%%%%%%%%%%%%%
    % a_gps = 20; %2
    % alpha_gps = exp(-a_gps*Ts_gps);
    % %alpha_gps = exp(-a_gps*Ts_imu);
    %
    %
    % if(isempty(pn_hat))
    %     pn_hat = gps_sensor(1);
    % else
    %     if(mod(time, Ts_gps)==0) % <=============== Only update at GPS rate
    %         del_n = gps_sensor(1) - pn_hat;
    %         pn_hat = LowPassFilter(pn_hat, gps_sensor(1), alpha_gps);
    %     end
    % end
    %
    % if(isempty(pe_hat))
    %     pe_hat = gps_sensor(2);
    %     chi_hat2 = 0;
    % else
    %     if(mod(time, Ts_gps)==0) % <=============== Only update at GPS rate
    %         del_e = gps_sensor(2) - pe_hat;
    %         pe_hat = LowPassFilter(pe_hat, gps_sensor(2), alpha_gps);
    %
    %         chi_hat_est = atan2(del_e,del_n);
    %         chi_hat2 = LowPassFilter(chi_hat2, chi_hat_est, alpha_gps);
    %     end
    % end
    %
    % if(isempty(chi_hat))
    %     chi_hat = gps_sensor(5);
    % else
    %     if(mod(time, Ts_gps)==0) % <=============== Only update at GPS rate
    %         chi_hat = LowPassFilter(chi_hat, gps_sensor(5), alpha_gps);
    %     end
    % end
    % psi_hat = chi_hat;

    %%%%%%%%%%%%%%%%%%%%%
    %%% GPS smoothing
    %%%%%%%%%%%%%%%%%%%%
    Qgps = [10 ^ 2 0 0 0 0 0 0; ... %pn
                0 10 ^ 2 0 0 0 0 0; ... % pe
                0 0 2 ^ 2 0 0 0 0; ... %Vg
                0 0 0 (5 * pi / 180) ^ 2 0 0 0; ... %chi
                0 0 0 0 25 0 0; ... %wn
                0 0 0 0 0 25 0; ... %we
                0 0 0 0 0 0 (5 * pi / 180) ^ 2]; %psi

    Rgps = [sensor_params.sig_gps(1) ^ 2 0 0 0 0 0; ...
                0 sensor_params.sig_gps(2) ^ 2 0 0 0 0; ...
                0 0 sensor_params.sig_gps_v ^ 2 0 0 0; ...
                0 0 0 (sensor_params.sig_gps_v / 20) ^ 2 0 0; ...
                0 0 0 0 sensor_params.sig_gps_v ^ 2 0; ...
                0 0 0 0 0 sensor_params.sig_gps_v ^ 2];

    if (isempty(xhat_gps))
        xhat_gps = [gps_sensor(1); gps_sensor(2); gps_sensor(4); gps_sensor(5); 0; 0; gps_sensor(5)];
        P_gps = 10 * Qgps;
        yerr_gps = [0; 0; 0; 0; 0; 0];
    else
        %%% Propagate
        %[xdot, A] = AttitudeFilterUpdate(phi_hat, theta_hat, phat, qhat, rhat);
        [xdot_gps, A_gps] = GPSSmoothingUpdate(xhat_gps, Va, qhat, rhat, phi_hat, theta_hat, g);
        xhat_gps = xhat_gps + xdot_gps * Ts_imu; % <=============== Assumes filter runs at IMU rate
        P_gps = P_gps + Ts_imu * (A_gps * P_gps + P_gps * A_gps' + Qgps); % <=============== Assumes filter runs at IMU rate

        if (mod(time, Ts_gps) == 0) % <=============== Only update at GPS rate
            %%% Measurement update
            %[zhat, H] = AttitudeFilterMeasurement(phi_hat, theta_hat, phat, qhat, rhat, Va, g);
            [zhat_gps, H_gps] = GPSSmoothingMeasurement(xhat_gps, Va);
            ygps = [gps_sensor(1); gps_sensor(2); gps_sensor(4); gps_sensor(5); 0; 0];
            L_gps = P_gps * H_gps' * inv(Rgps + H_gps * P_gps * H_gps');
            P_gps = (eye(7) - L_gps * H_gps) * P_gps;

            yerr_gps = ygps - zhat_gps;

            yerr_gps(4) = wrapToPi(yerr_gps(4));

            xhat_gps = xhat_gps + L_gps * (yerr_gps);
        else % <=============== Still update innovation
            [zhat_gps, H_gps] = GPSSmoothingMeasurement(xhat_gps, Va);
            ygps = [gps_sensor(1); gps_sensor(2); gps_sensor(4); gps_sensor(5); 0; 0];
            yerr_gps = ygps - zhat_gps;
            yerr_gps(4) = wrapToPi(yerr_gps(4));
        end

    end

    %%%%%%%%%%%%%%%
    %%% output
    %%%%%%%%%%%%%%%
    wind_body_est = TransformFromInertialToBody([xhat_gps(5); xhat_gps(6); 0], [phi_hat; theta_hat; xhat_gps(7)]);
    air_rel_est = [Va * cos(theta_hat); 0; Va * sin(theta_hat)];
    vel_body_est = air_rel_est + wind_body_est;

    aircraft_state_est = [xhat_gps(1); xhat_gps(2); -hhat; ...
                              phi_hat; theta_hat; xhat_gps(7); ...
                              vel_body_est; ...
                              phat; qhat; rhat];
    wind_inertial_est = [xhat_gps(5); xhat_gps(6); 0];

    gps_est = [xhat_gps(1); xhat_gps(2); xhat_gps(3); xhat_gps(4); xhat_gps(5); xhat_gps(6); xhat_gps(7)];
    P_est_est = P_gps;
    innov = yerr_gps;

    % aircraft_state_est = [pn_hat; pe_hat; -hhat;...
    %      phi_hat; theta_hat; psi_hat;...
    %      Va*cos(theta_hat); 0; Va*sin(theta_hat);...
    %      phat; qhat; rhat];
    % wind_inertial_est = [0; 0; 0];

    gps_globals.phat = phat;
    gps_globals.qhat = qhat;
    gps_globals.rhat = rhat;
    gps_globals.press_stat = press_stat;
    gps_globals.press_dyn = press_dyn;
    gps_globals.phi_hat = phi_hat;
    gps_globals.theta_hat = theta_hat;
    gps_globals.P_est = P_est;
    gps_globals.xhat_gps = xhat_gps;
    gps_globals.P_gps = P_gps;

end %function

%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%
% Low Pass Filter
%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%

function ynew = LowPassFilter(yold, unew, alpha)
    ynew = alpha * yold + (1 - alpha) * unew;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
% Attitude Filter Equations
%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%

function [xdot, A] = AttitudeFilterUpdate(phi, theta, p, q, r)

    xdot = [p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta); ...
                q * cos(phi) - r * sin(phi)];

    A = [q * cos(phi) * tan(theta) - r * sin(phi) * tan(theta) (q * sin(phi) + r * cos(phi)) / (cos(theta) ^ 2); ...
             -q * sin(phi) - r * cos(phi) 0];

end

function [y, H] = AttitudeFilterMeasurement(phi, theta, p, q, r, Va, g)

    y = [q * Va * sin(theta) + g * sin(theta); ...
             r * Va * cos(theta) - p * Va * sin(theta) - g * cos(theta) * sin(phi); ...
             -q * Va * cos(theta) - g * cos(theta) * cos(phi)];

    H = [0 q * Va * cos(theta) + g * cos(theta); ...
             -g * cos(phi) * cos(theta) -r * Va * sin(theta) - p * Va * cos(theta) + g * sin(phi) * sin(theta); ...
             g * sin(phi) * cos(theta) (q * Va + g * cos(phi)) * sin(theta)];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
% GPS Smoothing Filter Equations
%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%

% xh = [pn, pe, Vg, chi, wn, we, psi]
function [xdot, A] = GPSSmoothingUpdate(xh, Va, q, r, roll, pitch, g)

    psidot = q * sin(roll) / cos(pitch) + r * cos(roll) / cos(pitch);
    Vg_dot = ((Va * sin(xh(7)) + xh(6)) * (Va * psidot * cos(xh(7))) - (Va * cos(xh(7)) + xh(5)) * (Va * psidot * sin(xh(7)))) / xh(3);

    xdot = [xh(3) * cos(xh(4)); ...
                xh(3) * sin(xh(4)); ...
                Vg_dot; ...
                (g / xh(3)) * tan(roll) * cos(xh(4) - xh(7)); ...
                0; ...
                0; ...
                psidot];

    pVgd_ppsi = -Va * (xh(5) * cos(xh(4)) + xh(6) * sin(xh(4))) / xh(3);
    pcd_pVg =- (g / (xh(3)) ^ 2) * tan(roll) * cos(xh(4) - xh(7));
    pcd_pc =- (g / xh(3)) * tan(roll) * sin(xh(4) - xh(7));
    pcd_ppsi = (g / xh(3)) * tan(roll) * sin(xh(4) - xh(7)); ;

    A = [0 0 cos(xh(4)) -xh(3) * sin(xh(4)) 0 0 0; ...
             0 0 sin(xh(4)) xh(3) * cos(xh(4)) 0 0 0; ...
             0 0 -Vg_dot / xh(3) 0 -psidot * Va * sin(xh(7)) / xh(3) psidot * Va * cos(xh(7)) / xh(3) pVgd_ppsi; ...
             0 0 pcd_pVg pcd_pc 0 0 pcd_ppsi; ...
             0 0 0 0 0 0 0; ...
             0 0 0 0 0 0 0; ...
             0 0 0 0 0 0 0];

end

function [y, H] = GPSSmoothingMeasurement(xh, Va)

    y = [xh(1); ...
             xh(2); ...
             xh(3); ...
             xh(4); ...
             Va * cos(xh(7)) + xh(5) - xh(3) * cos(xh(4)); ...
             Va * sin(xh(7)) + xh(6) - xh(3) * sin(xh(4))];

    H = [1 0 0 0 0 0 0; ...
             0 1 0 0 0 0 0; ...
             0 0 1 0 0 0 0; ...
             0 0 0 1 0 0 0; ...
             0 0 -cos(xh(4)) xh(3) * sin(xh(4)) 1 0 -Va * sin(xh(7)); ...
             0 0 -sin(xh(4)) -xh(3) * cos(xh(4)) 0 1 Va * cos(xh(7)); ...
         ];

end