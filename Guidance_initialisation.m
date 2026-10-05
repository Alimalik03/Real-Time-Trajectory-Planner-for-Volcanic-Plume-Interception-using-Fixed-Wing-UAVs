% ====================================
% UAV 6DOF SIMULATION USING RK4 METHOD
% ====================================
clear all;
clc;

% --- Load Parameters ---
param_1a;
Params_autopilot;
traj = load('traj.mat');
traj = traj.traj;
traj_NED = [traj(:,1), traj(:,2), traj(:,3)]; % Assuming traj is [N, E, D]

% --- Simulation Setup ---
t_final = 200;          % seconds
dt = 0.01;             % time step
T = 0:dt:t_final;      % time vector

% --- Initial State (12 states) ---
x0 = [P.pn0; P.pe0; -P.pd0;
      P.u0; P.v0; P.w0;
      P.phi0; P.theta0;P.psi0;
      P.p0; P.q0; P.r0];

% --- Control Inputs (deflections + throttle)
    % Format: [δe; δa; δr; δt]
delta_a = P.delta_a_t;    % elevator deflection (rad)
delta_e = P.delta_e_t;     % aileron deflection (rad)
delta_r = P.delta_r_t;     % rudder deflection (rad)
delta_T = P.delta_T_t;            % throttle (0 to 1)

delta = [delta_e; delta_a; delta_r; delta_T];
wind = [P.wind_n; P.wind_e; P.wind_d];

% Preallocate
X = zeros(length(T), 12);
Xdot = zeros(length(T), 12);
U = zeros(length(T), 4);
current_path_error_hist = zeros(9241,1);


% Store guidance commands for plotting or analysis
R_history = zeros(length(T), 3); % To store reference waypoints (r)
Q_history = zeros(length(T), 3); % To store direction vectors (q)

%% %% Controller Initialization

% --- Initial state ---
x = x0;

% Define the final waypoint for termination check
final_waypoint = traj_NED(end, :)';

% % Pick a center and a feasible radius
% centerNED = [900; 1000;  -800];   % loiter over this point at ~800 m altitude AGL/NED
% rho       = 180;                  % m (>= V^2/(g*tan(phi_max)) with margin)
% dir       = +1;                   % CCW (left) orbit
% orbit     = makeOrbitFixed(centerNED, rho, dir);

% Define a small threshold for reaching the final waypoint
proximity_threshold = 7; % meters
chi_inf = deg2rad(45);     % 30–60 deg
k_path  = 0;            % straight gain
k_orbit = 0.5;             % orbit gain
R       = 150; 

Chi_act_vec = zeros(length(T),1);   % log of actual course
prev_pn = x0(1);
prev_pe = x0(2);
chi_act  = x0(9);                   % fallback for the first step


for i = 1:length(T)
    t = T(i);
    h = -x(3); % Altitude (positive up)
    u_current = x(4);
    v_current = x(5);
    w_current = x(6);
    theta_current = x(8);
    phi_current = x(7);
    psi_current = x(9);
    p_current = x(10);
    q_current = x(11);
    r_current = x(12);
    V = sqrt(x(4)^2 + x(5)^2 + x(6)^2); % Airspeed magnitude
    beta = asin(x(5)/V); % Sideslip angle
    alpha = atan2(w_current, u_current); % Angle of attack (atan2 for correct quadrant)
    
    % Get current MAV position for followWpp
    p_mav = x(1:3); % [pn; pe; pd]
    
  
    % Finite-difference ground-velocity from positions
    dpn = (p_mav(1) - prev_pn) / dt;
    dpe = (p_mav(2) - prev_pe) / dt;

    if (abs(dpn) + abs(dpe)) > 1e-6
       chi_meas = atan2(dpe, dpn);     % course from N/E ground velocity
    else
       chi_meas = chi_act;             % avoid NaN if nearly stationary
    end  

    % Optional: light smoothing to reduce jitter
    alpha   = 0.2;                      % 0..1 (higher = less smoothing)
    dchi    = wrapToPi(chi_meas - chi_act);
    chi_act = wrapToPi(chi_act + alpha*dchi);

    % Log and update previous positions
    Chi_act_vec(i) = chi_act;
    prev_pn = p_mav(1);
    prev_pe = p_mav(2);
    % chi = x(9);

    % --- Call followWpp to get reference waypoint (r) and direction (q) ---
    % 'traj_NED' is your waypoint path 'W'
    % 'p_mav' is your MAV position 'p'
    [r_guidance, q_guidance] = followWpp(traj_NED, p_mav);
   
    
    % Store for plotting/analysis later
    R_history(i, :) = r_guidance';
    % R_history(i, :) = r';

    Q_history(i, :) = q_guidance';
    % Q_history(i, :) = q'; 

    [Chi_commanded_val, H_commanded_val, V_commanded_val, current_path_error_m] = pathFollowingCourseAltitudeAirspeed(x, r_guidance, q_guidance, P); % Keep original for now, needs refinement to use r,q
    current_path_error_hist(i) = current_path_error_m;

    % [H_commanded_val, Chi_commanded_val,e_py] = follow_Straight_Line(r_guidance, q_guidance, p_mav, chi_act, chi_inf, k_path, dt);
    % V_commanded_val = 35;
    % current_path_error_hist(i) = e_py;

    % [H_commanded_val, Chi_commanded_val, V_commanded_val, mode, info] = guidance_function(x, traj_NED, R, P, chi_inf, k_path, k_orbit);
    % [H_commanded_val, Chi_commanded_val, V_commanded_val, mode, info] = guidanceStraightOrTurn(x, traj_NED, P, dt);


    [theta_commanded] = altitude_from_pitch_controller(H_commanded_val , h, V,dt,P);
    % [theta_commanded] = altitude_with_pitch_hold(H_commanded_val , h,V,dt ,P);

    phi_commanded = course_hold_controller(Chi_commanded_val,chi_act,V,dt,P);
    % phi_commanded = course_hold(Chi_commanded_val,chi_act,V,dt,P);
    delta_a = roll_hold(phi_commanded,phi_current,p_current,V,P);
    delta_T = Airspeed_with_throttle_controller(V_commanded_val,V,dt,P);

    delta_e = pitch_attitude_controller(theta_commanded,theta_current,q_current,dt,P); % Or from your pitch controller if reactivated
    
    % delta_e = pitch_hold(theta_commanded,theta_current,q_current,V,P); % Or from your pitch controller if reactivated
    delta_r = P.delta_r_t; % Or from your sideslip controller if reactivated

    delta = [delta_e; delta_a; delta_r; delta_T];

    % Store control input
    U(i,:) = [delta_e, delta_a, delta_r, delta_T];
    
    wind = [P.wind_n; P.wind_e; P.wind_d];

    % --- RK4 Integration ---
    k1 = mav_6dof(t,       x, delta, wind, P);
    k2 = mav_6dof(t+dt/2,  x + (dt/2)*k1, delta, wind, P);
    k3 = mav_6dof(t+dt/2,  x + (dt/2)*k2, delta, wind, P);
    k4 = mav_6dof(t+dt,    x + dt*k3,     delta, wind, P);
    xdot_rk4 = (1/6)*(k1 + 2*k2 + 2*k3 + k4);

    % Save
    X(i,:) = x';
    Xdot(i,:) = xdot_rk4';
    
    % Update state
    x = x + dt * xdot_rk4;

    % --- Termination Condition Check ---
    % Calculate distance to the final waypoint
    dist_to_final_waypoint = norm(p_mav - final_waypoint);
    
    % Check if MAV is close enough to the final waypoint AND
    % if the followWpp function has already reached its last segment
    % (implicitly handled by followWpp's 'i' reaching N-1).
    if dist_to_final_waypoint < proximity_threshold
        time_to_intercept = t;
        % save("time_to_intercept.mat",time_to_intercept)
        fprintf('MAV reached final waypoint. Simulation ending at time %.2f s.\n', t);
        % Trim preallocated arrays to actual simulation length
        X = X(1:i,:);
        Xdot = Xdot(1:i,:);
        U = U(1:i,:);
        T = T(1:i);
        R_history = R_history(1:i,:);
        Q_history = Q_history(1:i,:);
        break; % Exit the simulation loop
    end
    
    % Animate aircraft (if drawAircraft is available)
    uu_draw = [x; t];
    % drawAircraft(uu_draw);
end

%           PLOTTING
% ---------- States (one) ----------
figure('Name','States','Color','w');

subplot(4,3,1);  plot(T, X(:,1));  grid on; ylabel('pn [m]');        xlabel('Time [s]');xlim([0 118]);
subplot(4,3,2);  plot(T, X(:,2));  grid on; ylabel('pe [m]');        xlabel('Time [s]');xlim([0 118]);
subplot(4,3,3);  plot(T, -X(:,3));  grid on; ylabel('pd [m]');        xlabel('Time [s]');xlim([0 118]);

subplot(4,3,4);  plot(T, X(:,4));  grid on; ylabel('u [m/s]');       xlabel('Time [s]');xlim([0 118]);
subplot(4,3,5);  plot(T, X(:,5));  grid on; ylabel('v [m/s]');       xlabel('Time [s]');xlim([0 118]);
subplot(4,3,6);  plot(T, X(:,6));  grid on; ylabel('w [m/s]');       xlabel('Time [s]');xlim([0 118]);

subplot(4,3,7);  plot(T, rad2deg(X(:,7)));  grid on; ylabel('\phi [rad]');    xlabel('Time [s]');xlim([0 118]);
subplot(4,3,8);  plot(T, rad2deg(X(:,8)));  grid on; ylabel('\theta [rad]');  xlabel('Time [s]');xlim([0 118]);
subplot(4,3,9);  plot(T, rad2deg(X(:,9)));  grid on; ylabel('\psi [rad]');    xlabel('Time [s]');xlim([0 118]);


subplot(4,3,10); plot(T, X(:,10)); grid on; ylabel('p [rad/s]');     xlabel('Time [s]');xlim([0 118]);
subplot(4,3,11); plot(T, X(:,11)); grid on; ylabel('q [rad/s]');     xlabel('Time [s]');xlim([0 118]);
subplot(4,3,12); plot(T, X(:,12)); grid on; ylabel('r [rad/s]');     xlabel('Time [s]');xlim([0 118]);

sgtitle('State Variables vs Time');

% ---------- Control Inputs (one figure, 4 subplots) ----------
figure('Name','Control Inputs','Color','w');

subplot(2,2,1); plot(T, rad2deg(U(:,1))); grid on; ylabel('\delta_e [rad]'); xlabel('Time [s]');xlim([0 118]);
subplot(2,2,2); plot(T, rad2deg(U(:,2))); grid on; ylabel('\delta_a [rad]'); xlabel('Time [s]');xlim([0 118]);
subplot(2,2,3); plot(T, rad2deg(U(:,3))); grid on; ylabel('\delta_r [rad]'); xlabel('Time [s]');xlim([0 118]);
subplot(2,2,4); plot(T, U(:,4)); grid on; ylabel('\delta_t [-]');   xlabel('Time [s]');xlim([0 118]);

sgtitle('Control Surface / Throttle Commands');

% %plot
% figure;
% % plot(T, current_path_error_hist, 'LineWidth',1.5);
% xlabel('Time (s)');
% ylabel('Path Error (m)');
% title('Current Path Error vs Time');
% grid on

% ----plot Trajectory--------
figure;
hold on;
grid on;
box on;
plot3(traj_NED(:,2), traj_NED(:,1), -traj_NED(:,3), 'b--', 'LineWidth', 2, 'DisplayName', 'Desired Trajectory');
plot3(X(:,2), X(:,1), -X(:,3), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Aircraft Trajectory');
plot3(traj_NED(1,2), traj_NED(1,1), -traj_NED(1,3), 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'blue', 'DisplayName', 'Aircraft Initial Position');
plot3(traj_NED(end,2), traj_NED(end,1), -traj_NED(end,3), 's', 'MarkerSize', 8, 'MarkerFaceColor', 'green', 'DisplayName', 'Plume Location');
% plot3(X(1,2), X(1,1), -X(1,3), 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'red', 'DisplayName', 'Aircraft Start');
% plot3(X(end,2), X(end,1), -X(end,3), 's', 'MarkerSize', 8, 'MarkerFaceColor', 'magenta', 'DisplayName', 'Aircraft End');

% Labels and Title
xlabel('East [m]');
ylabel('North [m]');
zlabel('Altitude [m]');
title('Scenario 4: Spiral climb into plume');
legend('Location', 'best');
axis equal;
view(45, 30);


% % Plot r and q vectors for specific time steps (e.g., every 100th step)
% % The 'quiver3' function plots 3D vectors.
% skip_step = 100; % Adjust to control density of plotted vectors
% for k = 1:skip_step:length(T)
%     % r_guidance is the origin of the vector, q_guidance is the direction
%     % Scale the direction vector 'q' for better visualization if needed
%     quiver3(R_history(k,2), R_history(k,1), -R_history(k,3), ...
%             Q_history(k,2) * 50, Q_history(k,1) * 50, -Q_history(k,3) * 50, ... % Scale by 50m for visibility
%             'Color', [0.5 0.5 0.5], 'LineWidth', 1, 'DisplayName', 'Guidance Vector (q)');
% end
% % Make sure legend entry for 'Guidance Vector' appears only once
% h_legend = findobj(gca,'DisplayName','Guidance Vector (q)');
% if ~isempty(h_legend) && length(h_legend) > 1
%     delete(h_legend(2:end)); % Delete duplicate legend entries
% end