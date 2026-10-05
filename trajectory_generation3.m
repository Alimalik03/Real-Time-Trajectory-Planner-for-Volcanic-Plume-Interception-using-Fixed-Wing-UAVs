%% PARAMETERS
% Aircraft performance
param_1a;
Params_autopilot;
V = P.Va_trim;            % m/s Airspeed
g = 9.81;                 % Gravity

max_bank   = P.phi_max;   % [rad] Max bank angle
turn_radius = V^2/(g*tan(max_bank));
max_yaw_rate = V/turn_radius;           % [rad/s]

max_pitch  = P.theta_max;%deg2rad(20);               % [rad] Max (absolute) pitch angle for climb/desc
max_climb_rate   = V * sin(max_pitch);  % m/s (positive)
max_descend_rate = -V * sin(max_pitch); % m/s (negative)

dt = 0.2;                 % Time step [s]

% Initial state
x0 = P.pn0; y0 = P.pe0; z0 = -P.pd0;    % [m]
yaw0 = P.psi0;                          % initial azimuth (0 = x+ axis)
pitch0 = 0;                             % level flight

% Plume position
xp = 900; yp = 1000; zp = -1000;          % [m] (set to test climb or descend)

% Compute the desired azimuth to the plume and set the initial yaw to it.
% vec2plume_initial = [xp, yp, zp] - [x0, y0, z0];
% yaw0 = atan2(vec2plume_initial(2), vec2plume_initial(1));

%% INITIALIZE
pos   = [x0, y0, z0];
yaw   = yaw0;
pitch = pitch0;
traj  = pos;

%% PHASE 1: LEVEL TURN TO AZIMUTH
% Compute azimuth to plume
vec2plume = [xp, yp, zp] - pos;
az_des    = atan2(vec2plume(2), vec2plume(1));
yaw_error = wrapToPi(az_des - yaw);

figure; hold on; grid on; axis equal;
plot3(x0, y0, z0, 'bo', 'MarkerFaceColor','b','MarkerSize',9);
plot3(xp, yp, zp, 'ro', 'MarkerFaceColor','r','MarkerSize',9);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

while abs(yaw_error) > deg2rad(2)
    % Only change yaw, keep pitch 0 (level turn)
    delta_yaw = sign(yaw_error)*min(abs(yaw_error), max_yaw_rate*dt);
    yaw = yaw + delta_yaw;

    % No pitch, so only xy changes
    pos = pos + V*dt*[cos(pitch)*cos(yaw), cos(pitch)*sin(yaw), sin(pitch)];
    traj = [traj; pos];

    % Update error
    vec2plume = [xp, yp, zp] - pos;
    az_des    = atan2(vec2plume(2), vec2plume(1));
    yaw_error = wrapToPi(az_des - yaw);

    % Draw
    if mod(size(traj,1),5)==1
        plot3(traj(:,1),traj(:,2),traj(:,3),'k-'); drawnow;
    end
end

%% PHASE 2: CONTINUOUS (SPIRAL) INTERCEPT TO THE PLUME
% Keep turning toward the current azimuth to the plume and
% adjust pitch each step to burn down altitude error gradually.
% No separate straight-line final: stop when at the plume.

k_alt = 0.5;          % [1/s] proportional gain for climb rate
dist_threshold = 6;   % [m] 3D intercept threshold

while true
    % Vector and distance to plume
    vec2plume  = [xp, yp, zp] - pos;
    dist2plume = norm(vec2plume);
    if dist2plume <= dist_threshold
        break; % 3D intercept complete
    end

    % Desired azimuth (bearing) and yaw update (rate-limited)
    az_des    = atan2(vec2plume(2), vec2plume(1));
    yaw_error = wrapToPi(az_des - yaw);
    delta_yaw = sign(yaw_error)*min(abs(yaw_error), max_yaw_rate*dt);
    yaw = yaw + delta_yaw;

    % Altitude control: desired climb rate = k_alt * altitude_error (saturated)
    alt_error      = zp - pos(3);                     % [m]
    climb_rate_cmd = k_alt * alt_error;               % [m/s]
    climb_rate_cmd = max(max_descend_rate, min(max_climb_rate, climb_rate_cmd));

    % Convert climb rate to pitch command, with pitch limits
    pitch_cmd = asin(climb_rate_cmd / V);
    pitch = max(-max_pitch, min(max_pitch, pitch_cmd));

    % Integrate kinematics
    dx = V*cos(pitch)*cos(yaw)*dt;
    dy = V*cos(pitch)*sin(yaw)*dt;
    dz = V*sin(pitch)*dt;
    pos = pos + [dx, dy, dz];
    traj = [traj; pos];

    % Draw
    if mod(size(traj,1),5)==1
        plot3(traj(:,1), traj(:,2), traj(:,3), 'k-'); drawnow;
    end
end

% (No PHASE 3: straight final intercept)

save('traj.mat','traj')
legend('Start','Plume','Traj'); % 'Intercept'
title('3D Path');
