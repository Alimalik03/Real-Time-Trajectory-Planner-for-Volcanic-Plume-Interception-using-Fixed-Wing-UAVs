%Autopilot Params
% loop aircraft params
P.delta_a_max   = 45*pi/180;    % 
P.delta_r_max   = 45*pi/180;
P.delta_e_max   = 45*pi/180;
P.delta_t_max = 1;

% loop design params Lateral control
P.zeta_phi      = 2.3700;       
P.e_phi_max     = 15*pi/180;    % 
P.phi_max       = 45*pi/180;    
P.zeta_chi      = 0.7166;       
P.W_chi         = 10;           % 
P.e_beta_max    = 30*pi/180;    %
P.zeta_beta     = -0.1390;  

% additional loop design params
P.e_theta_max   = 10*pi/180;    
P.zeta_theta    = 1.7982;       
P.theta_max     = 25*pi/180;    
P.zeta_h        = 0.2184;       
P.W_h           = 10;           
P.zeta_V2       = 0.7539;       
P.W_V2          = 10;           
P.zeta_V        = 6.3578;       
P.omega_n_V     = 1.2402;       

% Performance params
P.theta_takeoff             = 10*pi/180;
P.altitude_take_off_zone    = 5; % meters
P.altitude_hold_zone        = 25; % meters
P.tecs_weight               = 0.9; % in [0,2], with higher meaning more on E_K


%
V_n = 35;
max_bank = P.phi_max;%deg2rad(45);      % Max bank angle [rad]
P.turn_radius = V_n^2/(9.81*tan(max_bank));
P.max_yaw_rate = V_n/P.turn_radius; % [rad/s]

max_pitch = P.theta_max;%deg2rad(20);     % Max (absolute) pitch angle for climb/desc
P.climb_rate = V_n * sin(max_pitch);   % m/s (positive)
P.descent_rate = -V_n * sin(max_pitch);% m/s (negative)