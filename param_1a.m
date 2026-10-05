 clc;
clear all;
Params_autopilot;

P = struct();
P.gravity = 9.8;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
%physical parameters of airframe
P.mass = 25;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = .1204;

P.I = [P.Jx, 0, -P.Jxz; 
    0, P.Jy, 0;
    -P.Jxz 0 P.Jz]; %Inertia Vector 

% physical calculations
T    = P.Jx*P.Jz-P.Jxz^2;
P.T1 = P.Jxz*(P.Jx-P.Jy+P.Jz)/T;
P.T2 = (P.Jz*(P.Jz-P.Jy) + P.Jxz^2)/T;
P.T3 = P.Jz/T;
P.T4 = P.Jxz/T;
P.T5 = (P.Jz-P.Jx)/P.Jy;
P.T6 = P.Jxz/P.Jy;
P.T7 = (P.Jx*(P.Jx-P.Jy)+P.Jxz^2)/T;
P.T8 = P.Jx/T;

% aerodynamic coefficients
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;

P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;

P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

% Additional aerodynamic coefficients
P.C_p_0         = P.T3*P.C_ell_0 + P.T4*P.C_n_0;
P.C_p_beta      = P.T3*P.C_ell_beta + P.T4*P.C_n_beta;
P.C_p_p         = P.T3*P.C_ell_p + P.T4*P.C_n_p;
P.C_p_r         = P.T3*P.C_ell_r + P.T4*P.C_n_r;
P.C_p_delta_a   = P.T3*P.C_ell_delta_a + P.T4*P.C_n_delta_a;
P.C_p_delta_r   = P.T3*P.C_ell_delta_r + P.T4*P.C_n_delta_r;
P.C_r_0         = P.T4*P.C_ell_0 + P.T8*P.C_n_0;
P.C_r_beta      = P.T4*P.C_ell_beta + P.T8*P.C_n_beta;
P.C_r_p         = P.T4*P.C_ell_p + P.T8*P.C_n_p;
P.C_r_r         = P.T4*P.C_ell_r + P.T8*P.C_n_r;
P.C_r_delta_a   = P.T4*P.C_ell_delta_a + P.T8*P.C_n_delta_a;
P.C_r_delta_r   = P.T4*P.C_ell_delta_r + P.T8*P.C_n_delta_r;

% wind parameters
P.wind_n = 0;%3;
P.wind_e = 0;%2;
P.wind_d = 0;
P.L_u = 0;%200;
P.L_v = 0;%200;
P.L_w = 0;%50;
P.sigma_u = 0;%1.06; 
P.sigma_v = 0;%1.06;
P.sigma_w = 0;%.7;


% compute trim conditions using 'mavsim_trim.slx'
% initial airspeed
% Va0 = 30;
P.Va0 = 35;
P.gamma = deg2rad(0);%5*pi/180;  % desired flight path angle (radians)
P.R     = inf;       % desired radius (m) - use (+) for right handed orbit, 
%                                             (-) for left handed orbit


% autopilot sample rate
P.Ts = 0.1;


% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = P.Va0; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

trim_init;
u_trim = P.u_trim;
x_trim = P.x_trim;

% run trim commands
% [x_trim, u_trim]=compute_trim('mavsim_trim',P.Va0,P.gamma,P.R)


% set initial conditions to trim conditions
% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = P.x_trim(4);  % initial velocity along body x-axis
P.v0     = P.x_trim(5);  % initial velocity along body y-axis
P.w0     = P.x_trim(6);  % initial velocity along body z-axis
P.phi0   = P.x_trim(7);  % initial roll angle  
P.theta0 = P.x_trim(8);  % initial pitch angle
P.psi0   = P.x_trim(9);  % initial yaw angle
P.p0     = P.x_trim(10);  % initial body frame roll rate
P.q0     = P.x_trim(11);  % initial body frame pitch rate
P.r0     = P.x_trim(12);  % initial body frame yaw rate

P.delta_a_t = P.u_trim(2);
P.delta_e_t = P.u_trim(1);
P.delta_r_t = P.u_trim(3);
P.delta_T_t = P.u_trim(4);
P.Va_trim= sqrt(P.u0^2 + P.v0^2 + P.w0^2);

% load('traj.mat');
% traj_NED = [traj(:,1), traj(:,2), traj(:,3)];
