% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)
    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4); % in body axes
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1); % Positive deflection gives nose down/LH rotation around y
    delta_a = delta(2); % Positive deflection gives roll right/RH rotation around x INCORRECT????
    delta_r = delta(3); % Positive deflection gives yaw left/LH rotation around z
    delta_t = delta(4); % Positive deflection gives 
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = 0;%wind(4); % gust along body x-axis
    v_wg    = 0;%wind(5); % gust along body y-axis    
    w_wg    = 0;%wind(6); % gust along body z-axiss
    
    
    % =====================================================================
    % ------------------------------- WIND --------------------------------
    % =====================================================================
    
    V_ws_veh  = [w_ns; w_es; w_ds];                  % Steady wind in inertial v frame
    V_wg_body = [u_wg; v_wg; w_wg];                  % Gust vector in body frame
    
    % To get wind data in the body frame, we need to transform the
    % steady wind from the vehicle ned frame to the body frame
    V_ws_body = rotate_VtoB(V_ws_veh, phi, theta, psi);  % Steady wind in body frame   
    V_w_body  = V_ws_body + V_wg_body;              % All wind in body frame
        
    % compute airspeed vector
    u_r = u - V_w_body(1);
    v_r = v - V_w_body(2);
    w_r = w - V_w_body(3);
    
    % compute air data
    Va = sqrt((u_r^2)+(v_r^2)+(w_r^2));       % scalar, airspeed
    alpha = atan(w_r/u_r);                  % angle of attack
    beta = asin(v_r/Va);                    % angle of sideslip
    
    % In NED ref frame (inertial)
    V_w_ned = rotateBtoV(V_w_body,phi,theta,psi);
    
    % =====================================================================
    % ------------------------------ FORCES -------------------------------
    % =====================================================================
    % Note: The simple version of C_L(alpha) etc is being used: = C_L0+C_Lalpha*alpha    
    
    % dynamic pressure
    dynpres     = 0.5*(P.rho)*(Va^2)*(P.S_wing);

    % gravity
    force_g_veh     = [0; 0; P.mass*P.gravity]; %x;y;z positive down
    force_g_body    = rotate_VtoB(force_g_veh,phi,theta,psi); % from ned to body
    
    % longitudinal: lift force
    C_LAoA      = P.C_L_alpha*alpha;
    C_Lpitch    = 0.5*P.C_L_q*P.c*q/Va;
    C_Lelev     = P.C_L_delta_e*delta_e;
    F_lift      = dynpres*(P.C_L_0 + C_LAoA + C_Lpitch + C_Lelev);
    
    % longitudinal: drag force
    C_DAoA      = P.C_D_alpha*alpha;
    C_Dpitch    = 0.5*P.C_D_q*P.c*q/Va;
    C_Delev     = P.C_D_delta_e*delta_e;
    F_drag      = dynpres*(P.C_D_0 + C_DAoA + C_Dpitch + C_Delev); 
    
    % longitudinal: pitching moment
    C_mAoA      = P.C_m_alpha*alpha;
    C_mpitch    = 0.5*P.C_m_q*P.c*q/Va;
    C_melev     = P.C_m_delta_e*delta_e;
    T_pitch     = dynpres*P.c*(P.C_m_0 + C_mAoA + C_mpitch + C_melev); 
    
    % lateral: lateral force
    C_Ysideslip = P.C_Y_beta*beta;
    C_Yroll     = 0.5*P.C_Y_p*P.b*p/Va;
    C_Yyaw      = 0.5*P.C_Y_r*P.b*r/Va;
    C_Yail      = P.C_Y_delta_a*delta_a;
    C_Yrud      = P.C_Y_delta_r*delta_r;
    F_y         = dynpres*(P.C_Y_0 + C_Ysideslip + C_Yroll + C_Yyaw + C_Yail + C_Yrud);
    
    % lateral: rolling moment
    C_lsideslip = P.C_ell_beta*beta;
    C_lroll     = 0.5*P.C_ell_p*P.b*p/Va;
    C_lyaw      = 0.5*P.C_ell_r*P.b*r/Va;
    C_lail      = P.C_ell_delta_a*delta_a;
    C_lrud      = P.C_ell_delta_r*delta_r;
    T_roll      = dynpres*P.b*(P.C_ell_0 + C_lsideslip + C_lroll + C_lyaw + C_lail + C_lrud);    
    
    % lateral: yawing moment
    C_nsideslip = P.C_n_beta*beta;
    C_nroll     = 0.5*P.C_n_p*P.b*p/Va;
    C_nyaw      = 0.5*P.C_n_r*P.b*r/Va;
    C_nail      = P.C_n_delta_a*delta_a;
    C_nrud      = P.C_n_delta_r*delta_r;
    T_yaw       = dynpres*P.b*(P.C_n_0 + C_nsideslip + C_nroll + C_nyaw + C_nail + C_nrud);    
    
    % aero forces & moments (body axis)
    force_aero_body  = [-F_drag*cos(alpha) + F_lift*sin(alpha);...
                       F_y;...
                       -F_drag*sin(alpha) - F_lift*cos(alpha)];
                   
    moment_aero_body = [T_roll;...
                        T_pitch;...
                        T_yaw];
    
    % propulsive forces and moments (body axis)
    propVector  = [((P.k_motor*delta_t)^2 - Va^2); 0; 0];
    force_prop_body  = (0.5*P.rho*P.S_prop*P.C_prop)*propVector;
    
    moment_prop_body = [-P.k_T_P*(P.k_Omega*delta_t)^2 ; 0; 0];    
    
    % compute external forces and torques on aircraft in body axes
    % Force = force_g_body + force_aero_body + force_prop_body
    % Torque = moment_aero_body + moment_prop_body
    Force(1) =  force_g_body(1) + force_aero_body(1) + force_prop_body(1);
    Force(2) =  force_g_body(2) + force_aero_body(2) + force_prop_body(2);
    Force(3) =  force_g_body(3) + force_aero_body(3) + force_prop_body(3); 
    
    Torque(1) = moment_aero_body(1) + moment_prop_body(1);
    Torque(2) = moment_aero_body(2) + moment_prop_body(2);
    Torque(3) = moment_aero_body(3) + moment_prop_body(3);
    
    out = [Force'; Torque'; Va; alpha; beta; V_w_ned];
        
end



