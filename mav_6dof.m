function [Xdot] = mav_6dof(t,x,delta,wind, P)
% function for 6DOF EQM

    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    phi   = x(7);
    theta = x(8);
    psi   = x(9);
    p     = x(10);
    q     = x(11);
    r     = x(12);
    % fx    = uu(1);
    % fy    = uu(2);
    % fz    = uu(3);
    % ell   = uu(4);
    % m     = uu(5);
    % n     = uu(6);
    delta_e    = delta(1);
    delta_a    = delta(2);
    delta_r    = delta(3);
    delta_T   = delta(4);
    
%     plotForce([fx;fy;fz],t);

    delta = [delta_e; delta_a; delta_r; delta_T];  % define appropriately

    % wind = [P.wind_n; P.wind_e; P.wind_d];  % or time-varying if needed

    Out = forces_moments(x,delta, wind, P);
    display(Out)

    Force = [Out(1);Out(2);Out(3)];

    Torque = [Out(4);Out(5);Out(6)];
    
    V = Out(7);

    alpha = Out(8);
    beta = Out(9);

    % fx = output(1);
    % fy = output(2);
    % fz = output(3);
    % ell = output(4);
    % m = output(5);
    % n = output(6);
    % 
    % disp(fx)
    % disp(m)


    % posDot = rotate_BtoV([u; v; w],phi,theta,psi);
    xyzDot = rotateBtoV([u;v;w],phi,theta,psi);

    % uvwDot = [r*v-q*w; p*w-r*u; q*u-p*v] + [fx; fy; fz]./P.mass;
    uvwDot = (Force)/P.mass - cross([p;q;r],[u;v;w]);
    % uvwDot = cross([p;q;r], [u;v;w]) + Force / P.mass;
    
    % angDot = [1     sin(phi)*tan(theta)     cos(phi)*tan(theta);
    %           0     cos(phi)                -sin(phi);
    %           0     sin(phi)/cos(theta)     cos(phi)/cos(theta);...
    %           ]*[p; q; r];

    phi_theta_psi_Dot = Euler_rates(p,q,r,phi,theta,psi);
    
    % pqrDot = [P.T1*p*q - P.T2*q*r + P.T3*ell + P.T4*n;...
    %           P.T5*p*r - P.T6*(p^2 - r^2) + m/P.Jy;...
    %           P.T7*p*q - P.T1*q*r + P.T4*ell + P.T8*n;...
    %           ];

    pqr_Dot = P.I\(Torque) - cross([p;q;r],P.I*[p;q;r]);
    
    xdot    = xyzDot(1);
    ydot    = xyzDot(2);
    zdot    = xyzDot(3);
    udot     = uvwDot(1);
    vdot     = uvwDot(2);
    wdot     = uvwDot(3);
    phidot   = phi_theta_psi_Dot(1);
    thetadot = phi_theta_psi_Dot(2);
    psidot   = phi_theta_psi_Dot(3);
    pdot     = pqr_Dot(1);
    qdot     = pqr_Dot(2);
    rdot     = pqr_Dot(3);
    
Xdot = [xdot; ydot; zdot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];

% end mdlDerivatives

