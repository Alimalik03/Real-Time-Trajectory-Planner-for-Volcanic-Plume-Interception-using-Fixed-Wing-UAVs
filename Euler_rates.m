function [phi_theta_psi_dot] = Euler_rates(p,q,r,phi,theta,psi)
phi_theta_psi_dot = [1     sin(phi)*tan(theta)     cos(phi)*tan(theta);
              0     cos(phi)                -sin(phi);
              0     sin(phi)/cos(theta)     cos(phi)/cos(theta);...
              ]*[p; q; r];

% phi_dot = phi_theta_psi_dot(1);
% theta_dot = phi_theta_psi_dot(2);
% psi_dot = phi_theta_psi_dot(3);
