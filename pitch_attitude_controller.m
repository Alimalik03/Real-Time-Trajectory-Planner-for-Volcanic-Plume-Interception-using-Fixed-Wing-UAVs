function [delta_e] = pitch_attitude_controller(theta_ref,theta,q,dt,P)

%% Constants
persistent int_theta % Declare persistent
persistent int_q     % Declare persistent

% Outer loop: θ → q_ref
Kp_theta = 4.3;
Ki_theta = 18.15;

% Inner loop: q → δe
Kp_q = -0.5;
Ki_q = -3.5;

%% Outer loop theta control
if isempty(int_theta) % Initialize only if empty (first call)
   int_theta = 0;
end

e_theta = theta_ref - theta;
int_theta = int_theta + e_theta * dt;
q_ref = Kp_theta * e_theta + Ki_theta * int_theta;
q_ref = max(min(q_ref, deg2rad(25)), deg2rad(-25));

%% Inner loop

if isempty(int_q) % Initialize only if empty (first call)
    int_q = 0;
end

e_q = q_ref - q;
int_q = int_q + e_q * dt;
delta_e = Kp_q * e_q + Ki_q * int_q;
delta_e = max(min(delta_e, deg2rad(25)), deg2rad(-25));

end