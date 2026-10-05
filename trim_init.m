%%% trim_initialisation file

alpha_guess = 0.1;
beta_guess =0;
theta_guess = 0.1;
phi_guess = 0;
psi_guess = 0;
dela_guess = 0;
dele_guess = 0;
delr_guess =0;
throttle = 0.5;

guess_array = [alpha_guess;beta_guess;theta_guess;phi_guess;psi_guess;dele_guess;dela_guess;delr_guess;throttle];

Trim_V = P.Va0;
Trim_gamma = P.gamma;
Trim_R = P.R;

P.Trim_output = call_lsq(guess_array,Trim_V,Trim_gamma,Trim_R,P);

% === Extract trimmed variables ===
alpha = deg2rad(P.Trim_output(1));
beta  = deg2rad(P.Trim_output(2));
theta = deg2rad(P.Trim_output(3));
phi   = deg2rad(P.Trim_output(4));
psi   = deg2rad(P.Trim_output(5));

% === Compute body velocities ===
u = Trim_V * cos(alpha) * cos(beta);
v = Trim_V * sin(beta);
w = Trim_V * sin(alpha) * cos(beta);

% === Angular rates ===

p = -pi/180*(Trim_V / Trim_R) * sin(phi) * sin(theta);
q = pi/180*(Trim_V / Trim_R) * sin(phi) * cos(theta);
r = pi/180*(Trim_V / Trim_R) * cos(phi) * cos(theta);
% % % === Control inputs ===
% [delta_a, delta_r] = compute_lateral_trim(p, q, r, beta, Trim_V, P);
% delta_e = compute_elevator_trim(p, q, r, alpha, Trim_V, P);
% delta_T = compute_throttle_trim(Trim_V, v, w, q, r, theta, alpha, delta_e, P);

% === State and control vectors ===
P.x_trim = [0; 0; 0; u; v; w; phi; theta; psi; p; q; r];
% P.u_trim = [delta_e; delta_a; delta_r; delta_T];
P.u_trim = [P.Trim_output(7); P.Trim_output(6); P.Trim_output(8); P.Trim_output(9)];

