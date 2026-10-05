function [delta_T] = Airspeed_with_throttle_controller(V_ref, u, dt, P)

persistent int_v_prev
persistent e_v_prev % For derivative calculation

if isempty(int_v_prev)
    int_v_prev = 0;
    e_v_prev = V_ref - u; % Initialize for first step
end

% Velocity Controller Gains
kp_v = 0.0785;
ki_v = 0.0221;
kd_v = 0.0942;

% Controller equation
e_v = V_ref - u;

% Integral term
int_v = int_v_prev + e_v * dt;

% Derivative term (using backward difference for simplicity)
d_v = (e_v - e_v_prev) / dt;

delta_T = kp_v * e_v + ki_v * int_v + kd_v * d_v;

% Clamp delta_T to valid range (0 to 1 for throttle)
% delta_T = max(0, min(delta_T, 1));
delta_T = sat(delta_T, 1, 0);

% Update persistent variables for next iteration
int_v_prev = int_v;
e_v_prev = e_v;

end