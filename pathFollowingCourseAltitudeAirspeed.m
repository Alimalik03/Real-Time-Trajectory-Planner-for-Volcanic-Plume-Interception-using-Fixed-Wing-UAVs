function [Chi_commanded_val, H_commanded_val, V_commanded_val, current_path_error_m] = pathFollowingCourseAltitudeAirspeed(x, r_guidance, q_guidance, P)
% PATHFOLLOWINGCOURSEALTITUDEAIRSPEED Calculates commanded course, altitude, and airspeed
%   based on guidance from followWpp.
%
% Inputs:
%   x: Current 12-state vector of the MAV [pn; pe; pd; u; v; w; phi; theta; psi; p; q; r]
%   r_guidance: 3x1 vector, the reference waypoint from followWpp [rn; re; rd]
%   q_guidance: 3x1 vector, the direction vector from followWpp [qn; qe; qd]
%   P: Structure containing aircraft parameters (e.g., P.Va0 for desired airspeed)
%
% Outputs:
%   Chi_commanded_val: Commanded course angle (rad)
%   H_commanded_val: Commanded altitude (m, positive up)
%   V_commanded_val: Commanded airspeed (m/s)
%   current_path_error_m: A simplified path error metric

% Extract current MAV position from state vector
p_mav = x(1:3); % [pn; pe; pd]

% Altitude Command (from r_guidance)
% The commanded altitude is simply the negative of the 'down' component of r_guidance.
% r_guidance(3) is 'pd' (positive down), so -r_guidance(3) converts it to altitude (positive up).
H_commanded_val = -r_guidance(3);

% Airspeed Command
% This can be a fixed desired airspeed from your parameters or a dynamic target.
% For waypoint following, it's often a constant cruise speed.
V_commanded_val = P.Va0; % Assuming P.Va0 is your desired cruise airspeed

% % Course Command (from q_guidance)
% The commanded course angle (Chi) is derived from the North (qn) and East (qe)
% components of the direction vector q_guidance.
% atan2(y, x) gives the angle in the correct quadrant.
Chi_commanded_val = atan2(q_guidance(2), q_guidance(1)); % atan2(qe, qn)

% Path Error (simplified for demonstration)
% A simple path error could be the distance from the MAV to the reference waypoint.
% In a more sophisticated path follower, this would be a cross-track error to the segment.
% current_path_error_m = norm(p_mav - r_guidance);
% --- Cross-track error to straight-line path (Beard & McLain, ch.10)
% chi_q = atan2(q_guidance(2), q_guidance(1));          % path course
R_i2p = [ cos(Chi_commanded_val)  sin(Chi_commanded_val)  0;                  % inertial->path frame
         -sin(Chi_commanded_val)  cos(Chi_commanded_val)  0;
               0            0     1];

e_p   = R_i2p * (p_mav - r_guidance);                 % relative error in path frame
current_path_error_m = abs(e_p(2));                   % cross-track |epy|

end