function [r, q] = followWpp(W, p)
% FOLLOWWPP Implements the waypoint following algorithm.
%   (r, q) = followWpp(W, p)
%   Input:
%     W = {w1, ..., wN}: Waypoint path, an Nx3 matrix of [North, East, Down] coordinates.
%     p = (pn, pe, pd)': MAV position, a 3x1 vector.
%   Output:
%     r: Reference waypoint (wi-1), a 3x1 vector.
%     q: Direction vector of the current segment (qi-1), a 3x1 vector.

persistent currentWaypointIndex;
persistent lastWaypointPathHash; % To detect new waypoint paths

N = size(W, 1); % Number of waypoints

if N < 3
    error('Waypoint path must have at least 3 waypoints (N >= 3).');
end

% Compute a simple hash for the waypoint path to detect changes
% In a real system, you might pass a path ID or manage this more robustly.
currentPathHash = sum(W(:)); % Simple sum of all elements as a hash

if isempty(currentWaypointIndex) || currentPathHash ~= lastWaypointPathHash
    fprintf('New waypoint path detected or first call. Resetting waypoint index.\n');
    currentWaypointIndex = 2; % Initialize waypoint index: i <- 2
    lastWaypointPathHash = currentPathHash;
end

% Ensure i doesn't go beyond N-1
i = min(currentWaypointIndex, N - 1);

% 4: r <- wi-1
r = W(i - 1, :)'; % Transpose to make it a column vector

% 5: qi-1 <- (wi - wi-1) / |wi - wi-1|
segment_prev = W(i, :)' - W(i - 1, :)';
norm_segment_prev = norm(segment_prev);
if norm_segment_prev == 0
    q_i_minus_1 = zeros(3, 1); % Handle zero-length segment
else
    q_i_minus_1 = segment_prev / norm_segment_prev;
end

% The logic for incrementing 'i' only applies if we're not on the last segment
if i < N - 1
    % 6: qi <- (wi+1 - wi) / |wi+1 - wi|
    segment_next = W(i + 1, :)' - W(i, :)';
    norm_segment_next = norm(segment_next);
    if norm_segment_next == 0
        q_i = zeros(3, 1);
    else
        q_i = segment_next / norm_segment_next;
    end

    % 7: ni <- (qi-1 + qi) / |qi-1 + qi|
    sum_q_vectors = q_i_minus_1 + q_i;
    norm_sum_q_vectors = norm(sum_q_vectors);
    if norm_sum_q_vectors == 0
        n_i = zeros(3, 1); % Should ideally not happen if segments have length
    else
        n_i = sum_q_vectors / norm_sum_q_vectors;
    end

    % 8: if p ∈ H(wi, ni) then
    % The condition (p - wi) . ni >= 0 means p has crossed the half-plane.
    % This is a common way to define a half-plane in 3D.
    if dot((p - W(i, :)'), n_i) >= 0
        fprintf('MAV crossed plane at waypoint %d. Incrementing index.\n', i);
        % 9: Increment i <- (i + 1) until i = N - 1
        currentWaypointIndex = min(currentWaypointIndex + 1, N - 1);
    end
end

% 11: return r, q = qi-1 at each time step
r = W(currentWaypointIndex - 1, :)'; % Ensure r always corresponds to the segment we just passed or are on
q = q_i_minus_1; % The direction vector for the current segment
end