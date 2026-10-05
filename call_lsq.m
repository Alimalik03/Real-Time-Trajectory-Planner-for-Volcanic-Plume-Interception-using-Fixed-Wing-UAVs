function trim_out = call_lsq(guess_array, Va, gamma, R, P)
    % guess_array: [alpha (deg), beta (deg), theta (deg), phi (deg), psi (deg),
    %               delta_a (rad), delta_e (rad), delta_r (rad), delta_T (0-1)]

    input_array = [Va; gamma; R];

    % Lower and upper bounds (angles in degrees to match trim_cost)
    lb = [-25; -5;  -45; -30; -30; -0.5; -0.5; -0.5; 0];
    ub = [ 25;  5;   45;  30;  30;  0.5;  0.5;  0.5; 1];

    options = optimoptions('lsqnonlin', 'Display', 'iter', 'FunctionTolerance', 1e-8);

    % Run optimizer (passing extra arguments to trim_cost)
    [z_trim, ~, ~, ~] = lsqnonlin(@(z) trim_cost(z, input_array, P), ...
                                  guess_array, lb, ub, options);

    trim_out = z_trim';  % row vector
end
