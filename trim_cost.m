function F = trim_cost(guess_array,Input_array,P)
    % Extract state and control
    alpha = pi/180*guess_array(1);
    beta = pi/180*guess_array(2);
    theta = pi/180*guess_array(3);
    phi = pi/180*guess_array(4);
    psi = pi/180*guess_array(5);
    
    delta_a = guess_array(6);
    delta_e = guess_array(7);
    delta_r = guess_array(8);
    delta_T = guess_array(9);

    V = Input_array(1);
    gamma = Input_array(2);  
    R = Input_array(3);

    Turn_Rate = (V/R);%*pi/180;

    u = V*cos(alpha)*cos(beta);
    v = V*sin(beta);
    w = V*sin(alpha)*cos(beta);


    if isinf(R)
        p=0;q=0;r=0;
    else
      
        p = -(Turn_Rate)*sin(theta);
        q = (Turn_Rate)*sin(phi)*cos(theta);
        r = (Turn_Rate)*cos(phi)*cos(theta);
    end

    
    % State vector
    x = [0; 0; 0; u; v; w; phi; theta; psi; p; q; r];

    %control vector
    delta = [delta_e; delta_a; delta_r; delta_T];

    %wind
    wind = [0; 0; 0];

    State_derivatives = mav_6dof(0,x,delta,wind,P);
    % % Compute constraints
    % gamma = theta - alpha;
    % h_dot = -xdot(3);                % Climb rate
    % r_cmd = Va / R_d;                % Desired yaw rate

    h_dot = V*sin(gamma);
    H_dot = -State_derivatives(3) - h_dot;
    r_cmd = V/R;
    r_dot = State_derivatives(9) - r_cmd;

    % Trim conditions
    F = [
        State_derivatives(4:6);                   % Force balance
        State_derivatives(10:12);                 % Moment balance                  % Turn rate
        H_dot;     % Climb rate match
        % r_dot
        ];
end
