%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% roll_hold
%   - roll attitude loop for lateral autopilot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_a = roll_hold(phi_c, phi, p, Va, P)

    error = phi_c-phi;
    dynpres = 0.5*P.rho*(Va^2)*P.S_wing;
    a_phi1  = -dynpres*P.b*P.C_p_p*(0.5*P.b/Va);
    a_phi2  = dynpres*P.b*P.C_p_delta_a;

    omega_n_phi = sqrt(abs(a_phi2)*P.delta_a_max/P.e_phi_max);
    
    % instability with sat at k_d=0.75, and 20% off is 0.6
    %       zeta=(k_d_phi*a_phi2 + a_phi1)/(2*omega_n_phi)=2.37

    k_d_phi = (2*P.zeta_phi*omega_n_phi - a_phi1)/a_phi2;
    k_p_phi = sign(a_phi2)*P.delta_a_max/P.e_phi_max;

    delta_a = sat(k_p_phi*(error) - k_d_phi*p,P.delta_a_max,-P.delta_a_max);
    
end
