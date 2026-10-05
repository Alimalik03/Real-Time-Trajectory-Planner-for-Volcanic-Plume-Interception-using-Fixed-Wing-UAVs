function[phi_commanded] = course_hold_controller(chi_c, chi, Va, dt, P)
persistent int_phi 
persistent d_phi

if isempty(int_phi)
    int_phi = 0;
end

if isempty(d_phi)
    d_phi = 0;
end

error = chi_c - chi;
Kp_h = 12;
Ki_h = 7;
Kd_h = 0.0000;
int_phi = int_phi + error*dt;
d_phi = d_phi + (error/dt);
phi_commanded = Kp_h*error + Ki_h*int_phi + Kd_h * d_phi;

phi_commanded = sat(phi_commanded, P.phi_max, -P.phi_max);

end