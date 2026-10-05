function [theta_commanded] = altitude_from_pitch_controller(h_commanded , h, Va,dt,P)

persistent int_h 
persistent d_h

if isempty(int_h)
    int_h = 0;
end

if isempty(d_h)
    d_h = 0;
end

error = h_commanded - h;
Kp_h = 0.0400; %0.0320;% 0.0280
Ki_h = 0.00025;%0.00025;%0.000143;
Kd_h = 0.0000;
int_h = int_h + error*dt;
d_h = d_h + (error/dt);
theta_commanded = Kp_h*error + Ki_h*int_h + Kd_h * d_h;

theta_commanded = sat(theta_commanded, P.theta_max, -P.theta_max);


