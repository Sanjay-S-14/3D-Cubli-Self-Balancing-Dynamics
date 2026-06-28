function tau_out = SMC_Controller(theta, theta_dot, omega_wheel)

%1 Assigning constants
m = 1.081; % Mass
I = 0.01581; % Inertia of Mass
L = 0.106; % Lever arm
g = 9.80665; % Gravity 

%2 Assinging tuning parameters
c = 20;
K = 0.15;
Epsilon = 0.05;

%3 Assigning operating point
theta_target = pi/4;
theta_dot_target = 0;

% --- NEW: WHEEL VELOCITY CONSTRAINT PARAMETERS ---
k_w = 0.0002;
max_lean = 0.18; 
% -------------------------------------------------
    
% Calculate lean offset to brake the wheel
lean_shift = k_w * omega_wheel; 
    
% Saturate the lean so the cube doesn't accidentally flip over!
if lean_shift > max_lean
   lean_shift = max_lean;
elseif lean_shift < -max_lean
    lean_shift = -max_lean;
end
    
% Dynamic target angle
%theta_target = theta_target - lean_shift;
% -------------------------------------------------

%4 Calculating error
e = theta - theta_target;
e_dot = theta_dot - theta_dot_target;

%5 Defining sliding surface
s = (e * c) + e_dot;

%6 Calculating equivalent control
tau_eq = -( (m * g * L * sin(e)) + (I * c * e_dot) );

%7 Calculating switching control 
tau_sw = - K * tanh(s / Epsilon);

%8 Calculating required torque
tau_frame = tau_eq + tau_sw;

%9 Assigning outpu torque
tau_motor_raw = -tau_frame; 
    
% 10. MOTOR SATURATION (Physical Limit)
% Prevents the massive 20 N-m spike during the initial catch
max_torque = 1.2; % Stall torque of your motor in N-m
    
if tau_motor_raw > max_torque
    tau_out = max_torque;
elseif tau_motor_raw < -max_torque
    tau_out = -max_torque;
else
    tau_out = tau_motor_raw;
end
end
