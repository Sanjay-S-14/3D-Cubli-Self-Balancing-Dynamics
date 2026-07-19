function [tau_x, tau_y, tau_z,error_distance] = smc_edge_controller(theta_current, omega_frame, omega_wheels, jump_trigger, target_angle, max_lean, k_momentum, safe_speed, epsilon_e, K_e, c_e, r_com, g, m, I_wheel, I_matrix)
    % theta_current: [roll, pitch, yaw] in radians from the sensor (Earth Frame)
    % omega_frame:   [wx, wy, wz] in rad/s from the gyroscope (Body Frame)
    % omega_wheels:  [wx_wheel, wy_wheel, wz_wheel] in rad/s

    jump_check = 0;
    tau_motor = [0; 0; 0;];

    % =========================================================
    % OUTER LOOP: MOMENTUM MANAGEMENT (Deadband Lean)
    % =========================================================

    % Read instantaneous Y-axis (Pitch) wheel speed
    pitch_wheel_speed = omega_wheels(2);

    % Deadband logic
    if pitch_wheel_speed > safe_speed
        active_omega = pitch_wheel_speed - safe_speed;
    elseif pitch_wheel_speed < -safe_speed
        active_omega = pitch_wheel_speed + safe_speed;
    else
        active_omega = 0; % Inside safe zone, do not lean
    end 

    % Calculate and saturate the lean shift
    lean_shift = k_momentum * active_omega;
    lean_shift = max(min(lean_shift, max_lean), -max_lean);

    % Generate dynamic target (Base -45 deg edge + shift)
    target_pitch = target_angle - lean_shift;
    theta_target = [0; target_pitch; 0];
    
    % =========================================================
    % CALCULATE ERRORS & FRAME MAPPING
    % =========================================================
    % Raw Position error (In Earth Frame)
    e_euler = theta_current - theta_target;
    
    % Extract current Roll (phi) and Pitch (theta)
    phi = theta_current(1);
    theta = theta_current(2);
    
    % Kinematic Transformation Matrix (W)
    % Maps Earth-frame Euler angle errors into physical Body-frame axes
    W = [1,  0,        -sin(theta); 
         0,  cos(phi),  sin(phi)*cos(theta); 
         0, -sin(phi),  cos(phi)*cos(theta)];
         
    % Project the error into the physical Body Frame
    e_body = W * e_euler;
    e_dot_body = omega_frame;
    
    % Check how close the cube is to the edge balance point (in radians)
    error_distance = norm(e_body);

    % =========================================================
    % SMC 3D EDGE CATCH & BALANCE
    % =========================================================
    if jump_trigger >= 0.2 && jump_trigger <= 2.3
        % ---------------------------------------------------------
        % STATE 1: THE Y-AXIS TORQUE BUILD UP
        % ---------------------------------------------------------
        tau_motor = [0; -0.2; 0];

        jump_check = 0;

    elseif jump_trigger >= 2.3 && jump_trigger <= 2.32
        % ---------------------------------------------------------
        % STATE 2: THE Y-AXIS JUMP
        % ---------------------------------------------------------
        tau_motor = [0; 21; 0];

        jump_check = 1;

    elseif jump_trigger >= 2.32 && theta <= -0.3927
        
        % A. The 3D Sliding Surface (3x1 Vector)
        S = (c_e * e_body) + e_dot_body;
        
        % B. Dynamic Gravity Compensation (Rotate World Z down to Body)
        R31 = -sin(theta);
        R32 = cos(theta) * sin(phi);
        R33 = cos(theta) * cos(phi);
        g_body_rotated = [-g * R31; -g * R32; -g * R33];
        tau_gravity = cross(r_com, m * g_body_rotated);
        
        % C. SMC Equivalent Control (tau_eq)
        L_wheels = I_wheel .* omega_wheels; 
        alpha_desired = -c_e * e_dot_body;
        
        % Full 3D Inverse Dynamics
        tau_eq = (I_matrix * alpha_desired) + cross(omega_frame, (I_matrix * omega_frame + L_wheels)) - tau_gravity;
                 
        % D. SMC Switching Control (tau_sw)
        tau_sw = -K_e * tanh(S / epsilon_e);
        
        % E. Total Torque (Newton's Third Law flipped for the motors)
        tau_frame = tau_eq + tau_sw;
        tau_motor = [0; -tau_frame(2); 0];
        
        jump_check = 0;
        
    end

    % =========================================================
    % OUTPUT ASSIGNMENT & SATURATION
    % =========================================================
    
    tau_x = tau_motor(1);
    tau_z = tau_motor(3);

    if jump_check == 1

        tau_y = tau_motor(2);

    else

        % Limit the motors to their physical maximum (e.g. 1.2 N-m)
        max_torque = 1.2;
        tau_y = max(min(tau_motor(2), max_torque), -max_torque);

    end
