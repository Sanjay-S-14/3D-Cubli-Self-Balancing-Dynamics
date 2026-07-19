function [tau_x, tau_y, tau_z, q_ev_corner, error_distance] = quaternion_corner_controller(q_current, omega_frame, omega_wheels, tau_y_edge, jump_tigger, I_matrix, I_wheel, m, g, r_com, q_target_corner, c, K, epsilon)
    % q_current:    [w, x, y, z] live orientation from the IMU/Sensor
    % omega_frame:  [wx, wy, wz] in rad/s from the gyroscope (Body Frame)
    % omega_wheels: [wx_wheel, wy_wheel, wz_wheel] in rad/s

    tau_x = 0;
    tau_y = 0;
    tau_z = 0;
    
    % =========================================================
    % ERROR CALCULATION & UNWINDING FIX (Hamilton Product)
    % =========================================================
    % Extract current quaternion components for readability
    wc = q_current(1); xc = q_current(2); yc = q_current(3); zc = q_current(4);
    
    % --- Calculate Error for the CORNER ---
    % q_error = q_target_corner_inverse * q_current
    w1_c = q_target_corner(1); x1_c = -q_target_corner(2); y1_c = -q_target_corner(3); z1_c = -q_target_corner(4);
    
    q_ew_c = w1_c*wc - x1_c*xc - y1_c*yc - z1_c*zc;
    q_ex_c = w1_c*xc + x1_c*wc + y1_c*zc - z1_c*yc;
    q_ey_c = w1_c*yc - x1_c*zc + y1_c*wc + z1_c*xc;
    q_ez_c = w1_c*zc + x1_c*yc - y1_c*xc + z1_c*wc;
    
    % Shortest path enforcement (Unwinding Fix)
    if q_ew_c < 0
        q_ew_c = -q_ew_c; q_ex_c = -q_ex_c; q_ey_c = -q_ey_c; q_ez_c = -q_ez_c;
    end
    q_ev_corner = [q_ex_c; q_ey_c; q_ez_c];
    
    % Check how close the cube is to the 3D corner (Vector Magnitude)
    error_distance = norm(q_ev_corner);

    % =========================================================
    % 4. STATE MACHINE (Edge to Corner Logic)
    % =========================================================

    if jump_tigger > 0.0 && jump_tigger < 2.0
        % ---------------------------------------------------------
        % STATE 1: THE X-AXIS TORQUE BUILD UP
        % ---------------------------------------------------------
        jump_torque_x =  0.15; % Fire the X-wheel upward
        jump_torque_z = -0.15; % Fire the Z-wheel upward
        
        tau_motor = [jump_torque_x; tau_y_edge; jump_torque_z];

        tau_x = tau_motor(1);
        tau_y = tau_motor(2);
        tau_z = tau_motor(3);

    elseif jump_tigger >= 2.0 && jump_tigger < 2.02
        % ---------------------------------------------------------
        % STATE 2: THE X-AXIS JUMP
        % ---------------------------------------------------------
        jump_torque_x = -14.5; % Fire the X-wheel upward
        jump_torque_z =  14.5; % Fire the Z-wheel upward
        
        tau_motor = [jump_torque_x; 0; jump_torque_z];

        tau_x = tau_motor(1);
        tau_y = tau_motor(2);
        tau_z = tau_motor(3);
        
    elseif jump_tigger >= 2.032 && error_distance <= 0.075

        % ---------------------------------------------------------
        % STATE 3: SMC 3D CORNER CATCH & BALANCE
        % ---------------------------------------------------------
        
        % A. Exact Quaternion Derivative for alpha_desired
        q_ev_dot = 0.5 * (q_ew_c * omega_frame + cross(q_ev_corner, omega_frame));
        
        % B. The 3D Sliding Surface
        S = (c * q_ev_corner) + omega_frame;
        
        % C. Dynamic Gravity Compensation (Using Quaternion to DCM rotation)
        % Extracts the 3rd Row of the rotation matrix using live quaternion values
        R31 = 2 * (xc*zc - wc*yc);
        R32 = 2 * (yc*zc + wc*xc);
        R33 = 1 - 2 * (xc^2 + yc^2);
        
        g_body_rotated = [-g * R31; -g * R32; -g * R33];
        tau_gravity = cross(r_com, m * g_body_rotated);
        
        % D. SMC Equivalent Control (tau_eq)
        L_wheels = I_wheel .* omega_wheels; 
        alpha_desired = -c * q_ev_dot;
        
        tau_eq = - (I_matrix * alpha_desired) - cross(omega_frame, (I_matrix * omega_frame + L_wheels)) - tau_gravity;
                 
        % E. SMC Switching Control (tau_sw)
        tau_sw = -K .* tanh(S / epsilon);
        
        % F. Total Torque (Newton's Third Law flipped)
        tau_frame = tau_eq + tau_sw;
        tau_motor = -tau_frame;

        % G. Motor Saturation
        max_torque = 1.2;
        tau_x = max(min(tau_motor(1), max_torque), -max_torque);
        tau_y = max(min(tau_motor(2), max_torque), -max_torque);
        tau_z = max(min(tau_motor(3), max_torque), -max_torque);
    end 
end
