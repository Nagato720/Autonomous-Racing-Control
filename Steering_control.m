function [gamma_dot, gamma_desired, gamma_saturated] = ...
    Steering_control(t,x,y,psi,sigma,gamma,s0,n0,Path,Car)
    
    % Persistent memory for digital clock
    persistent t_prev gamma_dot_held gamma_desired_held gamma_saturated_held
    
    % Update only every 0.05 s
    if isempty(t_prev) || t == 0 || (t - t_prev) >= 0.05
            
        % Prevent Singularities
        v = max(sigma,1); 
        s_safe = min(max(s0,0),5514); 
        
        % State Error Relative to Optimized Path
        t_vec = Path.t(s_safe);
        theta_path = atan2(t_vec(2),t_vec(1));
        delta_psi = mod((psi - theta_path) + pi,2*pi) - pi;
        
        target_n = Path.n(s_safe);
        e_y = n0 - target_n; 
        state_error = [e_y; delta_psi];
        
        lookahead_time = 0.8; % [s]
        lookahead_dist = max(v*lookahead_time,2); 
        s_ahead = min(max(s_safe + lookahead_dist,0),5514); 
        
        t_ahead = Path.t(s_ahead);
        theta_ahead = atan2(t_ahead(2),t_ahead(1));
        dtheta = mod((theta_ahead - theta_path) + pi, 2*pi) - pi;
        kappa = dtheta/lookahead_dist; 
        gamma_ff = atan(Car.w*kappa); 
        
        % Weighting Matrices
        Q = diag([1,1]); 
        R = 550;           
        Kp_actuator = 5;   
        
        % Discrete-Time System Matrix
        dt = 0.05; 
        A_d = [1, v*dt; 0, 1];
        B_d = [(v^2*dt^2)/(2*Car.w); (v*dt)/Car.w]; 
             
        % Find Optimal K
        [K,~,~] = dlqr(A_d,B_d,Q,R);
        
        % Calculate Intermediate Targets
        calc_gamma_desired = -K*state_error + gamma_ff; 
        calc_gamma_saturated = min(max(calc_gamma_desired, Car.gamma_min), Car.gamma_max); 
        
        % Low-Level Actuator Control 
        calc_gamma_dot = Kp_actuator*(calc_gamma_saturated - gamma); 
        calc_gamma_dot = min(max(calc_gamma_dot, Car.gamma_dot_min), Car.gamma_dot_max);
        
        % Save calculations to memory
        gamma_dot_held = calc_gamma_dot; 
        gamma_desired_held = calc_gamma_desired;
        gamma_saturated_held = calc_gamma_saturated;
        
        t_prev = t; % Reset the clock
    end
    
    % Output Held Values
    gamma_dot = gamma_dot_held;
    gamma_desired = gamma_desired_held;
    gamma_saturated = gamma_saturated_held;
end