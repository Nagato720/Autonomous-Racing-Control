%% MAE502 Discrete PID Speed Controller [Derby]
function [R,v_desired,saturated] = Speed_control(t,x,y,psi,v,gamma,z_v,s0,...
    sf,sr,n0,nf,nr,path, Car, Kp, Ki, Kd)
  
    dt = 0.05; % Sampling period

    v_desired = path.v(sf);  
    e_v = v_desired - v; % Current speed error
    
    % Digital ECU Clock
    persistent t_prev e_v_prev R_held
     
    % Initialize ECU @ first simulation step
    if isempty(t_prev) || t == 0
        t_prev = t;
        e_v_prev = e_v;
        
        % Base PI calculation (no derivative on step 1)
        R_ff = Car.k*v_desired^2; 
        R_held = R_ff + (Kp*e_v) + (Ki*z_v); 
    end
    
    % Only run the PID math if a full 0.05s ECU tick has passed
    if (t - t_prev) >= (dt - 1e-4) % 1e-4 to account for floating point rounding
        
        dt = t - t_prev;
        d_ev = (e_v - e_v_prev)/dt; % Error derivative
        
        % PID Math + Aero Feedforward
        R_ff = Car.k*v_desired^2; 
        R_feedback = (Kp*e_v) + (Ki*z_v) + (Kd*d_ev);
        
        % Save the new force 
        R_held = R_ff + R_feedback;
        
        % Advance ECU clock
        t_prev = t;
        e_v_prev = e_v;
    end

    % Saturation
    if R_held > Car.R_max
        R = Car.R_max;
        saturated = 1;
    elseif R_held < Car.R_min
        R = Car.R_min;
        saturated = 1;
    else
        R = R_held;
        saturated = 0;
    end
end