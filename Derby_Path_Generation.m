%% MAE502 Trajectory Optimization [DERBY]
close all
clear
clc

%% Load Information
addpath("Utilities")

% Car
load('F1CarData.mat')
Car = CarParameter; 

% Track
load('CircuitOfAmerica.mat');
s_start = 0; 
i0 = 11;  % Start index
iF = 587; % Finish index
total_indexes = length(Track.arc_s);
is_loop = true;
save_name = "Derby_Path";
speed_scale = 0.94; % 0.94 works 
s_sam = Track.arc_s;
theta = Track.theta;

%% Define Racing Line Waypoints 
% Define waypoints to widen the corners
% format: [s_coordinate, n_offset]
% Positive n is LEFT of centerline, Negative n is RIGHT.
waypoints = [
   s_sam(1),0;  % Initialize "before" start 
      0,    0;  % True start
      
    % Turn 1
    250,  -4.0;  % Turn 1 Entry (Swing Right)
    320,   4.0;  % Turn 1 (Left Turn)
    450,  -4.0;  % Turn 1 Exit (Swing Right)
    500,     0;  % Straighten
  
    % Turn 2
    534,   4.0;  % Turn 2 Entry (Swing Left)
    634,  -4.0;  % Turn 2 (Right Turn) 
    734,   4.0;  % Turn 2 Exit (Swing Left) In more to avoid track violation
    784,     0;  % Straighten
    
    % Turn 3
    898,   4.0;  % Turn 3 Entry (Swing Left)     
    998,  -4.0;  % Turn 3 (Right Turn)
    1050,  4.0;  % Turn 3 Exit (Swing Left)
    
    % Turn 4  
    1265, -4.0;  % Turn 4 (Right Turn) 

    % Turn 5
    1434,  4.0;  % Turn 5 (Left Turn)
    
    % Turn 6
    1554,  4.0;  % Turn 6 Entry (Swing Left)     
    1654, -4.0;  % Turn 6 (Right Turn)
    1700,  4.0;  % Turn 6 Exit (Swing Left)
    
    % Turn 7
    1792, -4.0;  % Turn 7 Entry (Swing Right)     
    1892,  4.0;  % Turn 7 (Left Turn)
    1992,    0;  % Straighten
    
    % Turn 8 
    2092, -4.0;  % Turn 8 Entry (Swing Right)     
    2292,  4.0;  % Turn 8 (Left Turn)
    2392, -4.0;  % Turn 8 Exit (Swing Right)
    
    2492,    0;  % Snap to center for the long straight
    3400,    0;  % Stay centered until Turn 9 approach
    3450, -4.0;  % Turn 9 Entry (Swing Right to prep for Left Turn)
    
    % Turn 9
    3491,  4.0;  % Turn 9 (Left Turn)
    3591, -4.0;  % Turn 9 Exit (Swing Right)
   
    % Turn 10
    3640,  4.0;  % Turn 10 Entry (Swing Left)     
    3740, -4.0;  % Turn 10 (Right Turn)
  
    % Turn 11
    3895, -4.0;  % Entry (Keep Right)
    3995,  4.0;  % Turn Left
    4095, -4.0;  % Exit (Swing Right)
 
    % Turn 12
    4149,  4.0;  % Entry (Swing Left)
    4249, -4.0;  % Turn Right
    4500, -4.0;  % Keep Right
    
    % Turn 13  
    4753,  4.0;  % Turn Left
    4853, -4.0;  % Exit (Swing Right)
    
    % Turn 14
    4957, -4.0;  % Entry (Swing Right)     
    5057,  4.0;  % Turn Left
    5157, -4.0;  % Exit (Swing Right)
    5400,    0;  % Straighten 
    
    s_sam(end), 0  % End
];

% Interpolate to create a continuous n_path array
% 'makima' prevents the curve from overshooting on the straightaways
n_path = makima(waypoints(:,1),waypoints(:,2),s_sam);

%% Generate Absolute Path Coordinates
X = Track.cline;
X_path = zeros(3, total_indexes);
for i = 1:total_indexes
    % Rotation matrix for 2D track orientation
    R = [cos(theta(i)), -sin(theta(i)), 0;
         sin(theta(i)),  cos(theta(i)), 0;
         0,              0,             1];
    
    % Shift the centerline point by the lateral offset n
    X_path(:,i) = X(:,i) + R * [0; n_path(i); 0];
end

%% Estimate Curvature of New Optimized Track
% Grab x & y coordinates of the new path
x_new = X_path(1,:);
y_new = X_path(2,:);

% Calculate the first & second derivatives
dx = gradient(x_new);
dy = gradient(y_new);
ddx = gradient(dx);
ddy = gradient(dy);

% Calculate the new curvature kappa
curvature_raw = abs(dx.*ddy - dy.*ddx)./((dx.^2 + dy.^2).^(3/2));
curvature_raw(isnan(curvature_raw)) = 0; % Eliminate potential NaNs

% Smooth the numerical derivative noise heavily
if is_loop
    % Pad array with 50 elements from the end & beginning to wrap smoothly
    N_pad = 50; 
    padded_raw = [curvature_raw(end-N_pad+1:end),curvature_raw,curvature_raw(1:N_pad)];
    
    % Filter the padded array
    padded_smooth = sgolayfilt(padded_raw,3,31);
    
    % Crop the padded sections off to get back exactly to total_indexes length
    curvature = padded_smooth(N_pad+1:N_pad+total_indexes);
else
    curvature = sgolayfilt(curvature_raw,3,31); 
end

%% Generate Speed Trajectory 
sigma_max = sqrt(Car.R_max/Car.k);
Force_rwd_cornering_front = @(gamma,Ffl_max,Frl_max,m,~,d,L) sqrt(Ffl_max/(m*d/L^2*tan(gamma)/cos(gamma)));
Force_rwd_cornering_rear = @(gamma,Ffl_max,Frl_max,m,~,d,L) sqrt(Frl_max/(m/L*(1-d/L)*tan(gamma)));

v_ref_raw = zeros(1, total_indexes);
for i = 1:total_indexes
    gamma = atan(Car.w * abs(curvature(i)));
    
    % Protect against dividing by zero on perfectly straight sections
    if gamma < 1e-4
        v_ref_raw(i) = sigma_max;
    else
        v_ref_raw(i) = min(Force_rwd_cornering_front(gamma, Car.Ffl_max, Car.Frl_max, Car.m, Car.m0, Car.b, Car.w),...
                       Force_rwd_cornering_rear(gamma, Car.Ffl_max, Car.Frl_max, Car.m, Car.m0, Car.b, Car.w));
    end
end

% Scale & cap speed
v_ref_raw = v_ref_raw*speed_scale;
v_ref_raw(v_ref_raw > sigma_max) = sigma_max;

% Format: [start_s  end_s], speed_limit
% "Step-Downs" for heavy braking zones.
manual_sections = [
    [0 10],      75;
    [335 365],    3;  
    [518 550],   15;
    [728 748],   37;
    [830 890],   25;  
    [941 981],   23;
    [1027 1067], 23;
    [1133 1173], 25;
    [1258 1311], 25;
    [1396 1436], 10;
    [1529 1569], 20;
    [1655 1695], 15;
    [1860 1890], 15;
    [2250 2300],  7;  
    [2975 3015], 50;
    [3198 3218], 62;
    [3457 3497],  7;
    [3617 3647], 25;
    [3692 3722], 10;
    [3799 3819], 10;   
    [3930 3960], 15; 
    [3994 4014],  7;
    [4129 4159], 30;
    [4189 4209], 25;
    [4396 4416], 25;   
    [4305 4325], 13;
    [4396 4416], 25;
    [4513 4533], 25;
    [4737 4767], 14;
    [5026 5066], 10;
];

v_ref = v_ref_raw;

for i = 1:size(manual_sections, 1)
    s_sec_start = manual_sections(i, 1);
    s_sec_end = manual_sections(i, 2);
    v_ref_section = manual_sections(i, 3);

    start_idx = find(s_sam >= s_sec_start, 1);
    end_idx = find(s_sam <= s_sec_end, 1, 'last');

    % Apply the limit safely
    v_ref(start_idx:end_idx) = min(v_ref_section, v_ref_raw(start_idx:end_idx));
end

% Dynamically calculate physical limits from the car's data (w/ 5% margin)
a_max_braking = (abs(Car.R_min)/Car.m)*0.95; 
a_max_accel = (Car.R_max/Car.m)*0.95;

% Backward sweep to smooth deceleration
for sweep = 1:2
    for i = (total_indexes-1):-1:1
        ds = s_sam(i+1) - s_sam(i);
        v_safe = sqrt(v_ref(i+1)^2 + 2*a_max_braking*ds);
        v_ref(i) = min(v_ref(i), v_safe);
    end
    if is_loop
        ds_wrap = s_sam(end) - s_sam(end-1);
        v_safe_wrap = sqrt(v_ref(1)^2 + 2*a_max_braking*ds_wrap);
        v_ref(end) = min(v_ref(end), v_safe_wrap);
    end
end

% Forward sweep to smooth acceleration
for sweep = 1:2
    for i = 2:total_indexes
        ds = s_sam(i) - s_sam(i-1);
        v_safe = sqrt(v_ref(i-1)^2 + 2*a_max_accel*ds);
        v_ref(i) = min(v_ref(i), v_safe);
    end
    if is_loop
        ds_wrap = s_sam(2) - s_sam(1);
        v_safe_wrap = sqrt(v_ref(end)^2 + 2*a_max_accel*ds_wrap);
        v_ref(1) = min(v_ref(1), v_safe_wrap);
    end
end

%% Pack the Path Structure
% Calculate the tangent vectors of the new path
deltaXpath = X_path(:,2:end) - X_path(:,1:end-1);
deltaX = sqrt(deltaXpath(1,:).^2 + deltaXpath(2,:).^2 + deltaXpath(3,:).^2);
t_path = [deltaXpath(1,:)./deltaX; deltaXpath(2,:)./deltaX; deltaXpath(3,:)./deltaX];
t_path = [t_path, t_path(:,end)];   

Path.n = @(s) interp1(s_sam, n_path, s, 'linear', 'extrap');
Path.X = @(s) [interp1(s_sam, X_path(1,:), s, 'linear', 'extrap');
               interp1(s_sam, X_path(2,:), s, 'linear', 'extrap');
               interp1(s_sam, X_path(3,:), s, 'linear', 'extrap')];
Path.t = @(s) [interp1(s_sam, t_path(1,:), s, 'linear', 'extrap');
               interp1(s_sam, t_path(2,:), s, 'linear', 'extrap');
               interp1(s_sam, t_path(3,:), s, 'linear', 'extrap')];   
Path.v = @(s) interp1(s_sam, v_ref, s, 'linear', 'extrap');

%% Plot Results
figure(1)
clf
hold on
axis equal
box on
plot3(Track.bl(1,:), Track.bl(2,:), Track.bl(3,:), 'k', 'Markersize', 5)
plot3(Track.br(1,:), Track.br(2,:), Track.br(3,:), 'k', 'Markersize', 5)
plot3(Track.cline(1,:), Track.cline(2,:), Track.cline(3,:), 'k--', 'Markersize', 5)
plot3(X_path(1,:),X_path(2,:),v_ref,'m','linewi',1.5) % Plot speed profile in Z-axis
title('Optimized Racing Line & Speed Profile')

% Label s-coordinates on the track map
hold on
% Place a text label every 250 meters along the centerline
for s_label = 0:250:5500
    idx = find(s_sam >= s_label, 1);
    x_pos = Track.cline(1, idx);
    y_pos = Track.cline(2, idx);
    z_pos = 0; 
    text(x_pos, y_pos, z_pos, sprintf(' s=%d', s_label),'Color','red', ...
        'FontSize',10,'FontWeight','bold');
end

figure(2)
% clf
% subplot(2,1,1)
% hold on
% grid on
% box on
% plot(s_sam, curvature, 'b', 'LineWidth', 1)
% ylabel("$1/\rho$ [1/m]",'Interpreter','latex', "Rotation", 0)
% title('Path Curvature')
% subplot(2,1,2)
hold on
grid on
box on
plot(s_sam, v_ref, 'b', 'LineWidth', 1)
plot(s_sam, v_ref_raw * 0 + sigma_max, 'k--', 'LineWidth', 1)
ylabel('v [m/s]', "Rotation", 0)
xlabel('s [m]')
title('Target Velocity')

%% Save
save(save_name, "Path");