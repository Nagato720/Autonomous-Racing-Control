clear;clc;close all;
% %% Car information
load('F1CarData.mat')
Car=CarParameter;
sigma_max = sqrt(Car.R_max / Car.k);
%% Lateral Forces

% check constant steering force needed
% 
% [sigma,gamma] = meshgrid([0:1:floor(sigma_max), sigma_max], ...
%                          linspace(Car.gamma_max, Car.gamma_min, 200));

% gammadot = 0;
% R = 0;
% Ffl_ana = zeros(length(sigma(:,1)), length(sigma(1,:)));
% Frl_ana = zeros(length(sigma(:,1)), length(sigma(1,:)));
% for i = 1:length(sigma(:,1))
%     for j = 1:length(sigma(1,:))
%         [Ffl_ana(i, j), Frl_ana(i, j)] = Force_rwd(sigma(i, j), R, gamma(i, j), gammadot,Car.m,Car.m0,Car.b,Car.w);   
%     end
% end

% Ffl_ana(abs(Ffl_ana) > Car.Ffl_max) = NaN;
% Frl_ana(abs(Frl_ana) > Car.Frl_max) = NaN;
% figure;
% subplot(1, 2, 1);
% surf(sigma, gamma, Frl_ana);
% xlabel('longitudinal speed [m/s]');
% ylabel('steering angle [rad]');
% zlabel('Frl');
% title("front")
% view(2)
% colormap default
% colorbar
% subplot(1, 2, 2);
% surf(sigma, gamma, Ffl_ana);
% xlabel('longitudinal speed [m/s]');
% ylabel('steering angle [rad]');
% zlabel('Ffl');
% view(2)
% colormap default
% colorbar

%%




% sigma_max_front = zeros(1, length(gamma_can));
% sigma_max_rear = zeros(1, length(gamma_can));
% for i = 1:length(gamma_can)
%     [sigma_max_front(i), sigma_max_rear(i)] = Force_rwd_cornering(gamma_can(i), Car.Ffl_max, Car.Frl_max, Car.m, Car.m0, Car.b, Car.w);
% end
% figure; clf; hold on; grid on; box on;
% plot(gamma_can, sigma_max_front, 'r', 'LineWidth', 2);
% plot(gamma_can, sigma_max_rear, 'b--', 'LineWidth', 2);
% xlabel('steering angle [rad]');
% ylabel('longitudinal speed [m/s]');
% legend('front', 'rear');
% title('Maximum longitudinal speed vs. steering angle');
% hold off;

%%
func_Frl = @(sigma,R,gamma,gammadot,m,m0,d,L) ...
    -m/L*(1-d/L)*tan(gamma).*sigma.^2 ...
     +1/L./(m*(cos(gamma)).^2+m0*(sin(gamma)).^2)...
    .*(m*(m0*L-m*d)*gammadot.*sigma ...
      +(m0*L-m*d)*R.*sin(gamma).*cos(gamma)) ;
  
func_Ffl = @(sigma,R,gamma,gammadot,m,m0,d,L) ...
       -m*d/L^2*tan(gamma)./cos(gamma).*sigma.^2 ...
       -m*m0*gammadot.*sigma./(m*(cos(gamma)).^2+m0*(sin(gamma)).^2)./(cos(gamma)) ...
       -m0*R.*sin(gamma)./(m*(cos(gamma)).^2+m0*(sin(gamma)).^2);

Force_rwd_cornering_front = @(gamma,Ffl_max, Frl_max, m, ~, d, L) sqrt(Ffl_max / (m*d/L^2*tan(gamma)/cos(gamma)));
Force_rwd_cornering_rear = @(gamma,Ffl_max, Frl_max, m, ~, d, L) sqrt(Frl_max / (m/L*(1-d/L)*tan(gamma)));

gamma_can = linspace(Car.gamma_max/200, Car.gamma_max, 200);
gammadot = 0;


R = Car.R_max;
sigma_max_front_max = gamma_can;
sigma_max_rear_max = gamma_can;
sigma_max_front_min = gamma_can;
sigma_max_rear_min = gamma_can;
sigma_max_front = zeros(1, length(gamma_can));
sigma_max_rear = zeros(1, length(gamma_can));

sigma_max_front_R_max = gamma_can;
sigma_max_rear_R_max = gamma_can;

Ffl_coeff_R = @(gamma, m,m0,d,L) +1/L./(m*(cos(gamma)).^2+m0*(sin(gamma)).^2)...
    .*( (m0*L-m*d)*sin(gamma).*cos(gamma));
Ffl_coeff_gammadot = @(gamma, sigma, m,m0,d,L) 1/L./(m*(cos(gamma)).^2+m0*(sin(gamma)).^2)...
    .*(m*(m0*L-m*d)*sigma );


Ffl_state = @(sigma,R,gamma,gammadot) func_Ffl(sigma,R - Car.k * sigma^2, gamma,gammadot, Car.m, Car.m0, Car.b, Car.w);
Frl_state = @(sigma,R,gamma,gammadot) func_Frl(sigma,R - Car.k * sigma^2, gamma,gammadot, Car.m, Car.m0, Car.b, Car.w);

for i = 1:length(gamma_can)
    sigma_max_front(i) = Force_rwd_cornering_front(gamma_can(i), Car.Ffl_max, Car.Frl_max, Car.m, Car.m0, Car.b, Car.w);
    sigma_max_rear(i) = Force_rwd_cornering_rear(gamma_can(i), Car.Ffl_max, Car.Frl_max, Car.m, Car.m0, Car.b, Car.w);
    Frl_f0 = @(x) -func_Frl(x, Car.R_max - Car.k * x^2, gamma_can(i), gammadot, Car.m, Car.m0, Car.b, Car.w) - Car.Frl_max;
    sigma_max_rear_max(i) = fsolve(Frl_f0, sigma_max_rear(i));
    Ffl_f0 = @(x) -func_Ffl(x, Car.R_max - Car.k * x^2, gamma_can(i), gammadot, Car.m, Car.m0, Car.b, Car.w) - Car.Ffl_max;
    sigma_max_front_max(i) = fsolve(Ffl_f0, sigma_max_front(i)); 

    Frl_f0_min = @(x) func_Frl(x, Car.R_min - Car.k * x^2, gamma_can(i), gammadot, Car.m, Car.m0, Car.b, Car.w) - Car.Frl_max;
    sigma_max_rear_min(i) = fsolve(Frl_f0, sigma_max_rear(i));
    Ffl_f0_min = @(x) func_Ffl(x, Car.R_min - Car.k * x^2, gamma_can(i), gammadot, Car.m, Car.m0, Car.b, Car.w) - Car.Ffl_max;
    sigma_max_front_min(i) = fsolve(Ffl_f0, sigma_max_front(i)); 

    % Frl_fmax = @(x) -func_Frl(x, Car.R_max, gamma_can(i), gammadot, Car.m, Car.m0, Car.b, Car.w) - Car.Frl_max;
    % sigma_max_rear_R_max(i) = fsolve(Frl_fmax, sigma_max_rear(i));
    % Ffl_fmax = @(x) -func_Ffl(x, Car.R_max, gamma_can(i), gammadot, Car.m, Car.m0, Car.b, Car.w) - Car.Ffl_max;
    % sigma_max_front_R_max(i) = fsolve(Ffl_fmax, sigma_max_front(i)); 
end
%%
figure(1); clf; hold on; grid on; box on;
plot(gamma_can, sigma_max_front, 'r', 'LineWidth', 2);
plot(gamma_can, sigma_max_rear, 'b--', 'LineWidth', 2);
plot(gamma_can, sigma_max_front_max, 'm:', 'LineWidth', 2);
plot(gamma_can, sigma_max_rear_max, 'c-.', 'LineWidth', 2);
plot(gamma_can, sigma_max_front_min, 'g:', 'LineWidth', 2);
plot(gamma_can, sigma_max_rear_min, 'y-.', 'LineWidth', 2);
% plot(gamma_can, sigma_max_front_R_max, 'g:', 'LineWidth', 2);
% plot(gamma_can, sigma_max_rear_R_max, 'y-.', 'LineWidth', 2);
plot(gamma_can, gamma_can *  0 + sigma_max, "k")
xlabel('steering angle $\gamma$ [rad]');
ylabel('longitudinal speed $\sigma$[m/s]');
legend('front', 'rear', ...
     ['front max, R = ' num2str(Car.R_max, '%d') ', $\dot{\gamma}$ = ' num2str(gammadot, '%.1f')], ...
    ['rear max, R = ' num2str(Car.R_max, '%d') ', $\dot{\gamma}$ = ' num2str(gammadot, '%.1f')], ...
    ['front min, R = ' num2str(Car.R_min, '%d') ', $\dot{\gamma}$ = ' num2str(gammadot, '%.1f')], ...
    ['rear min, R = ' num2str(Car.R_min, '%d') ', $\dot{\gamma}$ = ' num2str(gammadot, '%.1f')], ...
    'Interpreter', 'latex');
title('Maximum longitudinal speed vs. steering angle');
hold off;
