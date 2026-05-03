%% MAE502 Main Simulation File [DERBY]
close all
clear all
clc

%% Car & Track Information
addpath("Utilities"); 
load('F1CarData.mat')
Car = CarParameter; 
load('CircuitOfAmerica.mat', 'Track');
s_start = 0;

load("Derby_Path.mat","Path"); % Closed-Loop path that works (faster)

%% ICs
pos0 = Track.center(s_start);
v0 = Path.v(s_start); % Start at optimized path target speed
gamma0 = 0; 

% Add z_v and z_gamma Integrator States for CL Controllers
x0 = [pos0(1:2);Track.ftheta(s_start);v0;gamma0;0;0]; 

%% Controllers
% Speed Control (Discrete PID)
Kp_speed = 800; 
Ki_speed = 0;
Kd_speed = 175; 
    
R_control = @(t,x,y,psi,sigma,gamma,z_v,s0,sf,sr,n0,nf,nr)...
    Speed_control(t,x,y,psi,sigma,gamma,z_v,s0,sf,sr,n0,nf,nr,Path,Car,...
    Kp_speed,Ki_speed,Kd_speed);

% Steering Control (Discrete LQR)
gamma_dot_control = @(t,x,y,psi,sigma,gamma,z_gamma,s0,sf,sr,n0,nf,nr)...
    Steering_control(t,x,y,psi,sigma,gamma,s0,n0,Path,Car);

%% Dynamics & Simulation 
car_dynamics = @(t,x,y,psi,sigma,gamma,z_v,z_gamma,s)...
    car_RWD_with_control(t,x,y,psi,sigma,gamma,z_v,z_gamma,...
    gamma_dot_control,R_control,Track,Car,s);
                                            
sys = @(t,x,para) car_dynamics(t,x(1),x(2),x(3),x(4),x(5),x(6),x(7),para);
Time = 250;
sim_step = 0.05; 
usize = 12;
Animation = 0;

%% Run Simulation
[t,y,u,TotalTime,Num_of_violation] = CarSimRealTime(sys,[0 Time],x0,...
    s_start,sim_step,usize,Track,Car,Animation,0);

% Print Results 
if any(Num_of_violation ~= 0)
    fprintf('# of Violations: %d Off Track, %d Front Force, %d Rear Force \n\n',...
        Num_of_violation(1),Num_of_violation(2),Num_of_violation(3));
else
    fprintf('\nFinished without violation. Lap Time: %.2f\n\n',TotalTime);
end

% Show Path
plot_results;
% keyboard 

TeamName = "The_Derbmobile";
save("The_Derbmobile.mat",'t','y','u',"Num_of_violation" ,"TotalTime","TeamName");