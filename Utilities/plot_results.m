%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAE 502 Vehicle Control Systems Project 
% Formula 1 car racing: subfunctions
%   Copyright (C)2025 Chaozhe He. All Rights Reserved.
%   Author: Prof. Chaozhe He
%           Department of Mechanical and Aerospace Engineering
%           SUNY University at Buffalo
%           March 2025
% Any issues/bug reports,
% please email to chaozheh@buffalo.edu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Result plot function
% uncomment the following if you want to directly plot results.
%% Car information
% load('F1CarData.mat')
% Car=CarParameter;
%% Track Information
% load('CircuitOfAmerica.mat');

%% if you want to plot results
% load("Sample_path_run.mat")
%% Show the trajectories

L=[];
Height=8;
Width=7;
FontSize=16;
showzoom=1;
figure(101);clf;
figureset(gcf, 'Height',Height,'Width',Width);
figure(101);
subplot(4,1,1)
plot(t,y(:,1), 'b');
L=[L,ylabel('$x$[m]', "Rotation", 0)];
subplot(4,1,2)
plot(t,y(:,2), 'b');
L=[L,ylabel('$y$[m]', "Rotation", 0)];
subplot(4,1,3)
plot(t,y(:,3), 'b');
L=[L,ylabel('$\psi$[rad]', "Rotation", 0)];
subplot(4,1,4)
hold on;box on;
plot(t,y(:,4), 'b');
L=[L,ylabel('$v$[m/s]', "Rotation", 0)];L=[L,xlabel('$t$[sec]')];
%%
RR=u(:,1);
sss=u(:,5);
nnn=u(:,8);
nnnf=u(:, 9);
nnnr=u(:, 10);
gamma_actual=y(:,5);
gamma_dot_actual=u(:,2);
[Ffl_ana,Frl_ana]=Force_rwd(y(:,4), u(:,1) - Car.k * y(:,4).^2, y(:,5), u(:,2),...
                            Car.m,Car.m0,Car.b,Car.w);   

%%
figure(102);clf;
figureset(gcf, 'Height',Height,'Width',Width);
subplot(5,1,1);hold on;grid on; box on;
plot(t,RR, 'b');
plot(t, t*0 + Car.R_max, 'k--');
plot(t, t*0 + Car.R_min, 'k--');
L=[L,ylabel('$R$[N]', "Rotation", 0)];
subplot(5,1,2);hold on;grid on; box on;
plot(t,sss, 'b');
L=[L,ylabel('$s$[m]', "Rotation", 0)];
subplot(5,1,3);hold on; grid on; box on;
plot(t,nnn, 'b');
plot(t,nnnf, "r--");
plot(t,nnnr,"g-.");
n_boundary = Track.fun_width(sss)/2;
plot(t,n_boundary, 'k--', "LineWidth", 1);
plot(t,-n_boundary, 'k--', "LineWidth", 1);
L=[L,ylabel('$n$[m]', "Rotation", 0)];
subplot(5,1,4); hold on;grid on; box on;
plot(t, gamma_actual, 'b');
if exist("Inputs", "var")
    plot(t, Inputs.gamma(t), 'r--');
end
plot(t, t*0 + Car.gamma_max, 'k--');
plot(t, t*0 + Car.gamma_min, 'k--');

L=[L,ylabel('$\gamma$[rad]', "Rotation", 0)];
subplot(5,1,5); hold on;grid on; box on;
plot(t,gamma_dot_actual, 'b');
if exist("Inputs", "var")
    plot(t, Inputs.gamma_dot(t), 'r--');
end
plot(t, t*0 + Car.gamma_dot_max, 'k--');
plot(t, t*0 + Car.gamma_dot_min, 'k--');
if exist("gamma_dot_limit", "var")
    plot(t, gamma_dot_limit, 'm--');
end
L=[L,ylabel('$\dot{\gamma}$[rad/s]', "Rotation", 0)];

L=[L,xlabel('$t$[sec]')];
axleHandles = findobj(gcf, 'Type', 'axes');
linkaxes(axleHandles, 'x');

figure(103);clf;
figureset(gcf, 'Height',Height,'Width',Width);
subplot(5,1,1);hold on; grid on; box on;
plot(sss,RR, 'b');
plot(sss, sss*0 + Car.R_max, 'k--');
plot(sss, sss*0 + Car.R_min, 'k--');
L=[L,ylabel('$R$[N]', "Rotation", 0)];
subplot(5,1,2);hold on; grid on; box on;
plot(sss,Ffl_ana, "b"); 
L=[L,ylabel('$F_{\rm F}$[N]', "Rotation", 0)];
plot(sss, Car.Ffl_max + t*0, 'k--');
plot(sss, -Car.Ffl_max + t*0, 'k--');
subplot(5,1,3);hold on; grid on; box on;
plot(sss,Frl_ana, "b");
plot(sss, Car.Frl_max + t*0, 'k');
plot(sss, -Car.Frl_max + t*0, 'k');
L=[L,ylabel('$F_{\rm R}$[N]', "Rotation", 0)];
subplot(5,1,4);hold on; grid on; box on;
plot(sss,y(:,4), 'b');
if length(u(1, :)) > 10
    plot(sss, u(:, 11), "r--");
    legend("Actual", "Desired");
end
L=[L,ylabel('$v$ [m/s]', "Rotation", 0)];
subplot(5,1,5);hold on; grid on; box on;
plot(sss,y(:,5), 'b');
if length(u(1, :)) > 11
    plot(sss, u(:, 12), "r--");
    legend("Actual", "Desired");
end
L=[L,ylabel('$\gamma$ [m/s]', "Rotation", 0)];
L=[L,xlabel('$s$[m]')];
axleHandles = findobj(gcf, 'Type', 'axes');
linkaxes(axleHandles, 'x');
%% Show route together with track
figure(3);clf;
hold on;box on;axis equal;
plot3(Track.bl(1,:),Track.bl(2,:),Track.bl(3,:),'k-','LineWidth',1);
plot3(Track.br(1,:),Track.br(2,:),Track.br(3,:),'k-','LineWidth',1);
plot3(Track.cline(1,:),Track.cline(2,:),Track.cline(3,:),'k--','LineWidth',1);
plot(y(:,1),y(:,2),'LineWidth',2);
L=[L,ylabel('$y$[m]', "Rotation", 0)];
L=[L,xlabel('$x$[m]')];
%%
set(L,'Interpreter','latex');
