% Fault Detection of a Quadrotor with Nonlinaer State Estimation using 
% Constrained Zonotopes

clear;close all;clc
format long g
set(0,'DefaultLineLineWidth',2);

rng(8); % for reproducibility

%%
fp = struct;
fp = constants(fp);
fp = navigation_parameters(fp);
fp = guidance_parameters(fp);
fp = controller_parameters(fp);
fp = nse_parameters(fp);

% Initial Conditions
t0    = -fp.T;
x0    = [0;0;0;pi/3;0;0;0;0;0;0;0;0];
zeta0 = 0.1;
xi0   = 0.1;
tf    = 30;

c  = zeros(fp.n,1);
G  = blkdiag(2,2,2,pi/2,pi/6,pi/6,1,1,1,pi/12,pi/12,pi/12);
X0 = conZonotope(c,G,[],[]);

% Simulation

tk = tic;
[t,y,bounds,fault] = simulate_quadrotor(fp,[t0,tf],x0,zeta0,xi0,X0);
toc(tk);

idx = find(t>0,1,'Last');
t = t(1:idx);
y = y(1:idx,:);
bounds = bounds(1:idx,:,:);

figure(1);hold on;grid on;box on
plot3(fp.xref(t),fp.yref(t),fp.zref(t));
plot3(y(1:idx,1),y(1:idx,2),y(1:idx,3));
view(3);

for i = 1:9
    figure(i+1);hold on;grid on;box on
    plot(t,y(:,i),'r.-');
    plot(t(3:end),bounds(3:end,i,1),'m.-');
    plot(t(3:end),bounds(3:end,i,2),'c.-');
end

for i = 10:12
    figure(i+1);hold on;grid on;box on
    plot(t,y(:,i+2),'r.-');
    plot(t(3:end),bounds(3:end,i,1),'m.-');
    plot(t(3:end),bounds(3:end,i,2),'c.-');
end
