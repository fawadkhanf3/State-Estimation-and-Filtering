clear;close all;clc

load inputs.mat

t_ref  = x(1,:); % sec
p_des  = x(2,:); % deg
p_meas = x(3,:); % deg

simulation.Fs              = 40;
simulation.stopTime        = 60;

servoModel.K_d    = 8.45;
servoModel.dP     = 29;
servoModel.S      = 5800;
servoModel.dP_ref = 33.5;
servoModel.K      = 0.6;

x1 = [t_ref(:),p_des(:)];
x2 = [t_ref(:),p_meas(:)];

L1 = 1;
L2 = -0.5;