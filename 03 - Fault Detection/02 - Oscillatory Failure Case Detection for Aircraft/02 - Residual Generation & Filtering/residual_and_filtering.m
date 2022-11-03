clear;close all;clc

load inputs.mat

t_ref  = x(1,:); % sec
p_meas = x(3,:); % deg

x = [t_ref(:),p_meas(:)];

load estimations.mat

p_est = p_est.';

y1 = p_est(:,[1,2]);
y2 = p_est(:,[1,3]);

simulation.Fs              = 40;
simulation.stopTime        = 60;



