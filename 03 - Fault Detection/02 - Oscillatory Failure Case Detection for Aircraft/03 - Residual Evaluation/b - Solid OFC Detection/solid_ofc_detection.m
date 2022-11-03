clear;close all;clc

load solid_residual.mat

x = x.';

Threshold    = 0.1;
MaxCrossings = 3;

tp = 1/2/pi;
simulation.Fs              = 40;
simulation.stopTime        = 60;

res = x(:,[1,2]);
est = x(:,[1,3]);

