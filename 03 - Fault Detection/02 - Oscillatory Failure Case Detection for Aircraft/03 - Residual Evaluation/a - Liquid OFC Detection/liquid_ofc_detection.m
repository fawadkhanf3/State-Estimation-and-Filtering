clear;close all;clc

load liquid_residual.mat
x = x.';

Threshold    = 0.1;
MaxCrossings = 3;

tp = 1/2/pi;
simulation.Fs              = 40;
simulation.stopTime        = 60;