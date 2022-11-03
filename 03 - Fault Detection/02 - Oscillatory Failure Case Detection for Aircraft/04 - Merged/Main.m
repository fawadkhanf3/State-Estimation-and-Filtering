clear;close all;clc

addpath('Test Cases');

flag = 13;

switch (flag)
    case 1
        load nominal.mat
    case 2
        load light_turbulence.mat
    case 3
        load moderate_turbulence.mat
    case 4
        load severe_turbulence.mat
    case 5
        load NZ_Step.mat
    case 6
        load NZ_Sine.mat
    case 7
        load NZ_Chirp.mat
    case 8
        load NZ_Step_with_Severe_Turbulence.mat
    case 9
        load sensor_liquid_ofc.mat
    case 10
        load sensor_solid_ofc.mat
    case 11
        load current_liquid_ofc.mat
    case 12
        load current_solid_ofc.mat
    otherwise
        load solid_ofc_energycase.mat
end

x = x.';

delta_des  = x(:,[1,2]);
delta_meas = x(:,[1,3]);

simulation.Fs              = 40;
simulation.stopTime        = 60;

servoModel.K_d    = 8.45;
servoModel.dP     = 29;
servoModel.S      = 5800;
servoModel.dP_ref = 33.5;
servoModel.K      = 0.6;

L1 = 1;
L2 = -0.5;

tp = 1/(2*pi);
 
TE = 50;
Threshold    = 0.1;
MaxCrossings = 3;

out1 = sim('analytical_sim',simulation.stopTime);
out2 = sim('observer_sim',simulation.stopTime);

t1 = out1.simout.time;
d1 = out1.simout.data;

t2 = out2.simout.time;
d2 = out2.simout.data;

figure(1);hold on;grid on;box on
plot(t1,d1,'r.-');
plot(t2,d2,'b.-');

rmpath('Test Cases');