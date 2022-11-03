%% Kalman Filter (Example 03 - with Delay)

close all;clear all;clc

% Plant Dynamics

% G = 1/(s-a)
% State space formulation gives 
    % xdot = ax+u
    % y = x
    % A = [a], B=1, C=1
    % we take a = -1
    % T = 1 (maximum value) ; we'll take 0,0.1,0.5,1
    % dt = 0.01 (integration time for the simulator)
    % u = square signal with a magnitude of 10 and a period of 2pi
    % x0 = 20,x^0 = 0 (xhat0 = Initial Estimate)

A = -1;
B = 1;
C = 1;
D = 0;

delay_T = 1;
Ts = 0.01;
t = [0:Ts:10]';
u = 10*square(t);
n = length(t);
Q = 1; R=1;
rng default
%w = sqrt(1)*randn(n,1);
v = sqrt(5)*randn(n,1);
sys = ss(A,B,C,D);

s = tf ('s');
[num,den]=pade(delay_T,2);
del_pade=tf(num,den);
sys_del=sys*del_pade;

sys_del = c2d(sys_del,Ts,'foh');
[A,B,C,D]=ssdata(sys_del);
y = lsim(sys*exp(-s*delay_T),u,t,-20);      
yv = y + v;
ord=length(A);
P = 100*eye(ord);%*B*Q*B';         % Initial error covariance
x = zeros(ord,1);     % Initial condition on the state
ye = zeros(length(t),1);
ycov = zeros(length(t),1); 

for i = 1:length(t)
  % Measurement update
  Mn = P*C'/(C*P*C'+R);
  x = x + Mn*(yv(i)-C*x);   % x[n|n]
  P = (eye(ord)-Mn*C)*P;      % P[n|n]

  ye(i) = C*x;
  errcov(i) = C*P*C';

  % Time update
  x = A*x + B*u(i);        % x[n+1|n]
  P = A*P*A' + B*Q*B';     % P[n+1|n]
end

%%
 plot(t,y,'r--',t,ye,'b',t,u,'y',t,yv,'k-',...
 t,ye+3*errcov','b.',t,ye-3*errcov','b.')
title('Time-varying Kalman filter response')
xlabel('No. of samples'), ylabel('Output')
legend('True output','Estimated output','Input','Measured output','Location','SouthEast')
shg;%axis ([0 1 -40 50])
% print -depsc -tiff -r50 -painters pade.eps