%% Kalman Filter (Example 02)

A = -1; B = 1; C = 1; D = 0;
delay_T = 0; Ts = 0.01;
t = [0:Ts:10]'; u = 10*square(t);
n = length(t);
Q = 1; R=1;
rng default
v = sqrt(5)*randn(n,1);
sys = ss(A,B,C,D);
sys_del = sys;
sys_del = c2d(sys_del,Ts,'foh');
[A,B,C,D] = ssdata(sys_del);
s = tf('s');
y = lsim(sys*exp(-s*delay_T),u,t,-20);      
yv = y + v; ord = length(A);
P = 100*eye(ord); % Initial error covariance
x = zeros(ord,1);     % Initial condition on the state
ye = zeros(length(t),1); ycov = zeros(length(t),1); 

for i = 1:length(t)
  % Measurement update
  K = P*C'/(C*P*C'+R);
  x = x + K*(yv(i)-C*x);   % x[n|n]
  P = (eye(ord)-K*C)*P;      % P[n|n]

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
legend('True output','Estimated output','Input',...
'Measured output','Location','SouthEast');shg;