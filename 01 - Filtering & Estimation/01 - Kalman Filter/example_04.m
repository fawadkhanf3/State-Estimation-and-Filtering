A = [1.1269   -0.4940    0.1129;
     1.0000         0         0;
          0    1.0000         0];  % F Matrix

B = [-0.3832;
      0.5919;
      0.5191];  % G Matrix

C = [1 0 0];    % H Matrix

t = [0:100]';
u = sin(t/5);   % Input

Q = 1;
R = 1;
 
% u+v = Noise = Process Noise + Measurement Noise

n = length(t);
rng('default')
w = sqrt(Q)*randn(n,1); 
v = sqrt(R)*randn(n,1);

sys = ss(A,B,C,0,-1);  % -1 means the system is discrete with unspecified time 
y = lsim(sys,u+w); % Linear Simulation
yv = y + v; % Measurement ??

figure(1);
plot(yv);
figure(2);
plot(t,yv);

P = B*Q*B';         % Initial error covariance
x = zeros(3,1);     % Initial condition on the state
ye = zeros(length(t),1); % Estimated Output
ycov = zeros(length(t),1); % y-covariance

for i=1:length(t)
  % Measurement update
  Mn = P*C'/(C*P*C'+R); % Also write it K
  
  x = x + Mn*(yv(i)-C*x);   % x[n|n] 
  P = (eye(3)-Mn*C)*P;      % P[n|n] %% (I-KH)P

  ye(i) = C*x; % Estimated Output %% C is H (observation matrix)
  errcov(i) = C*P*C'; %(HPH^T)

  % Time update
  x = A*x + B*u(i);        % x[n+1|n]
  P = A*P*A' + B*Q*B';     % P[n+1|n]   %% P is the covarian of state estimate
  
end

%%
figure(3)
subplot(211), plot(t,y,'--',t,ye,'-')  %% Output & Estimate
title('Time-varying Kalman filter response')
xlabel('No. of samples'), ylabel('Output')
subplot(212), plot(t,y-yv,'-.',t,y-ye,'-') %% Measurement Error & Estimation Error
xlabel('No. of samples'), ylabel('Output')

 %%
sigmaa = sqrt(errcov)';

%% Estimation Error

%Variance (sum(square of all values)/total number)

EstErr = y - ye;
EstErrCov = sum(EstErr.*EstErr)/length(EstErr);

% This is more than the other, because Q is less
% also, it won't follow any trends in the system like as in time etc
