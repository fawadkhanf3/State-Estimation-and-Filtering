clear;close all;clc
format long g
set(0,'DefaultLineLineWidth',2);
rng(8);

N  = 1000;
dt = 0.001;
t  = dt*(1:N);

F = [1,dt;0,1];
G = [-1/2*dt^2;-dt];

H = [1,0];
u = 9.80665;

I = eye(2);

y0 = 100;
v0 = 0;

xt = zeros(2,N);

xt(:,1) = [y0;v0];

for k = 2:N
    xt(:,k) = F*xt(:,k-1) + G*u;
end

Q = [0,0;0,0];

R = 4;

P = [10,0;0,0.01];

v = sqrt(R)*(rand(size(H,1),N)*2-1);

Dw = eye(2);

z = H*xt + v;

V = conZonotope(interval(-sqrt(R),sqrt(R)));
W = conZonotope([0;0]);

x = zeros(2,N);

x(:,1) = [105;0];

x01 = [0.95*x(1,1),1.05*x(1,1)];
x02 = [x(2,1)-2,x(2,1)+2];

X0 = conZonotope(interval([x01(1);x02(1)],[x01(2);x02(2)]));

Xhat = X0;

CKF_xhat_plus = Xhat.Z(:,1);
CKF_P_plus    = P;
CKF_xhat_prev = CKF_xhat_plus;
CKF_P_prev    = CKF_P_plus;

for k = 2:N
    
    Xhat = cz_linear_estimator(Xhat,u,W,z(k),V);
    
    [CKF_xhat_plus, CKF_P_plus] = constrained_kalman_filter(...
                                CKF_xhat_prev, CKF_P_prev,u,...
                                z(k),Q,R,Xhat);
                            
    CKF_xhat_prev = CKF_xhat_plus;
    CKF_P_prev    = CKF_P_plus;
    
    x(:,k) = CKF_xhat_plus;
    
end

%
figure(1);
subplot(211);hold on;grid on;box on;
plot(t,z,'g-',t,x(1,:),'b--');
plot(t,xt(1,:),'r:');
xlabel('t (s)');ylabel('x_{1} = h (m)');
legend('Measured','Estimated','True');

subplot(212);hold on;grid on;box on;
plot(t,x(2,:),'b--');
plot(t,xt(2,:),'r:');
xlabel('t (s)');ylabel('x_{2} = v (m/s)');
legend('Estimated','True');

figure(2)
subplot(211)
plot(t,x(1,:)-xt(1,:),'m');
xlabel('t (s)');ylabel('\Delta x_{1} = h (m)');
subplot(212)
plot(t,x(2,:)-xt(2,:),'m');
xlabel('t (s)');ylabel('\Delta x_{2} = v (m/s)');