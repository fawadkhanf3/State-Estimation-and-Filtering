%% Total Variation Signal Denoising

clear;close all;clc

%%

f = [-1*ones(1000,1);
     +3*ones(100,1);
     +1*ones(500,1);
     -2*ones(800,1);
     0*ones(900,1)]; 

plot(f);
axis([1 length(f) -4 4]);
title('Original');

g = f + .25*randn(length(f),1);

figure;
plot(g,'r');
title('Noisy');
axis([1 length(f) -4 4]);

fc = denoisetv(g,.5);

figure;
plot(fc,'g');
title('De-noised');
axis([1 length(f) -4 4]);

function f = denoisetv(g,mu)

I = length(g);
u = zeros(I,1);
y = zeros(I,1);
rho = 10;

eigD = abs(fftn([-1;1],[I,1])).^2;

for k = 1:100
    f = real(ifft(fft(mu*g+rho*Dt(u)-Dt(y))./(mu+rho*eigD)));
    v = D(f)+(1/rho)*y;
    u = max(abs(v)-1/rho,0).*sign(v);
    y = y-rho*(u-D(f));
end

end

function y = D(x)
y = [diff(x);x(1)-x(end)];
end

function y = Dt(x)
y = [x(end)-x(1);-diff(x)];
end
