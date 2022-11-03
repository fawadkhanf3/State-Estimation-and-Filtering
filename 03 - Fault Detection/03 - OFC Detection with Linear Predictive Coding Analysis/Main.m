clear;close all;clc
load for_LPC7

x = x.';
y = x(:,3);

p     = 5;
Fs    = 40;
Tsamp = 1/Fs;

window_length = 3*Fs;
segment = nan(window_length,1);

L = length(segment);
f = Fs*(0:floor(L/2))/L;

Threshold1 = 0.2;
Threshold2 = 0.2;
counter   = 0;

r1  = 1;
r2  = 1-Threshold1;
xc1 = -r1:0.01:r1;
xc2 = -r2:0.01:r2;
y1  = +sqrt(r1^2-xc1.^2); y2 = -sqrt(r1^2-xc1.^2);
y3  = +sqrt(r2^2-xc2.^2); y4 = -sqrt(r2^2-xc2.^2);

timer = 0;
timer_prev = 0;

for i = 1:length(y)
    
    segment = [segment;y(i)];
    segment(1) = [];
    
    if any(isnan(segment))
        continue;
    else
        if (abs(y(i))>0.1)
        
        a = lpc(segment,p);
%         s = roots(a);
        a = my_lpc(segment,p);
        s = roots([1,a]);
        
        d = zeros(p,1);
        for m = 1:p
            d(m,1) = abs(1-norm([real(s(m));imag(s(m))]));
        end
        
        if any(d<Threshold1)
            
            Y  = fftshift(fft(segment,L))/L;
            
            P  = abs(Y);
            P  = P(1:floor(L/2)+1);
            [a,b] = max(P);        
            if any(P(f>=1)>Threshold2)
                timer   = i/Fs;
                counter = counter+1;
                if (abs(timer-timer_prev)>0.5), counter = 0; end
                timer_prev = timer;
                
            end
            
        end
               
        if counter > 3
            fprintf('\n Fault detected at %g\n',x(i,1));
            break;
        end
        
%         figure(1);clf;hold on;grid on;box on
%         plot(xc,y1,'r.-',xc,y2,'r.-');
%         plot(xc2,y3,'b.-',xc2,y4,'b.-');
%         plot(real(s),imag(s),'*','MarkerSize',10);
%         title(sprintf('time = %2.4f',x(i,1)));
%         drawnow;
        end
    end
end