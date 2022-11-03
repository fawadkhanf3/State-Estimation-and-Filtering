function [time,states,bounds] = simulate_quadrotor(fp,ts,x0,zeta0,xi0,X0)

t0 = ts(1); tf = ts(2);

time   = zeros(length(t0:fp.T:tf),1);
states = zeros(length(t0:fp.T:tf),14);
bounds = zeros(length(t0:fp.T:tf),12,2);

t = t0;
X = [x0(1:9);zeta0;xi0;x0(10:12)];
% [x y z psi theta phi u v w zeta xi p q r]

time(1)     = t;
states(1,:) = X(:);

Xhat = X0;
XBounds = interval(Xhat);
x_up  = XBounds.sup;
x_low = XBounds.inf;
bounds(1,:,1) = x_up(:);
bounds(1,:,2) = x_low(:);

k = 1;
while t<=tf
    
    t = t+fp.T;
    
    [xref,yref,zref,psiref] = reference_trajectory(t,fp);
    ref_traj = {xref,yref,zref,psiref};
    
    [u1,u2,u3,u4] = controller(t,X(:),ref_traj,fp);
    control_input = {u1,u2,u3,u4};
    
    disturbances = external_disturbances();
    
    % Simulate with Forward Euler Integration
    [X,U] = state_propagation(t,X(:),control_input,disturbances,fp);
    
    Y = measurements(X(:),fp);
    
    k = k+1;
    time(k)     = t;
    states(k,:) = X(:);
    
    %% Nonlinear State Estimation
    YK = Y(:) + (-fp.Dv*fp.V);
    Xbar = nse_prediction(t,Xhat,U,fp)+fp.W;
    Xhat = nse_update(Xbar,YK,fp);
    XBounds = interval(Xhat);
    x_up  = XBounds.sup;
    x_low = XBounds.inf;
    
    bounds(k,:,1) = x_up(:);
    bounds(k,:,2) = x_low(:);
        
end