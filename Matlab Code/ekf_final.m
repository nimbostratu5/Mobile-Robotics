function [ X, S ] = ekf_final( x0,s0,u,z,m )
global dt

vt = u(1,1) ;
wt = u(2,1)*vt;
theta = x0(3,1);

x_comp = -(vt/wt)*cos(theta) + (vt/wt)*cos(theta + (wt*dt));
y_comp = -(vt/wt)*sin(theta) + (vt/wt)*sin(theta + (wt*dt));
G_t = [1, 0, x_comp; 0, 1, y_comp; 0, 0, 1];

ij00 = (-sin(theta) + sin(theta + (wt*dt)))/wt;
ij01 = ((vt/(wt^2))*(sin(theta) - sin(theta+(wt*dt)))) +  ( ((vt*dt)/wt)*(cos(theta+(wt*dt))) );
ij10 = (cos(theta) - cos(theta + (wt*dt)))/wt;
ij11 = ((-vt/(wt^2))*(cos(theta) - cos(theta+(wt*dt)))) + ( ((vt*dt)/wt)*(sin(theta+(wt*dt))) );
V_t = [ij00, ij01; ij10, ij11; 0, dt];

a1 = 1; a2 = 1; a3 = 0.1; a4 = 0.1;
M_t = [ (a1*(vt^2) + a2*(wt^2)), 0;  0, (a3*(vt^2) + a4*(wt^2)) ];

mu00 = ((-vt/wt)*sin(theta)) + ((vt/wt)*sin(theta + (wt*dt)));
mu10 = ((vt/wt)*cos(theta)) - ((vt/wt)*cos(theta + (wt*dt)));

Q = [0.0025, 0, 0; 0, 0.0025, 0; 0, 0, 0.025];
H = [1,0,0; 0,1,0; 0,0,1];
I = eye(3);

if m == 1
    x_bar = x0 + [ mu00; mu10; (wt*dt)];
    s_bar = G_t*s0*G_t' + V_t*M_t*V_t';
    %kalman gain 
    K = s_bar*H'/(H*s_bar*H' + Q);
    %update state vector
    X = x_bar + K*(z - H*x_bar);
    %update covariance
    S = (I - K*H)*s_bar; %sigma
elseif m == 0
    X = x0 + [ mu00; mu10; (wt*dt)];
    S = G_t*s0*G_t' + V_t*M_t*V_t';
end
end

