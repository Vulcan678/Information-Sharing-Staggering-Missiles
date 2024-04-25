function [ nc ] = OGL(Z_PN,t_go,del_m,at_n,am_n,tau)
% Inputs: 
% Z_PN [m] - simple estimation of final lateral target displacement from initial LOS
% t_go [sec] - estimated time to interception 
% del_m [rad] - angle from LOS to interceptor velocity vector
% at_n [m/sec^2] - current normal acceleration of the target
% am_n [m/sec^2] - current normal acceleration of the interceptor
% tau [sec] - time constant of the dynamics of the interceptor
% Output: 
% nc [m/sec^2] - acceleration command of interceptor according to LQDG
%                guidance law, assuming only interceptor's command delay 

h=t_go/tau;
psi=exp(-h)+h-1;
delta=3+6*h-6*h^2+2*h^3-3*exp(-2*h)-12*h*exp(-h);

N=(6*psi*h^2)/delta;
nc=N*(Z_PN+0.5*at_n*(t_go^2)-am_n*psi*tau^2)/(cos(del_m)*t_go^2);  
      
end