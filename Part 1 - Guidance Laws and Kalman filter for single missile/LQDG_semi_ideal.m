function [ nc ] = LQDG_semi_ideal(gem_b,Z_PN,t_go,del_m,am_n,tau)
% Inputs: 
% gen_b [1,1] - vector of design parameters
% Z_PN [m] - simple estimation of final lateral target displacement from initial LOS
% t_go [sec] - estimated time to interception
% del_m [rad] - angle from LOS to interceptor velocity vector
% am_n [m/sec^2] - current normal acceleration of the interceptor
% tau [sec] - ime constant of the dynamics of the interceptor
% Output: 
% nc [m/sec^2] - acceleration command of interceptor according to LQDG
%                guidance law, assuming only interceptor's command delay 

gem=gem_b(1);
b=gem_b(2);

h=t_go/tau;
psi=exp(-h)+h-1;

delta=3+6*h-6*h^2+2*(1-gem^(-2))*h^3-3*exp(-2*h)-12*h*exp(-h)+6/(b*tau^3);
N=(6*psi*h^2)/delta;
nc=N*(Z_PN-am_n*psi*tau^2)/(cos(del_m)*t_go^2);  

      
end