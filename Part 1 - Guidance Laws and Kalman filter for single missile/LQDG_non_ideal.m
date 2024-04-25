function [ nc ] = LQDG_non_ideal(gem_b,Z_PN,t_go,del_m,at_n,am_n,tau,theta)
% Inputs: 
% gen_b [1,1] - vector of design parameters
% Z_PN [m] - simple estimation of final lateral target displacement from initial LOS
% t_go [sec] - estimated time to interception 
% del_m [rad] - angle from LOS to interceptor velocity vector
% at_n [m/sec^2] - current normal acceleration of the target
% am_n [m/sec^2] - current normal acceleration of the interceptor
% tau [sec] - time constant of the dynamics of the interceptor
% theta [sec] - time constant of the dynamics of the target
% Output: 
% nc [m/sec^2] - acceleration command of interceptor according to LQDG
%                guidance law, assuming interceptor's and target's command delay 

gem=gem_b(1);
b=gem_b(2);

h=t_go/tau;
h_bar=t_go/theta;
eps=theta/tau;
psi=exp(-h)+h-1;
psi_bar=exp(-h_bar)+h_bar-1;

delta=3+6*h-6*h^2+2*(1-gem^(-2))*h^3-3*exp(-2*h)-12*h*exp(-h)+6/(b*tau^3);
f=-3*eps^3-6*h*eps^2+6*eps*h^2+12*h*(eps^2)*exp(-h_bar)+3*(eps^3)*exp(-2*h_bar);
N=(6*psi*h^2)/(delta+f*gem^(-2));
nc=N*(Z_PN+at_n*psi_bar*theta^2-am_n*psi*tau^2)/(cos(del_m)*t_go^2);  
      
end