function [ nc ] = DGL1(am_at_k,Z_PN,t_go,tf,tau,theta,am_n,at_n)
% Inputs: 
% am_at_k [m/sec^2,m/sec^2,1] - vector of maximal acceleration of interceptor
% and target, respectively and k the design parameter
% Z_PN [m] - simple estimation of final lateral target displacement from initial LOS
% t_go [sec] - estimated time to interception 
% tf [sec] - estimated time of interception relative to start time
% tau [sec] - time constant of the dynamics of the interceptor
% theta [sec] - time constant of the dynamics of the target
% am_n [m/sec^2] - current normal acceleration of the interceptor
% at_n [m/sec^2] - current normal acceleration of the target
% Output: 
% nc [m/sec^2] - acceleration command of interceptor according to DGL0
%                guidance law, assuming interceptor's and target's command delay 

sat = @(x, delta) min(max(x/delta, -1), 1);

max_am=am_at_k(1);
max_at=am_at_k(2);
k=am_at_k(3);

h=t_go/tau;
h_bar=t_go/theta;
psi=exp(-h)+h-1;
psi_theta=exp(-h_bar)+h_bar-1;

z_bound=max_am*(tau^2)*(0.5*(h^2)-psi)-max_at*(theta^2)*(0.5*(h_bar^2)-psi_theta);
z=Z_PN-psi*am_n*(tau^2)+at_n*(theta^2)*psi_theta;


if abs(z)<abs(z_bound)
    p_s=max_am*sat(z,k*z_bound);
else
    p_s=max_am*sign(z);
end

nc=p_s;

end