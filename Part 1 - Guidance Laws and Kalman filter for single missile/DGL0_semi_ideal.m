function [ nc ] = DGL0_semi_ideal(am_at_k,Z_PN,t_go,tf,tau,am_n)
% Inputs: 
% am_at_k [m/sec^2,m/sec^2,1] - vector of maximal acceleration of interceptor
% and target, respectively and k the design parameter
% Z_PN [m] - simple estimation of final lateral target displacement from initial LOS
% t_go [sec] - estimated time to interception 
% tf [sec] - estimated time of interception relative to start time
% tau [sec] - ime constant of the dynamics of the interceptor
% am_n [m/sec^2] - current normal acceleration of the interceptor
% Output: 
% nc [m/sec^2] - acceleration command of interceptor according to DGL0
%                guidance law, assuming only interceptor's command delay 

sat = @(x, delta) min(max(x/delta, -1), 1);

max_am=am_at_k(1);
max_at=am_at_k(2);
k=am_at_k(3);

h=t_go/tau;
psi=exp(-h)+h-1;

z_bound=max_am*(t_go^2)*(0.5*(1-max_at/max_am)-(psi/(h^2)));

ts=find_ts_DGL0(tau,max_at/max_am,tf);

z=Z_PN-psi*am_n*(tau^2);

if (abs(z)<z_bound)&&((tf-t_go)<ts)
    p_s=max_am*sat(z,k*z_bound); 
else
    p_s=max_am*sign(z);
end

nc=p_s;
      
end