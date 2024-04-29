function [nc] = DGL1(t_go,am_n,at_n,am_max,at_max,tau,theta,k,v_rho,dlambda)
% Function determaining the acceleration command of the pursuer using DGL1
% Input: 
% t_go - the current time left to interception (sec)
% am_n (m/sec^2) - the acceleration normal to the LOS of the pursuer
% am_n (m/sec^2) - the acceleration normal to the LOS of the target
% am_max (m/sec^2) - the maximal acceleration command of the pursuer
% at_max (m/sec^2) - the maximal acceleration command of the target
% tau (sec) - the dynamic delay of the pursuer
% theta (sec) - the dynamic delay of the target
% k - the  portion of the singular region in which the acceleration is linear
% v_rho - the closing speed between the interceptor and the target (m/sec)
% dlambda - the derivative (in time) of the LOS angle bwtween the
%           interceptor and the target (rad/sec)
% Output: 
% nc (m/sec^2) - the acceleartion command of the pursuer

sat = @(x, delta) min(max(x/delta, -1), 1);

h=t_go/tau;
h_bar=t_go/theta;
psi=exp(-h)+h-1;
psi_theta=exp(-h_bar)+h_bar-1;
z_bound=abs(am_max*(tau^2)*(0.5*(h^2)-psi)-at_max*(theta^2)*(0.5*(h_bar^2)-psi_theta)); % singularity boundary
z=-v_rho*(t_go^2)*dlambda-psi*am_n*(tau^2)+at_n*(theta^2)*psi_theta; % ZEM_DGL1

if abs(z)<z_bound
    nc=am_max*sat(z,k*z_bound);
else
    nc=am_max*sign(z);
end

end

