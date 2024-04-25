function [ nc ] = APN(Z_PN,t_go,del_m,at_n)
% Inputs: 
% Z_PN [m] - simple estimation of final lateral target displacement from initial LOS
% t_go [sec] - estimated time to interception 
% del_m [rad] - angle from LOS to interceptor velocity vector
% at_n [m/sec^2] - maximal normal acceleration of the target
% Output: 
% nc [m/sec^2] - acceleration command of interceptor according to APN guidance law   

nc=3*(Z_PN+0.5*at_n*t_go^2)/(cos(del_m)*t_go^2);   

end

