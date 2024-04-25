function [ nc ] = PN(N,Z_PN,t_go,del_m)
% Inputs: 
% N [1] - design parameter
% Z_PN [m] - simple estimation of final lateral target displacement from initial LOS
% t_go [sec] - estimated time to interception 
% del_m [rad] - angle from LOS to interceptor velocity vector
% Output: 
% nc [m/sec^2] - acceleration command of interceptor according to PN guidance law  

nc=N*Z_PN/(cos(del_m)*t_go^2);   

end

