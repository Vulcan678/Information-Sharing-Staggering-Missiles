function [ nc ] = DGL0_ideal(am_at,Z_PN,t_go,del_m)
% Inputs: 
% am_at [m/sec^2,m/sec^2] - vector of maximal acceleration of interceptor and target, respectively
% Z_PN [m] - simple estimation of final lateral target displacement from initial LOS
% t_go [sec] - estimated time to interception 
% del_m [rad] - angle from LOS to interceptor velocity vector
% Output: 
% nc [m/sec^2] - acceleration command of interceptor according to DGL0
%                guidance law, assuming no command delay 

max_am=am_at(1);
max_at=am_at(2);

N=2/(1-(max_at/max_am));
nc=N*Z_PN/(cos(del_m)*t_go^2); 
      
end