function [dy_dt] = ODE_t(t,y,help)
% Function calculating the derivative of y, target state vector
% Input: 
% t - the current time (sec)
% y - vector of the variables (xt,yt,gamma_t,at) solved
% help - a vector containing constat variables which aids to calculate the solution
% Output: 
% dy_dt - the derivative vecotr of the variables in y 


xt=y(1);
yt=y(2);
gamma_t=y(3);
at=y(4);

vt=help(1);
nt=help(2);
theta=help(3);

dy_dt(1,1)=-vt*cos(gamma_t);
dy_dt(2,1)=vt*sin(gamma_t);
dy_dt(3,1)=at/vt;
dy_dt(4,1)=(nt-at)/theta;

end

