function [dy_dt] = ODE_m(t,y,help)
% Function calculating the derivative of y, interception missile state vector
% Input: 
% t - the current time (sec)
% y - vector of the variables (xm,ym,gamma_m,am,rho,lambda,gamma_t,at) solved
% help - a vector containing constat variables which aids to calculate the solution
% Output: 
% dy_dt - the derivative vecotr of the variables in y 

xm=y(1);
ym=y(2);
gamma_m=y(3);
am=y(4);
rho=y(5);
lambda=y(6);
gamma_t=y(7);
at=y(8);

vm=help(1);
vt=help(2);
nc=help(3);
nt=help(4);
tau=help(5);
theta=help(6);

delta_m=gamma_m-lambda;
delta_t=gamma_t+lambda;

v_rho=-(vm*cos(delta_m)+vt*cos(delta_t));
v_lambda=-vm*sin(delta_m)+vt*sin(delta_t);

dy_dt(1,1)=vm*cos(gamma_m);
dy_dt(2,1)=vm*sin(gamma_m);
dy_dt(3,1)=am/vm;
dy_dt(4,1)=(nc-am)/tau;
dy_dt(5,1)=v_rho;
dy_dt(6,1)=v_lambda/rho;
dy_dt(7,1)=at/vt;
dy_dt(8,1)=(nt-at)/theta;

end

