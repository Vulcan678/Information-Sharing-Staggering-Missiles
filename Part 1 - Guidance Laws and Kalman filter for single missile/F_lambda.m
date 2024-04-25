function [f] = F_lambda(lambda,params1,params2,h)
% function for evaulating the derivative of lambda
% Inputs: 
% lambda - current lanbda value
% params1 [rad,rad,m,rad/sec,rad/sec] - (gamma_m/t) angle of velocity respectively to x-axis of 
% interceptor and target, respectively
%                                       (rho) distance between target and interceptor
%                                       (ddel_m/t) angle rate between gamma and LOS of 
% interceptor and target, respectively
% params2 [m/sec,m/sec] - velocity of interceptor and target, respectively
% h [sec] - time step (dt)
% Output: 
% f - evaluation of lambda derivative using taylor series

gamma_m=params1(1);
gamma_t=params1(2);
rho=params1(3);
ddel_m=params1(4);
ddel_t=params1(5);
vm=params2(1);
vt=params2(2);

del_m=gamma_m-lambda;
del_t=gamma_t+lambda;
v_lambda=-vm*sin(del_m)+vt*sin(del_t);
v_rho=-(vm*cos(del_m)+vt*cos(del_t));

v=v_lambda/rho;
a=(rho*(-vm*ddel_m*cos(del_m)+vt*ddel_t*cos(del_t))-v_lambda*v_rho)/(rho^2);

f=v+h*a;
end

