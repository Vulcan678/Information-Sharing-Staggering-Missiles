function [ f ] = func_result_DGL0( t,tau,rv_ru,tf )
% a function that equals to 0 when t=ts
% Inputs: 
% t [sec] - current time
% tau [sec] - time constant of the dynamics of the interceptor
% rv_ru [1] - maximal acceleration ratio between target and interceptor
% tf [sec] - estimated time of interception relative to start time
% Output: 
% f [1] - function value 

h=(tf-t)/tau;
psi=exp(-h)+h-1;

f=abs((psi/(h^2))-0.5*(1-rv_ru));

end

