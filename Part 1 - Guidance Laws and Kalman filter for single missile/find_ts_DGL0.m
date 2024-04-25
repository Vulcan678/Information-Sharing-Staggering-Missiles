function [ ts ] = find_ts_DGL0( tau,rv_ru,tf )
% a function for estimating the switching time of the targe, ts
% Inputs: 
% tau [sec] - time constant of the dynamics of the interceptor
% rv_ru [1] - maximal acceleration ratio between target and interceptor
% tf [sec] - estimated time of interception relative to start time
% Output: 
% ts [sec] - estimation of the acceleration switching time of the target,
%            according to DGL0 guidance law derivations

fun=@(t) func_result_DGL0(t,tau,rv_ru,tf);
if tf<=0
    tf=15;
end
ts=fminbnd(fun,0,tf*2);

end

