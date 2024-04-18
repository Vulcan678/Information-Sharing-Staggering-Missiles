function [nt] = nt_manuver(t,at_max,T_sw)
% Function determaining the acceleration command of the target, assuming 
% bang-bang manuver
% Input: 
% t - the current time (sec)
% at_max (m/sec^2) - the maximal acceleration command of the target
% T_sw (sec) - the time when the switch must accure
% Output: 
% nt (m/sec^2) - the acceleartion command of the target

if t<T_sw
    nt=-at_max;
else
    nt=at_max;
end

end

