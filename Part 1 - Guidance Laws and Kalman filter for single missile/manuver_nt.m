function [ nt ] = manuver_nt( manuver,nt_amp,w,phase,t )
% function calculating target acceleration command for different strategies
% Inputs: 
% manuver - number representing the chosen manuver strategy
% nt_amp [m/sec^2] - maximal acceleration of the target
% w [rad/sec] - phase change rate for bang-bang or sin\cos strategies
% phase [rad] - initial phase for sin\cos strategies
% t [sec] - current time
% Output: 
% nt [m/sec^2] - acceleration command of target according to the chosen strategy

switch manuver
    case 1
        nt=nt_amp*sin(w*t+phase);
    case 2
        nt=nt_amp*cos(w*t+phase);
    case 3
        nt=nt_amp;
    case 4
        if t<((2*pi/w)-1e-5)
            nt=-nt_amp;
        else
            nt=nt_amp;
        end
end

end

