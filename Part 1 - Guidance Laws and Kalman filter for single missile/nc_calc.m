function [ nc ] = nc_calc( t_go, tf, tau, theta, am_n, at_n, v_rho, dlambda, del_m, param, flag, maxnc, method )
% Inputs: 
% t_go [sec] - estimated time to interception 
% tf [sec] - estimated time of interception relative to start time
% tau [sec] - time constant of the dynamics of the interceptor
% theta [sec] - time constant of the dynamics of the target
% am_n [m/sec^2] - current normal acceleration of the interceptor
% at_n [m/sec^2] - current normal acceleration of the target
% v_rho [m/sec] - minus the closing speed between the interceptor and the target
% dlambda [rad/sec] - angular rate of LOS between the interceptor and the target
% del_m [rad] - angle from LOS to interceptor velocity vector
% param - relevant design constants for the chosen guidance law
% flag - limit interceptor acceleration to maxnc (1) don't limit (0)
% maxnc [m/sec^2] - maximal acceleration of the interceptor
% method - number representing the chosen guidance law
% Output: 
% nc [m/sec^2] - acceleration command of interceptor according to the
%                chosen guidance law

    Z_PN=-v_rho*dlambda*t_go^2;

    switch method
        case 1 
            k=PN(param,Z_PN,t_go,del_m); %N
        case 2 
            k=APN(Z_PN,t_go,del_m,at_n); 
        case 3 
            k=OGL(Z_PN,t_go,del_m,at_n,am_n,tau);
        case 4 
            k=LQDG_ideal(param,Z_PN,t_go,del_m); %gem1
        case 5 
            k=LQDG_semi_ideal(param,Z_PN,t_go,del_m,am_n,tau); %[gem2,b2]
        case 6 
            k=LQDG_non_ideal(param,Z_PN,t_go,del_m,at_n,am_n,tau,theta); %[gem3,b3]
        case 7 
            k=DGL0_ideal(param,Z_PN,t_go,del_m); %[maxnc,maxnt]
        case 8 
            k=DGL0_semi_ideal(param,Z_PN,t_go,tf,tau,am_n); %[maxnc,maxnt,k]
        case 9 
            k=DGL1(param,Z_PN,t_go,tf,tau,theta,am_n,at_n); %[maxnc,maxnt,k]
    end
    if flag      
        if k<0
            nc=max(k,-maxnc);
        else
            nc=min(k,maxnc);
        end
    else
        nc=k;
    end
end

