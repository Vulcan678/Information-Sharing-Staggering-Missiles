function [ t,xm,ym,xt,yt,rho,vc,nc,t_go ] = Ideal (param,method,manuver,dt,xt0,yt0,xm0,ym0,vm,vt,HE,gamma_t0,nt_amp,w,phase,flag,maxnc )
% Ideal: method can't be {3,5,6,8,9}
% Inputs: 
% param - relevant design constants for the chosen guidance law
% method - number representing the chosen interceptor's guidance law
% manuver - number representing the chosen target's manuver strategy
% dt [sec] - the time step of the simulation (acts as indecator if we at 
%            the end game or not)
% xt0 [m] - initial x position of the target
% yt0 [m] - initial x position of the target
% xm0 [m,m] - vector of initial x position of the interceptor 
% ym0 [m,m] - vector of initial x position of the interceptor 
% vm [m/sec] - velocity of the interceptor
% vt [m/sec] - velocity of the target
% HE [rad] - initial heading error of the interceptor
% gamma_t0 [rad] - the initial value of gamma of the target
% nt_amp [m/sec] - the maximal acceleration command of the target
% w [rad/sec] - phase change rate for bang-bang or sin\cos strategies
% phase [rad] - initial phase for sin\cos strategies
% flag - limit interceptor acceleration to maxnc (1) don't limit (0)
% maxnc [m/sec] - the maximal acceleration command of the interceptor
% Outputs: 
% t [sec] - vector of the time at each dicreate point
% xm [m] - the x position of the interceptor as function of time
% ym [m] - the y position of the interceptor as function of time
% xt [m] - the x position of the target as function of time
% yt [m] - the y position of the target as function of time
% rho [m] - distance between the interceptor and the target as function of time
% vc [m/sec] - closing velocity between the interceptor and the target as function of time
% nc [m/sec^2] - the acceleration commands of the target as function of time
% t_go [sec] - estimated time of interception as function of time

% initialize
i=1;
t(i)=0;

%%% target
xt(i)=xt0;
yt(i)=yt0;

gamma_t(i)=gamma_t0;

nt(i)=manuver_nt(manuver,nt_amp,w,phase,0);
at(i)=nt(i);

%%% interceptor
xm(i)=xm0;
ym(i)=ym0;

am(i)=0;

%%% relative relations between interceptor and target
xtm(i)=xt(i)-xm(i);
ytm(i)=yt(i)-ym(i);
rho(i)=sqrt(xtm(i)^2+ytm(i)^2);

lambda(i)=atan2(ytm(i),xtm(i));
l(i)=asin(vt*sin(gamma_t(i)+lambda(i))/vm);%
gamma_m(i)=lambda(i);%+l(i)+HE;
del_m(i)=gamma_m(i)-lambda(i);
del_t(i)=gamma_t(i)+lambda(i);

v_rho(i)=-(vm*cos(del_m(i))+vt*cos(del_t(i)));
v_lambda(i)=-vm*sin(del_m(i))+vt*sin(del_t(i));
dlambda(i)=v_lambda(i)/rho(i);
dgamma_m(i)=am(i)/vm;
dgamma_t(i)=at(i)/vt;
ddel_m(i)=dgamma_m(i)-dlambda(i);
ddel_t(i)=dgamma_t(i)+dlambda(i);

t_go(i)=-rho(i)/v_rho(i);

at_n(i)=at(i)*cos(del_t(i));
am_n(i)=am(i)*cos(del_m(i));

% acceleration command calculation
nc(i)=nc_calc(t_go(i), t_go(i)+t(i), 0, 0, am_n(i), at_n(i), v_rho(i), dlambda(i), del_m(i), param, flag, maxnc, method);

Fxva=@(x,v,a,h) v+h*a;
F_lamb=@(x,v,a,h) F_lambda(x,v,a,h);

% performing the simulation until the interceptor hit or fly by the target
while (i==1||abs(rho(i))<=abs(rho(i-1)))&&t(i)<=15
    %%% if the interceptor is in close distance to the target, reduce the time step
    if rho(i)<4*dt*(vt+vm)
        dt=1e-4;
    end    
    i=i+1;
    t(i)=t(i-1)+dt;    
    
    % interceptor and target advancment
    xm(i)=RK4(Fxva, xm(i-1), vm*cos(gamma_m(i-1)),-dgamma_m(i-1)*vm*sin(gamma_m(i-1)), dt);
    ym(i)=RK4(Fxva, ym(i-1), vm*sin(gamma_m(i-1)), dgamma_m(i-1)*vm*cos(gamma_m(i-1)), dt);
    rho(i)=RK4(Fxva, rho(i-1), v_rho(i-1), vm*ddel_m(i-1)*sin(del_m(i-1))+vt*ddel_t(i-1)*sin(del_t(i-1)), dt);
    lambda(i)=RK4(F_lamb, lambda(i-1), [gamma_m(i-1),gamma_t(i-1),rho(i-1),ddel_m(i-1),ddel_t(i-1)], [vm,vt], dt);
    gamma_m(i)=RK4(Fxva, gamma_m(i-1), am(i-1)/vm, 0, dt);
    gamma_t(i)=RK4(Fxva, gamma_t(i-1), at(i-1)/vt, 0, dt);
        
    del_m(i)=gamma_m(i)-lambda(i);
    del_t(i)=gamma_t(i)+lambda(i);
    
    xt(i)=xm(i)+rho(i)*cos(lambda(i));
    yt(i)=ym(i)+rho(i)*sin(lambda(i));
    
    v_rho(i)=-(vm*cos(del_m(i))+vt*cos(del_t(i)));
    v_lambda(i)=-vm*sin(del_m(i))+vt*sin(del_t(i));
    dlambda(i)=v_lambda(i)/rho(i);
    
    t_go(i)=-rho(i)/v_rho(i);
    
    nc(i)=nc_calc(t_go(i), t_go(i)+t(i), 0, 0, am_n(i-1), at_n(i), v_rho(i), dlambda(i), del_m(i), param, flag, maxnc, method);
    nt(i)=manuver_nt(manuver,nt_amp,w,phase,t(i));
    am(i)=nc(i);
    at(i)=nt(i);       
    at_n(i)=at(i)*cos(del_t(i));
    am_n(i)=am(i)*cos(del_m(i));
    
    dgamma_m(i)=am(i)/vm;
    dgamma_t(i)=at(i)/vt;
    ddel_m(i)=dgamma_m(i)-dlambda(i);
    ddel_t(i)=dgamma_t(i)+dlambda(i);
end

vc=-v_rho;

end