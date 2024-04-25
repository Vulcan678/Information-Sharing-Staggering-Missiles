function [ t,xm,ym,xt,yt,rho,vc,nc,t_go ] = Non_Ideal_true_theta( param,method,manuver,tau,theta_true,theta,dt,xt0,yt0,xm0,ym0,vm,vt,HE,gamma_t0,nt_amp,w,phase,flag,maxnc,sigmas,delta_sigma,at_sigma )
% Non Ideal: method can't be {4,5,7}
% Using noised measurments for decision making and assuming wrong target's
% time constant of the dynamic
% Inputs: 
% param - relevant design constants for the chosen guidance law
% method - number representing the chosen interceptor's guidance law
% manuver - number representing the chosen target's manuver strategy
% tau [sec] - time constant of the dynamics of the interceptor
% theta_true [sec] - true time constant of the dynamics of the target
% theta [sec] - assumed time constant of the dynamics of the target
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
% sigmas [m,rad,rad,m/sec^2] - the values of the sqrt of the diagonal values 
%                              of the initial covariance matrix of the
%                              state vector
% delta_sigma [rad] - the SD of the interceptor's angle measurments
% at_sigma [m/sec^2] - the SD of the targets's acceleration measurments
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

% true
xt(i)=xt0;
yt(i)=yt0;
xm(i)=xm0;
ym(i)=ym0;

xtm(i)=xt(i)-xm(i);
ytm(i)=yt(i)-ym(i);
rho(i)=sqrt(xtm(i)^2+ytm(i)^2);

lambda(i)=atan2(ytm(i),xtm(i));
gamma_t(i)=gamma_t0;
l(i)=asin(vt*sin(gamma_t(i)+lambda(i))/vm);%
gamma_m(i)=lambda(i);%+l(i)+HE;
del_m(i)=gamma_m(i)-lambda(i);
del_t(i)=gamma_t(i)+lambda(i);

if method==9
    param(4)=l(1);
end

am(i)=0;
nt(i)=manuver_nt(manuver,nt_amp,w,phase,0);
at(i)=nt(i);

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

rho_sig=sigmas(1);
lam_sig=sigmas(2);
gam_sig=sigmas(3);
acc_sig=sigmas(4);

% missile mes
rho_mes(i)=normrnd(rho(i),rho_sig);
lambda_mes(i)=normrnd(lambda(i),lam_sig);
gamma_t_mes(i)=normrnd(gamma_t(i),gam_sig);
gamma_m_mes(i)=lambda(i);
del_m_mes(i)=del_m(i);
del_t_mes(i)=gamma_t_mes(i)+lambda_mes(i);
at_mes(i)=normrnd(at(i),acc_sig);
v_lambda_mes(i)=-vm*sin(del_m_mes(i))+vt*sin(del_t_mes(i));
v_rho_mes(i)=-(vm*cos(del_m_mes(i))+vt*cos(del_t_mes(i)));
dlambda_mes(i)=v_lambda_mes(i)/rho_mes(i);
at_n_mes(i)=at_mes(i)*cos(del_t_mes(i));
t_go_mes(i)=-rho_mes(i)/v_rho_mes(i);

xm_mes(i)=xm(i);
ym_mes(i)=ym(i);
xt_mes(i)=xm_mes(i)+rho_mes(i)*cos(lambda_mes(i));
yt_mes(i)=ym_mes(i)+rho_mes(i)*sin(lambda_mes(i));
dgamma_m_mes(i)=am(i)/vm;
dgamma_t_mes(i)=at_mes(i)/vt;
ddel_m_mes(i)=dgamma_m_mes(i)-dlambda_mes(i);
ddel_t_mes(i)=dgamma_t_mes(i)+dlambda_mes(i);

% acceleration command calculation
nc(i)=nc_calc(t_go_mes(i), t_go_mes(i)+t(i), tau, theta, am_n(i), at_n_mes(i), v_rho_mes(i), dlambda_mes(i), del_m_mes(i), param, flag, maxnc, method);

Fxva=@(x,v,a,h) v+h*a;
F_lamb=@(x,v,a,h) F_lambda(x,v,a,h);
FP_a=@(x,y,z,h) (y-x)/tau;
FT_a=@(x,y,z,h) (y-x)/theta_true;

% performing the simulation until the interceptor hit or fly by the target
while (i==1||abs(rho(i))<=abs(rho(i-1)))&&t(i)<=15
    %%% if the interceptor is in close distance to the target, reduce the time step
    if rho(i)<4*dt*(vt+vm)
        dt=1e-4;
    end    
    i=i+1;
    t(i)=t(i-1)+dt;    
    
    % true
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
    
    nt(i)=manuver_nt(manuver,nt_amp,w,phase,t(i));
    am(i)=RK4(FP_a, am(i-1), nc(i-1), 0,dt);
    at(i)=RK4(FT_a, at(i-1), nt(i-1), 0,dt); 
    at_n(i)=at(i)*cos(del_t(i));
    am_n(i)=am(i)*cos(del_m(i));
    
    % missile mes
    xm_mes(i)=xm(i);
    ym_mes(i)=ym(i);
    rho_mes(i)=RK4(Fxva, rho_mes(i-1), v_rho_mes(i-1), vm*ddel_m_mes(i-1)*sin(del_m_mes(i-1))+vt*ddel_t_mes(i-1)*sin(del_t_mes(i-1)), dt);
    del_m_mes(i)=normrnd(del_m(i),delta_sigma);
    gamma_m_mes(i)=gamma_m(i);
    lambda_mes(i)=gamma_m_mes(i)-del_m_mes(i);
    gamma_t_mes(i)=RK4(Fxva, gamma_t_mes(i-1), at_mes(i-1)/vt, 0, dt);
    
    del_t_mes(i)=gamma_t_mes(i)+lambda_mes(i);
    
    xt_mes(i)=xm_mes(i)+rho_mes(i)*cos(lambda_mes(i));
    yt_mes(i)=ym_mes(i)+rho_mes(i)*sin(lambda_mes(i));
    
    v_rho_mes(i)=-(vm*cos(del_m_mes(i))+vt*cos(del_t_mes(i)));
    v_lambda_mes(i)=-vm*sin(del_m_mes(i))+vt*sin(del_t_mes(i));
    dlambda_mes(i)=v_lambda_mes(i)/rho_mes(i);
    
    t_go_mes(i)=-rho_mes(i)/v_rho_mes(i);
    
    at_mes(i)=normrnd(at(i),at_sigma);
    at_n_mes(i)=at_mes(i)*cos(del_t_mes(i));
        
    % command 
    nc(i)=nc_calc(t_go(i), t_go(i)+t(i), tau, theta, am_n(i), at_n_mes(i), v_rho_mes(i), dlambda_mes(i), del_m_mes(i), param, flag, maxnc, method);

    % true
    dgamma_m(i)=am(i)/vm;
    dgamma_t(i)=at(i)/vt;
    ddel_m(i)=dgamma_m(i)-dlambda(i);
    ddel_t(i)=dgamma_t(i)+dlambda(i);
    
    % missile mes
    dgamma_m_mes(i)=am(i)/vm;
    dgamma_t_mes(i)=at_mes(i)/vt;
    ddel_m_mes(i)=dgamma_m_mes(i)-dlambda_mes(i);
    ddel_t_mes(i)=dgamma_t_mes(i)+dlambda_mes(i);
    
end

vc=-v_rho;

end