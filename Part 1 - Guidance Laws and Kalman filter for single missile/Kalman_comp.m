function [ t,xm,ym,xt,yt,rho,vc,nc,t_go,var,y_n_true,y_n_est,y_n_mes  ] = Kalman_comp( param,method,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm,vt,HE,gamma_t0,nt_amp,w,phase,flag,maxnc,sigmas,delta_sigma,at_sigma,w_sigma,real )
% Non Ideal: method can't be {4,5,7}
% Inputs: 
% param - relevant design constants for the chosen guidance law
% method - number representing the chosen interceptor's guidance law
% manuver - number representing the chosen target's manuver strategy
% tau [sec] - time constant of the dynamics of the interceptor
% theta [sec] - time constant of the dynamics of the target
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
% w_sigma [m/sec^2] - the assumed SD of the targets's acceleration measurments
% real - using the estimated state for decision making (0) or using the true state (1)
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
% var - estimated variance of the estimated state vector of as function of time
% y_n_true - true state vector of as function of time
% y_n_est - estimated state vector of as function of time
% y_n_mes - measured state vector of as function of time

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
l(i)=asin(vt*sin(gamma_t(i)+lambda(i))/vm)
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

y_n_true(:,i)=[del_m(i);at(i)];

rho_sig=sigmas(1);
lam_sig=sigmas(2);
gam_sig=sigmas(3);
acc_sig=sigmas(4);

% missile mes
rho_mes(i)=normrnd(rho(i),rho_sig);
lambda_mes(i)=normrnd(lambda(i),lam_sig);
gamma_t_mes(i)=normrnd(gamma_t(i),gam_sig);
del_m_mes(i)=gamma_m(i)-lambda_mes(i);
del_t_mes(i)=gamma_t_mes(i)+lambda_mes(i);
v_lambda_mes(i)=-vm*sin(del_m_mes(i))+vt*sin(del_t_mes(i));
v_rho_mes(i)=-(vm*cos(del_m_mes(i))+vt*cos(del_t_mes(i)));
dlambda_mes(i)=v_lambda_mes(i)/rho_mes(i);
at_mes(i)=normrnd(at(i),acc_sig);
at_n_mes(i)=at_mes(i)*cos(del_t_mes(i));
t_go_mes(i)=-rho_mes(i)/v_rho_mes(i);
y_n_mes(:,i)=[del_m_mes(i);at_mes(i)];

% acceleration command calculation
if(real)
    nc(i)=nc_calc(t_go(i), t_go(i)+t(i), tau, theta, am_n(i), at_n(i), v_rho(i), dlambda(i), del_m(i), param, flag, maxnc, method);
else
    nc(i)=nc_calc(t_go_mes(i), t_go_mes(i)+t(i), tau, theta, am_n(i), at_n_mes(i), v_rho_mes(i), dlambda_mes(i), del_m_mes(i), param, flag, maxnc, method);
end
    
% missile est
rho_est(i)=rho_mes(i);
lambda_est(i)=lambda_mes(i);
gamma_m_est(i)=gamma_m(i);
gamma_t_est(i)=gamma_t_mes(i);
del_m_est(i)=del_m_mes(i);
del_t_est(i)=del_t_mes(i);
v_rho_est(i)=v_rho_mes(i);
v_lambda_est(i)=v_lambda_mes(i);
at_est(i)=at_mes(i);
dlambda_est(i)=dlambda_mes(i);
t_go_est(i)=t_go_mes(i);
at_n_est(i)=at_n_mes(i);
dgamma_m_est(i)=am(i)/vm;
dgamma_t_est(i)=at_est(i)/vt;
ddel_m_est(i)=dgamma_m_est(i)-dlambda_est(i);
ddel_t_est(i)=dgamma_t_est(i)+dlambda_est(i);
y_n_est(:,i)=[del_m_est(i);at_est(i)];

p_n_n=diag(sigmas.^2);
Q_n=w_sigma;
R_n=[delta_sigma^2, 0; 0, acc_sig^2];
var(i,:)=[p_n_n(1,1), p_n_n(2,2), p_n_n(3,3), p_n_n(4,4)]; 

Fxva=@(x,v,a,h) v+h*a;
F_lamb=@(x,v,a,h) F_lambda(x,v,a,h);
FP_a=@(x,y,z,h) (y-x)/tau;
FT_a=@(x,y,z,h) (y-x)/theta;

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
    y_n_true(:,i)=[del_m(i);at(i)];
    
    % missile est n|n-1
    rho_est(i)=RK4(Fxva, rho_est(i-1), v_rho_est(i-1), vm*ddel_m_est(i-1)*sin(del_m_est(i-1))+vt*ddel_t_est(i-1)*sin(del_t_est(i-1)), dt);
    lambda_est(i)=RK4(F_lamb, lambda_est(i-1), [gamma_m_est(i-1),gamma_t_est(i-1),rho_est(i-1),ddel_m_est(i-1),ddel_t_est(i-1)], [vm,vt], dt);
    gamma_m_est(i)=gamma_m(i);
    gamma_t_est(i)=RK4(Fxva, gamma_t_est(i-1), at_est(i-1)/vt, 0, dt);
        
    del_m_est(i)=gamma_m_est(i)-lambda_est(i);
    del_t_est(i)=gamma_t_est(i)+lambda_est(i);
        
    v_rho_est(i)=-(vm*cos(del_m_est(i))+vt*cos(del_t_est(i)));
    v_lambda_est(i)=-vm*sin(del_m_est(i))+vt*sin(del_t_est(i));
    
    if(i==2)
        at_est(i)=at_est(i-1);
    else
        at_est(i)=RK4(Fxva, at_est(i-1), (at_est(i-1)-at_est(i-2))/(t(i-1)-t(i-2)), 0, dt);%at_est(i-1); 
    end
    
    % missile mes
    del_m_mes(i)=normrnd(del_m(i),delta_sigma);
    at_mes(i)=normrnd(at(i),at_sigma);    
    y_n_mes(:,i)=[del_m_mes(i);at_mes(i)];
        
    % kalman filter
    p_n1_n1=p_n_n;
    F=[0, v_lambda_est(i-1), vt*sin(del_t_est(i-1)), 0;
        -v_lambda_est(i-1)/(rho_est(i-1)^2), -v_rho_est(i-1)/rho_est(i-1), vt*cos(del_t_est(i-1))/rho_est(i-1), 0;
        0, 0, 0, 1/vt;
        0, 0, 0, -1/theta];
    phi_n=expm(F*dt);    
    Gamma_n=[0; 0; 0; 1/theta];
    H_n=[0, -1, 0, 0;
        0, 0, 0, 1];
    
    x_n_n1=[rho_est(i);lambda_est(i);gamma_t_est(i);at_est(i)];
    p_n_n1=phi_n*p_n1_n1*transpose(phi_n)+Gamma_n*Q_n*transpose(Gamma_n);
    temp=H_n*p_n_n1*transpose(H_n)+R_n;
    K_n=p_n_n1*transpose(H_n)*inv(temp);
    x_n_n=x_n_n1+K_n*(y_n_mes(:,i)-[del_m_est(i);at_est(i)]);
    p_n_n=(eye(4)-K_n*H_n)*p_n_n1; 
    var(i,:)=[p_n_n(1,1), p_n_n(2,2), p_n_n(3,3), p_n_n(4,4)];
    
    % missile est n|n
    rho_est(i)=x_n_n(1);
    lambda_est(i)=x_n_n(2);
    gamma_t_est(i)=x_n_n(3);    
    
    del_m_est(i)=gamma_m_est(i)-lambda_est(i);
    del_t_est(i)=gamma_t_est(i)+lambda_est(i);
        
    v_rho_est(i)=-(vm*cos(del_m_est(i))+vt*cos(del_t_est(i)));
    v_lambda_est(i)=-vm*sin(del_m_est(i))+vt*sin(del_t_est(i));
    dlambda_est(i)=v_lambda_est(i)/rho_est(i);
    
    t_go_est(i)=-rho_est(i)/v_rho_est(i);
    
    at_est(i)=x_n_n(4);    
    at_n_est(i)=at_est(i)*cos(del_t_est(i));
    y_n_est(:,i)=[del_m_est(i);at_est(i)];
    
    % command 
    if(real)
        nc(i)=nc_calc(t_go(i), t_go(i)+t(i), tau, theta, am_n(i), at_n(i), v_rho(i), dlambda(i), del_m(i), param, flag, maxnc, method);
    else
        nc(i)=nc_calc(t_go_est(i), t_go_est(i)+t(i), tau, theta, am_n(i), at_n_est(i), v_rho_est(i), dlambda_est(i), del_m_est(i), param, flag, maxnc, method);
    end
    
    % true
    dgamma_m(i)=am(i)/vm;
    dgamma_t(i)=at(i)/vt;
    ddel_m(i)=dgamma_m(i)-dlambda(i);
    ddel_t(i)=dgamma_t(i)+dlambda(i);
    
    % missile est n|n
    dgamma_m_est(i)=am(i)/vm;
    dgamma_t_est(i)=at_est(i)/vt;
    ddel_m_est(i)=dgamma_m_est(i)-dlambda_est(i);
    ddel_t_est(i)=dgamma_t_est(i)+dlambda_est(i);
end
vc=-v_rho;
end