function [t,xm,ym,xt,yt,nc,miu,miss,x_true,x_est,p_est] = NonSharing_EstimationCheck(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,Dt,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n)
% Inputs: 
% xm0 [m,m] - vector of initial x position of the persuers 
% ym0 [m,m] - vector of initial x position of the persuers 
% tau [sec] - time constant of the dynamics of the pursers
% am_max [m/sec] - the maximal acceleration command of the pursuers
% vm [m/sec] - velocity of the pursers
% k - parameter of the DGL1 law
% xt0 [m] - initial x position of the target
% yt0 [m] - initial x position of the target
% theta [sec] - time constant of the dynamics of the target
% at_max [m/sec] - the maximal acceleration command of the target
% T_sw [sec] - the switch time of the target's manuver
% vt [m/sec] - velocity of the target
% gamma_t0 [rad] - the initial value of gamma of the target
% xr [m] - the x position of the radar
% yr [m] - the y position of the radar
% dt [sec] - the time step of the simulation (actually acts as indecator 
%            if we at the end game or not)
% Dt [sec] - the time between two samplings
% Nmis - number of the missiles (must be by defualt 2)
% sigmas [m,rad,rad,m/sec^2] - the values of the sqrt of the diagonal values 
%                              of the initial covariance matrix of the radar
% delta_m_sig [rad] - the SD of the pursuer's measurments
% noise_int [m,rad,rad,m/sec^2] - the noise of the radar's measurments
% noise_mes [rad] - the noise of the pursuer's measurments
% pi_mat - the transition matrix between the modes
% Q_n - the tuning matrix for the IMM
% Outputs: 
% t [sec] - vector of the time at each dicreate point
% xm [m] - the x position of the pursuers as function of time
% ym [m] - the y position of the pursuers as function of time
% xt [m] - the x position of the target as function of time
% yt [m] - the y position of the target as function of time
% nc [m/sec^2] - the acceleration commands of the target as function of time
% miu - the modal probabilities calculated by the pursuers
% miss [m,m] - the miss of the first and the second pursuer
% x_true - the true values of the relative vectors
% x_est - the estimated values of the relative vectors
% p_est - the covariance matricies of the estimated relative vectors

% initialize
i=1;
t(i)=0;

%%% target
xt(i)=xt0;
yt(i)=yt0;

gamma_t(i)=gamma_t0;

nt(i)=nt_manuver(t(i)+Dt,at_max,T_sw);
at(i)=0;

%%% radar + radar mesaurments
xtr=xt(i)-xr;
ytr=yt(i)-yr;
rho_r=sqrt(xtr^2+ytr^2);
lambda_r=atan2(ytr,xtr);
gamma_t_r=gamma_t(i);
at_r=at(i);

rho_r_mes=rho_r+noise_int(1);
lambda_r_mes=lambda_r+noise_int(2);
gamma_t_r_mes=gamma_t_r+noise_int(3);
at_r_mes=at_r+noise_int(4);

for mis=1:Nmis
    %%% pursuers + target
    xm(i,mis)=xm0(mis);
    ym(i,mis)=ym0(mis);
    xtm(i,mis)=xt(i)-xm(i,mis);
    ytm(i,mis)=yt(i)-ym(i,mis);
    rho(i,mis)=sqrt(xtm(i,mis)^2+ytm(i,mis)^2);
    
    lambda(i,mis)=atan2(ytm(i,mis),xtm(i,mis));
    gamma_m(i,mis)=lambda(i,mis);
    delta_m(i,mis)=gamma_m(i,mis)-lambda(i,mis);
    delta_t(i,mis)=gamma_t(i)+lambda(i,mis);

    v_rho(i,mis)=-(vm*cos(delta_m(i,mis))+vt*cos(delta_t(i,mis)));
    v_lambda(i,mis)=-vm*sin(delta_m(i,mis))+vt*sin(delta_t(i,mis));

    t_go(i,mis)=-rho(i,mis)/v_rho(i,mis);
    dlambda(i,mis)=v_lambda(i,mis)/rho(i,mis);
    
    am(i,mis)=0;
    am_n(i,mis)=am(i,mis)*cos(delta_m(i,mis));
    at_n(i,mis)=at(i)*cos(delta_t(i,mis));
    
    dx(mis)=xr-xm(i,mis);
    dy(mis)=yr-ym(i,mis);
    dr(mis)=sqrt(dx(mis)^2+dy(mis)^2);
    
    % estimated(=measured) variables    
    rho_est(i,mis)=sqrt(rho_r_mes^2+dr(mis)^2+2*rho_r_mes*(dx(mis)*cos(lambda_r_mes)+dy(mis)*sin(lambda_r_mes)));
    lambda_est(i,mis)=atan2(dy(mis)+rho_r_mes*sin(lambda_r_mes),dx(mis)+rho_r_mes*cos(lambda_r_mes));
    gamma_t_est(i,mis)=gamma_t_r_mes;
    at_est(i,mis)=at_r_mes;
    
    delta_m_est(i,mis)=gamma_m(i,mis)-lambda_est(i,mis);
    delta_t_est(i,mis)=gamma_t_est(i,mis)+lambda_est(i,mis);
    
    v_rho_est(i,mis)=-(vm*cos(delta_m_est(i,mis))+vt*cos(delta_t_est(i,mis)));
    v_lambda_est(i,mis)=-vm*sin(delta_m_est(i,mis))+vt*sin(delta_t_est(i,mis));
    
    t_go_est(i,mis)=-rho_est(i,mis)/v_rho_est(i,mis);
    dlambda_est(i,mis)=v_lambda_est(i,mis)/rho_est(i,mis);
    
    am_n_est(i,mis)=am(i,mis)*cos(delta_m_est(i,mis));
    at_n_est(i,mis)=at_est(i,mis)*cos(delta_t_est(i,mis));
    
    % acceleration command calculation
    nc(i,mis) = DGL1(t_go(i,mis),am_n(i,mis),at_n(i,mis),am_max,at_max,tau,theta,k,v_rho(i,mis),dlambda(i,mis));
    
    % estimation needed variables
    x_n_n(:,1,mis)=[rho_est(i,mis);lambda_est(i,mis);gamma_t_est(i,mis);at_est(i,mis)];
    x_n_n(:,2,mis)=[rho_est(i,mis);lambda_est(i,mis);gamma_t_est(i,mis);at_est(i,mis)]; 
    
    p_n_n_temp=diag(sigmas.^2);
    Gx(1,1)=(rho_r+dx(mis)*cos(lambda_r)+dy(mis)*sin(lambda_r))/rho(i,mis);
    Gx(1,2)=rho_r*(dy(mis)*cos(lambda_r)-dx(mis)*sin(lambda_r))/rho(i,mis);
    Gx(2,1)=(dx(mis)*sin(lambda_r)-dy(mis)*cos(lambda_r))/(rho(i,mis)^2);
    Gx(2,2)=(rho_r^2+rho_r*(dx(mis)*cos(lambda_r)+dy(mis)*sin(lambda_r)))/(rho(i,mis)^2);
    G=[Gx,zeros(2,2);
        zeros(2,2),eye(2,2)];
    p_n_n_temp=G*p_n_n_temp*transpose(G);
    p_n_n(:,:,1,mis)=p_n_n_temp;
    p_n_n(:,:,2,mis)=p_n_n_temp;
    
    miu(:,i,mis)=[0.5;0.5];
    
    x_true(:,i,mis)=[rho(i,mis);lambda(i,mis);gamma_t(i);at(i)];
    x_est(:,i,mis)=x_n_n(:,1,mis)*miu(1,i,mis)+x_n_n(:,2,mis)*miu(2,i,mis);
    p_est(:,:,i,mis)=miu(1,i,mis)*(p_n_n(:,:,1,mis)+(x_n_n(:,1,mis)-x_est(:,i,mis))*transpose((x_n_n(:,1,mis)-x_est(:,i,mis))))+miu(2,i,mis)*(p_n_n(:,:,2,mis)+(x_n_n(:,2,mis)-x_est(:,i,mis))*transpose((x_n_n(:,2,mis)-x_est(:,i,mis))));
    
    rho_now(:,mis)=rho(i,mis)*ones(Dt/dt+1,1);
    rho_prev(:,mis)=rho_now(:,mis);
end

H_n=[0,-1,0,0];
R_n=[delta_m_sig^2];
active=[1,1];

% performing the simulation until all the missiles are deactivated
while t(i)<2*max(max(rho))/(vt+vm)
    %%% check if the distance grows for at least 4 time steps
    count=[0,0];
    for mis=1:Nmis
        if active(mis)==0
            continue
        end
        clear rho_prev1 rho_now1 rho_check
        rho_prev1=rho_prev(:,mis);
        rho_now1=rho_now(:,mis);
        rho_prev1(rho_prev1==0)=[];
        rho_now1(rho_now1==0)=[];
        rho_check=[abs(rho_prev1);abs(rho_now1)];
        for ind=2:length(rho_check)
            if rho_check(ind)>rho_check(ind-1)
                count(mis)=count(mis)+1;
                if count(mis)>3
                    %%% the "mis" missile missed
                    active(mis)=0; % deactivate 
                    miss(mis)=rho_check(ind-4); % set the miss as the miss distance
                    
                    %%% check if the second missile is close to the target
                    clear rho_prev2 rho_now2
                    rho_prev2=abs(rho_prev(:,2));
                    rho_now2=abs(rho_now(:,2));
                    rho_prev2(rho_prev2==0)=[];
                    rho_now2(rho_now2==0)=[];
                    if mis==1 && min(min(rho_prev2),min(rho_now2))>4*Dt*(vt+vm)
                        dt=1e-3; 
                    end
                    break
                end
            else
                count(mis)=0;
            end            
        end
    end
    
    %%% if all of the missiles are deactivated we end the simulation
    if sum(active)==0
        break
    end
     
    %%% if one of the missiles is in close distance to the target, reduce the time step
    if(sum(active)==2 && rho(i,1)<4*Dt*(vt+vm)) || (sum(active)==1 && rho(i,2)<4*Dt*(vt+vm))
        dt=1e-4;       
    end
        
    i=i+1;
    t(i)=t(i-1)+Dt;
    
    % target advancment
    initial_t=[xt(i-1),yt(i-1),gamma_t(i-1),at(i-1)];
    help_t=[vt,nt(i-1),theta];
    [time_t,res_t_temp]=ode45(@(t_temp,y) ODE_t(t_temp,y,help_t),[0,Dt],initial_t,odeset('RelTol',1e-9));
    res_t=res_t_temp(end,:);
    
    xt(i)=res_t(1);
    yt(i)=res_t(2);
    gamma_t(i)=res_t(3);
    at(i)=res_t(4);
    
    nt(i)=nt_manuver(t(i)+Dt,at_max,T_sw);
    
    clear rho_prev
    rho_prev=rho_now;
    clear rho_now
    
    x_n1_n1=x_n_n;
    p_n1_n1=p_n_n;
    
    for mis=1:Nmis
        if active(mis)==0
            continue
        end
        
        % pursuers advancment
        initial_m=[xm(i-1,mis),ym(i-1,mis),gamma_m(i-1,mis),am(i-1,mis),rho(i-1,mis),lambda(i-1,mis),gamma_t(i-1),at(i-1)];
        help_m=[vm,vt,nc(i-1,mis),nt(i-1),tau,theta];
        [time_m,res_m_temp]=ode45(@(t_temp,y) ODE_m(t_temp,y,help_m),[0,Dt],initial_m,odeset('RelTol',1e-9));
        res_m=res_m_temp(end,:);

        xm(i,mis)=res_m(1);
        ym(i,mis)=res_m(2);
        gamma_m(i,mis)=res_m(3);
        am(i,mis)=res_m(4);
        rho(i,mis)=res_m(5);
        lambda(i,mis)=res_m(6);
        
        [lambda_temp,rho_temp] = fix_lambda_rho(lambda(i,mis),1);
        lambda(i,mis)=lambda_temp;
        
        rho_now(1:length(res_m_temp(:,5)),mis)=res_m_temp(:,5);
        
        delta_m(i,mis)=gamma_m(i,mis)-lambda(i,mis);
        delta_t(i,mis)=gamma_t(i)+lambda(i,mis);

        v_rho(i,mis)=-(vm*cos(delta_m(i,mis))+vt*cos(delta_t(i,mis)));
        v_lambda(i,mis)=-vm*sin(delta_m(i,mis))+vt*sin(delta_t(i,mis));

        t_go(i,mis)=-rho(i,mis)/v_rho(i,mis);
        dlambda(i,mis)=v_lambda(i,mis)/rho(i,mis);

        am_n(i,mis)=am(i,mis)*cos(delta_m(i,mis));
        at_n(i,mis)=at(i)*cos(delta_t(i,mis));
        
        % mesurment
        y_n_mes(i,mis)=delta_m(i,mis)+noise_mes(i,mis);
        
        % estimation
        miu_ij(:,:,mis)=[pi_mat(1,1)*miu(1,i-1,mis),pi_mat(1,2)*miu(1,i-1,mis);
            pi_mat(2,1)*miu(2,i-1,mis),pi_mat(2,2)*miu(2,i-1,mis)];
        miu_j1_sum(mis)=sum(miu_ij(:,1,mis));
        miu_j2_sum(mis)=sum(miu_ij(:,2,mis));
        miu_ij(:,:,mis)=[miu_ij(:,1,mis)/miu_j1_sum(mis),miu_ij(:,2,mis)/miu_j2_sum(mis)];
        
        %%% estimation for each mode
        for j=1:2
            x_n1_n1_0=miu_ij(1,j,mis)*x_n1_n1(:,1,mis)+miu_ij(2,j,mis)*x_n1_n1(:,2,mis);
            p_n1_n1_0=miu_ij(1,j,mis)*(p_n1_n1(:,:,1,mis)+(x_n1_n1(:,1,mis)-x_n1_n1_0)*transpose(x_n1_n1(:,1,mis)-x_n1_n1_0))+miu_ij(2,j,mis)*(p_n1_n1(:,:,2,mis)+(x_n1_n1(:,2,mis)-x_n1_n1_0)*transpose(x_n1_n1(:,2,mis)-x_n1_n1_0));
            
            rho_mix=x_n1_n1_0(1);
            lambda_mix=x_n1_n1_0(2);
            gamma_t_mix=x_n1_n1_0(3);
            at_mix=x_n1_n1_0(4);
                
            gamma_m_mix=gamma_m(i-1,mis);
            
            delta_m_mix=gamma_m_mix-lambda_mix;
            delta_t_mix=gamma_t_mix+lambda_mix;
            
            v_rho_mix=-(vm*cos(delta_m_mix)+vt*cos(delta_t_mix));
            v_lambda_mix=-vm*sin(delta_m_mix)+vt*sin(delta_t_mix);          
            
            initial_est=[gamma_m(i-1,mis),am(i-1,mis),rho_mix,lambda_mix,gamma_t_mix,at_mix];
            help_est=[vm,vt,nc(i-1,mis),2*(j-1.5)*at_max,tau,theta];
            [time_est,res_est_temp]=ode45(@(t_temp,y) ODE_est(t_temp,y,help_est),[0,Dt],initial_est,odeset('RelTol',1e-9));
            res_est=res_est_temp(end,:);
            
            rho_j(j,mis)=res_est(3);
            lambda_j(j,mis)=res_est(4);
            gamma_t_j(j,mis)=res_est(5);
            at_j(j,mis)=res_est(6);
            
            delta_m_j(j,mis)=gamma_m(i,mis)-lambda_j(j,mis);
                                  
            if dt<5e-4         
                %%% the missile close to the target (end-game), so we do coasting
                dt_ar=time_est(2:end)-time_est(1:end-1);
                p_n_n1(:,:,j,mis)=p_n1_n1_0;
                
                for ind=1:length(dt_ar)   
                    dt_temp=dt_ar(ind);
                    
                    gamma_m_mix=res_est_temp(ind,1);
                    rho_mix=res_est_temp(ind,3);
                    lambda_mix=res_est_temp(ind,4);
                    gamma_t_mix=res_est_temp(ind,5);
            
                    delta_m_mix=gamma_m_mix-lambda_mix;
                    delta_t_mix=gamma_t_mix+lambda_mix;

                    v_rho_mix=-(vm*cos(delta_m_mix)+vt*cos(delta_t_mix));
                    v_lambda_mix=-vm*sin(delta_m_mix)+vt*sin(delta_t_mix);
                    
                    F=[0,v_lambda_mix,vt*sin(delta_t_mix),0;
                        -v_lambda_mix/(rho_mix^2),-v_rho_mix/rho_mix,vt*cos(delta_t_mix)/rho_mix,0;
                        0,0,0,1/vt;
                        0,0,0,-1/theta];

                    phi_n=expm(F*dt_temp);

                    p_n_n1(:,:,j,mis)=phi_n*p_n_n1(:,:,j,mis)*transpose(phi_n)+Q_n;
                end
            else
                F=[0,v_lambda_mix,vt*sin(delta_t_mix),0;
                    -v_lambda_mix/(rho_mix^2),-v_rho_mix/rho_mix,vt*cos(delta_t_mix)/rho_mix,0;
                    0,0,0,1/vt;
                    0,0,0,-1/theta];

                phi_n=expm(F*Dt);

                p_n_n1(:,:,j,mis)=phi_n*p_n1_n1_0*transpose(phi_n)+Q_n;
            end 
            x_n_n1(:,j,mis)=[rho_j(j,mis);lambda_j(j,mis);gamma_t_j(j,mis);at_j(j,mis)];
            
            %%% mesuarment estimation
            y_n_n1(i,j,mis)=delta_m_j(j,mis);

            S(j)=H_n*p_n_n1(:,:,j,mis)*transpose(H_n)+R_n;
            K=p_n_n1(:,:,j,mis)*transpose(H_n)*inv(S(j));
            x_n_n(:,j,mis)=x_n_n1(:,j,mis)+K*(y_n_mes(i,mis)-y_n_n1(i,j,mis));
            p_n_n(:,:,j,mis)=(eye(4)-K*H_n)*p_n_n1(:,:,j,mis)*transpose(eye(4)-K*H_n)+K*R_n*transpose(K);

            %%% mode conditioned likelihood
            Lambda(j)=(1/sqrt(2*pi*det(S(j))))*exp(-0.5*(y_n_mes(i,mis)-y_n_n1(i,j,mis))*inv(S(j))*transpose(y_n_mes(i,mis)-y_n_n1(i,j,mis)));
        end
        
        %%% if the differance of one of the estimated measurments and the mesuarment is larger than 4*SD we ignore the measurments
        if abs(y_n_mes(i,mis)-y_n_n1(i,1,mis))>4*sqrt(S(1)) || abs(y_n_mes(i,mis)-y_n_n1(i,2,mis))>4*sqrt(S(2))
            Lambda=0*Lambda;            
            x_n_n(:,:,mis)=x_n_n1(:,:,mis);
            p_n_n(:,:,:,mis)=p_n_n1(:,:,:,mis);
        end
        
        %%% Mode Probability Update + Blended Estimation and Covariance
        miu_sum=Lambda(1)*miu_j1_sum(mis)+Lambda(2)*miu_j2_sum(mis);
        
        if isnan(miu_sum)==1 || isinf(miu_sum)==1 || isreal(miu_sum)==0 || miu_sum<=0
            miu(:,i,mis)=miu(:,i-1,mis);
        else
            %%% limitiations on the value of miu
            miu1_temp=Lambda(1)*miu_j1_sum(mis)/miu_sum;
            miu_min=1e-3;
            if miu1_temp<miu_min
                miu1_temp=miu_min;
            end
            if miu1_temp>1-miu_min
                miu1_temp=1-miu_min;
            end
            miu(1,i,mis)=miu1_temp;
            miu(2,i,mis)=1-miu1_temp;
        end
        
        %%% mix the results of the estimation
        x_true(:,i,mis)=[rho(i,mis);lambda(i,mis);gamma_t(i);at(i)];
        x_est(:,i,mis)=x_n_n(:,1,mis)*miu(1,i,mis)+x_n_n(:,2,mis)*miu(2,i,mis);
        p_est(:,:,i,mis)=miu(1,i,mis)*(p_n_n(:,:,1,mis)+(x_n_n(:,1,mis)-x_est(:,i,mis))*transpose((x_n_n(:,1,mis)-x_est(:,i,mis))))+miu(2,i,mis)*(p_n_n(:,:,2,mis)+(x_n_n(:,2,mis)-x_est(:,i,mis))*transpose((x_n_n(:,2,mis)-x_est(:,i,mis))));
        
        [lambda_temp,rho_temp] = fix_lambda_rho(x_est(2,i,mis),x_est(1,i,mis));
        x_est(1,i,mis)=rho_temp;
        x_est(2,i,mis)=lambda_temp;
        
        rho_est(i,mis)=x_est(1,i,mis);
        lambda_est(i,mis)=x_est(2,i,mis);
        gamma_t_est(i,mis)=x_est(3,i,mis);
        at_est(i,mis)=x_est(4,i,mis);        
        
        delta_m_est(i,mis)=gamma_m(i,mis)-lambda_est(i,mis);
        delta_t_est(i,mis)=gamma_t_est(i,mis)+lambda_est(i,mis);

        v_rho_est(i,mis)=-(vm*cos(delta_m_est(i,mis))+vt*cos(delta_t_est(i,mis)));
        v_lambda_est(i,mis)=-vm*sin(delta_m_est(i,mis))+vt*sin(delta_t_est(i,mis));

        t_go_est(i,mis)=abs(-rho_est(i,mis)/v_rho_est(i,mis));
        dlambda_est(i,mis)=v_lambda_est(i,mis)/rho_est(i,mis);

        am_n_est(i,mis)=am(i,mis)*cos(delta_m_est(i,mis));
        at_n_est(i,mis)=at_est(i,mis)*cos(delta_t_est(i,mis));

        % acceleration command calculation
        nc(i,mis) = DGL1(t_go(i,mis),am_n(i,mis),at_n(i,mis),am_max,at_max,tau,theta,k,v_rho(i,mis),dlambda(i,mis));
    end    
end

end

