function [t,xm,ym,xt,yt,nc,miu,miss] = Sharing_KF_Sim(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,Dt,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n,DB,h,S_miss1_mean,S_miss1_SD,Q_n_red,TPM_new)
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
% DB [m] - database of miss distances of the first missile as function of T_sw
% h [m] - the resolution of the database search
% S_miss1_mean [m] - the mean miss distance of the first missile as function of T_sw
% S_miss1_SD [m] - the SD of the miss distance of the first missile as function of T_sw
% Q_n_red - the reduction coeffitante of the value of Q_n(4,4)
% TPM_new - the new transition matrix after the miss of the first missile
% Outputs: 
% t [sec] - vector of the time at each dicreate point
% xm [m] - the x position of the pursuers as function of time
% ym [m] - the y position of the pursuers as function of time
% xt [m] - the x position of the target as function of time
% yt [m] - the y position of the target as function of time
% nc [m/sec^2] - the acceleration commands of the target as function of time
% miu - the modal probabilities calculated by the pursuers
% miss [m,m] - the miss of the first and the second pursuer

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
    nc(i,mis) = DGL1(t_go_est(i,mis),am_n_est(i,mis),at_n_est(i,mis),am_max,at_max,tau,theta,k,v_rho_est(i,mis),dlambda_est(i,mis));
    
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
    
    x_est(:,i,mis)=x_n_n(:,1,mis)*miu(1,i,mis)+x_n_n(:,2,mis)*miu(2,i,mis);
    p_est(:,:,i,mis)=miu(1,i,mis)*(p_n_n(:,:,1,mis)+(x_n_n(:,1,mis)-x_est(:,i,mis))*transpose((x_n_n(:,1,mis)-x_est(:,i,mis))))+miu(2,i,mis)*(p_n_n(:,:,2,mis)+(x_n_n(:,2,mis)-x_est(:,i,mis))*transpose((x_n_n(:,2,mis)-x_est(:,i,mis))));
    
    rho_now(:,mis)=rho(i,mis)*ones(Dt/dt+1,1);
    rho_now_est(:,mis)=rho_est(i,mis)*ones(Dt/dt+1,1);
    rho_prev(:,mis)=rho_now(:,mis);
    rho_prev_est(:,mis)=rho_now_est(:,mis);
end

H_n=zeros(Nmis,4,2,Nmis);
H_n1=[0,-1,0,0];
R_n=eye(2)*delta_m_sig^2;
R_n1=[delta_m_sig^2];
active=[1,1];
runSim=0;

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

                    %%% determine the miss distancce based on the estimation
                    clear rho_prev1_est rho_now1_est
                    rho_prev1_est=abs(rho_prev_est(:,mis));
                    rho_now1_est=abs(rho_now_est(:,mis));
                    rho_prev1_est(rho_prev1_est==0)=[];
                    rho_now1_est(rho_now1_est==0)=[];
                    miss_est=min(min(min(rho_prev1_est),min(rho_now1_est)),min(x_est(1,:,mis)));
                    if miss_est>5
                        %%% if the estimated miss distance is larger than 5
                        %%% meter we estimate the initial mode and the
                        %%% switch time + continue the simulation with
                        %%% reduced value in Q_n(4,4) and new TPM + run
                        %%% inner simulation of an EKF
                        [T_sw_est,i_sw_est,mode_before,mode_after] = switch_time_est(miss_est,DB,h,Dt,miu,S_miss1_mean,S_miss1_SD);
                        runSim=1;
                        pi_mat=TPM_new;
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
    
    clear rho_prev rho_prev_est
    rho_prev=rho_now;
    rho_prev_est=rho_now_est;
    clear rho_now rho_now_est rho_now_est_temp t_now_est t_now_est_temp
    
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
    end    

    % Inner simulation run
    if runSim==1
        i_sim=1;
        dt_sim=1e-3;
        
        Q_n_sim=Q_n;
        Q_n_sim(4,4)=Q_n_sim(4,4)/Q_n_red;
        
        % initialize
        
        x_n_n_sim(:,:)=reshape(x_est(:,1,:),4,Nmis);
        p_n_n_sim(:,:,:)=reshape(p_est(:,:,1,:),4,4,Nmis);
        
        for i_sim=2:i-1
            for mis=1:Nmis  
                if rho(i_sim,1)<4*Dt*(vt+vm)
                    dt_sim=1e-4;       
                end
                
                x_n1_n1_sim=x_n_n_sim;
                p_n1_n1_sim=p_n_n_sim;   
                
                % estimation
                
                %%% estimation for each mode                   
                rho_mix=x_n1_n1_sim(1,mis);
                lambda_mix=x_n1_n1_sim(2,mis);
                gamma_t_mix=x_n1_n1_sim(3,mis);
                at_mix=x_n1_n1_sim(4,mis);

                gamma_m_mix=gamma_m(i_sim-1,mis);

                delta_m_mix=gamma_m_mix-lambda_mix;
                delta_t_mix=gamma_t_mix+lambda_mix;

                v_rho_mix=-(vm*cos(delta_m_mix)+vt*cos(delta_t_mix));
                v_lambda_mix=-vm*sin(delta_m_mix)+vt*sin(delta_t_mix);          

                initial_est=[gamma_m(i_sim-1,mis),am(i_sim-1,mis),rho_mix,lambda_mix,gamma_t_mix,at_mix];
                if i_sim<i_sw_est-0.5 || i_sw_est<0 % chose the targets manuver according to the estimated mode and T_sw
                    help_est=[vm,vt,nc(i_sim-1,mis),2*(mode_before-1.5)*at_max,tau,theta];
                else                        
                    help_est=[vm,vt,nc(i_sim-1,mis),2*(mode_after-1.5)*at_max,tau,theta];
                end
                [time_est,res_est_temp]=ode45(@(t_temp,y) ODE_est(t_temp,y,help_est),[0,Dt],initial_est,odeset('RelTol',1e-9));
                res_est=res_est_temp(end,:);

                rho_j(mis)=res_est(3);
                lambda_j(mis)=res_est(4);
                gamma_t_j(mis)=res_est(5);
                at_j(mis)=res_est(6);

                delta_m_j(mis)=gamma_m(i_sim,mis)-lambda_j(mis);

                if dt_sim<5e-4        
                    %%% the missile close to the target (end-game), so we do coasting
                    dt_ar=time_est(2:end)-time_est(1:end-1);
                    p_n_n1_sim(:,:,mis)=p_n1_n1_sim(:,:,mis);

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

                        p_n_n1_sim(:,:,mis)=phi_n*p_n_n1_sim(:,:,mis)*transpose(phi_n)+Q_n_sim;
                    end
                else
                    F=[0,v_lambda_mix,vt*sin(delta_t_mix),0;
                        -v_lambda_mix/(rho_mix^2),-v_rho_mix/rho_mix,vt*cos(delta_t_mix)/rho_mix,0;
                        0,0,0,1/vt;
                        0,0,0,-1/theta];

                    phi_n=expm(F*Dt);

                    p_n_n1_sim(:,:,mis)=phi_n*p_n1_n1_sim(:,:,mis)*transpose(phi_n)+Q_n_sim;
                end 
                x_n_n1_sim(:,mis)=[rho_j(mis);lambda_j(mis);gamma_t_j(mis);at_j(mis)];

                for mis_temp=1:Nmis
                    dxH=xm(i_sim,mis)-xm(i_sim,mis_temp);
                    dyH=ym(i_sim,mis)-ym(i_sim,mis_temp);
                    drH=sqrt(dxH^2+dyH^2);

                    H_n(mis_temp,1,1,mis)=(dyH*cos(lambda_j(mis))-dxH*sin(lambda_j(mis)))/((drH^2)+(rho_j(mis)^2)+2*rho_j(mis)*(dxH*cos(lambda_j(mis))+dyH*sin(lambda_j(mis))));
                    H_n(mis_temp,2,1,mis)=-rho_j(mis)*(dxH*cos(lambda_j(mis))+dyH*sin(lambda_j(mis))+rho_j(mis))/((drH^2)+(rho_j(mis)^2)+2*rho_j(mis)*(dxH*cos(lambda_j(mis))+dyH*sin(lambda_j(mis))));
                end

                %%% mesuarment estimation
                y_n_n1_sim(:,i_sim,mis)=[gamma_m(i_sim,1)-atan2(ym(i_sim,mis)-ym(i_sim,1)+rho_j(mis)*sin(lambda_j(mis)),xm(i_sim,mis)-xm(i_sim,1)+rho_j(mis)*cos(lambda_j(mis)));
                    gamma_m(i_sim,2)-atan2(ym(i_sim,mis)-ym(i_sim,2)+rho_j(mis)*sin(lambda_j(mis)),xm(i_sim,mis)-xm(i_sim,2)+rho_j(mis)*cos(lambda_j(mis)))];

                S(:,:,1)=H_n(:,:,1,mis)*p_n_n1_sim(:,:,mis)*transpose(H_n(:,:,1,mis))+R_n;

                % better calculation of the inverse and the determinant of S in case that S is symmetric                 
                if abs((S(1,2,1)-S(2,1,1))/S(1,2,1))<1e-3
                    sig_x=sqrt(S(1,1,1));
                    sig_y=sqrt(S(2,2,1));
                    rho_cor=S(1,2,1)/(sig_x*sig_y);

                    det_S=((sig_x*sig_y)^2)*(1-rho_cor^2);
                    inv_S=[S(2,2,1),-S(1,2,1);-S(2,1,1),S(1,1,1)]/det_S;

                    K=p_n_n1_sim(:,:,mis)*transpose(H_n(:,:,1,mis))*inv_S;
                else
                    K=p_n_n1_sim(:,:,mis)*transpose(H_n(:,:,1,mis))*inv(S(:,:,1));
                end

                x_n_n_sim(:,mis)=x_n_n1_sim(:,mis)+K*(y_n_mes(:,i_sim)-y_n_n1_sim(:,i_sim,mis));
                p_n_n_sim(:,:,mis)=(eye(4)-K*H_n(:,:,1,mis))*p_n_n1_sim(:,:,mis)*transpose(eye(4)-K*H_n(:,:,1,mis))+K*R_n*transpose(K);
                                   
                %%% if the differance of one of the estimated measurments and the mesuarment is larger than 4*SD we ignore the measurments
                if abs(y_n_mes(1,i_sim)-y_n_n1_sim(1,i_sim,mis))>4*sqrt(S(1,1,1)) || abs(y_n_mes(2,i_sim)-y_n_n1_sim(2,i_sim,mis))>4*sqrt(S(2,2,1))         
                    x_n_n_sim(:,mis)=x_n_n1_sim(:,mis);
                    p_n_n_sim(:,:,mis)=p_n_n1_sim(:,:,mis);
                end                 
                
                x_est_sim(:,i_sim,mis)=x_n_n_sim(:,mis);
                p_est_sim(:,:,i_sim,mis)=p_n_n_sim(:,:,mis);
                
                [lambda_temp,rho_temp] = fix_lambda_rho(x_est_sim(2,i_sim,mis),x_est_sim(1,i_sim,mis));
                x_est_sim(1,i_sim,mis)=rho_temp;
                x_est_sim(2,i_sim,mis)=lambda_temp;
            end
        end
        
        % return to reality
        for mis=1:Nmis
            x_n1_n1(:,1,mis)=x_est_sim(:,end,mis);
            x_n1_n1(:,2,mis)=x_est_sim(:,end,mis);

            p_n1_n1(:,:,1,mis)=p_est_sim(:,:,end,mis);
            p_n1_n1(:,:,2,mis)=p_est_sim(:,:,end,mis);

            if i_sw_est==-1
                if mode_before==1
                    miu(:,end,mis)=[1-1e-3;1e-3];
                else
                    miu(:,end,mis)=[1e-3;1-1e-3];
                end
            else
                if mode_after==1
                    miu(:,end,mis)=[1-1e-3;1e-3];
                else
                    miu(:,end,mis)=[1e-3;1-1e-3];
                end
            end
        end
        
        runSim=0;        
    end  
    
    % mesurment
    if sum(active)==2
        y_n_mes(:,i)=[delta_m(i,1)+noise_mes(i,1);
            delta_m(i,2)+noise_mes(i,2)];
    else
        %%% if the first missile deactiaveted
        y_n_mes1(i,mis)=delta_m(i,mis)+noise_mes(i,mis);
    end
  
    for mis=1:Nmis
        if active(mis)==0
            continue
        end        

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

            rho_now_est_temp(1:length(res_est_temp(:,3)),j,mis)=res_est_temp(:,3);
            t_now_est_temp(1:length(time_est),j,mis)=time_est;

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

            for mis_temp=1:Nmis
                dxH=xm(i,mis)-xm(i,mis_temp);
                dyH=ym(i,mis)-ym(i,mis_temp);
                drH=sqrt(dxH^2+dyH^2);

                H_n(mis_temp,1,j,mis)=(dyH*cos(lambda_j(j,mis))-dxH*sin(lambda_j(j,mis)))/((drH^2)+(rho_j(j,mis)^2)+2*rho_j(j,mis)*(dxH*cos(lambda_j(j,mis))+dyH*sin(lambda_j(j,mis))));
                H_n(mis_temp,2,j,mis)=-rho_j(j,mis)*(dxH*cos(lambda_j(j,mis))+dyH*sin(lambda_j(j,mis))+rho_j(j,mis))/((drH^2)+(rho_j(j,mis)^2)+2*rho_j(j,mis)*(dxH*cos(lambda_j(j,mis))+dyH*sin(lambda_j(j,mis))));
            end

            if sum(active)==2
                %%% mesuarment estimation
                y_n_n1(:,i,j,mis)=[gamma_m(i,1)-atan2(ym(i,mis)-ym(i,1)+rho_j(j,mis)*sin(lambda_j(j,mis)),xm(i,mis)-xm(i,1)+rho_j(j,mis)*cos(lambda_j(j,mis)));
                    gamma_m(i,2)-atan2(ym(i,mis)-ym(i,2)+rho_j(j,mis)*sin(lambda_j(j,mis)),xm(i,mis)-xm(i,2)+rho_j(j,mis)*cos(lambda_j(j,mis)))];

                S(:,:,j)=H_n(:,:,j,mis)*p_n_n1(:,:,j,mis)*transpose(H_n(:,:,j,mis))+R_n;

                % better calculation of the inverse and the determinant of S in case that S is symmetric                 
                if abs((S(1,2,j)-S(2,1,j))/S(1,2,j))<1e-3
                    sig_x=sqrt(S(1,1,j));
                    sig_y=sqrt(S(2,2,j));
                    rho_cor=S(1,2,j)/(sig_x*sig_y);

                    det_S=((sig_x*sig_y)^2)*(1-rho_cor^2);
                    inv_S=[S(2,2,j),-S(1,2,j);-S(2,1,j),S(1,1,j)]/det_S;

                    %%% mode conditioned likelihood
                    Lambda(j)=(1/(2*pi*sqrt(det_S)))*exp(-((((y_n_mes(1,i)-y_n_n1(1,i,j,mis))/sig_x)^2)+(((y_n_mes(2,i)-y_n_n1(2,i,j,mis))/sig_y)^2)-((2*rho_cor*(y_n_mes(1,i)-y_n_n1(1,i,j,mis))*(y_n_mes(2,i)-y_n_n1(2,i,j,mis)))/(sig_x*sig_y)))/(2*(1-rho_cor^2)));
                    K=p_n_n1(:,:,j,mis)*transpose(H_n(:,:,j,mis))*inv_S;
                else
                    %%% mode conditioned likelihood
                    Lambda(j)=(1/(2*pi*sqrt(det(S(:,:,j)))))*exp(-0.5*transpose(y_n_mes(:,i)-y_n_n1(:,i,j,mis))*inv(S(:,:,j))*(y_n_mes(:,i)-y_n_n1(:,i,j,mis)));
                    K=p_n_n1(:,:,j,mis)*transpose(H_n(:,:,j,mis))*inv(S(:,:,j));
                end

                x_n_n(:,j,mis)=x_n_n1(:,j,mis)+K*(y_n_mes(:,i)-y_n_n1(:,i,j,mis));
                p_n_n(:,:,j,mis)=(eye(4)-K*H_n(:,:,j,mis))*p_n_n1(:,:,j,mis)*transpose(eye(4)-K*H_n(:,:,j,mis))+K*R_n*transpose(K);

            else    
                %%% last missile left - non sharing
                %%% mesuarment estimation
                y_n_n11(i,j,mis)=delta_m_j(j,mis);

                S1(j)=H_n1*p_n_n1(:,:,j,mis)*transpose(H_n1)+R_n1;
                K1=p_n_n1(:,:,j,mis)*transpose(H_n1)/S1(j);
                x_n_n(:,j,mis)=x_n_n1(:,j,mis)+K1*(y_n_mes1(i,mis)-y_n_n11(i,j,mis));
                p_n_n(:,:,j,mis)=(eye(4)-K1*H_n1)*p_n_n1(:,:,j,mis)*transpose(eye(4)-K1*H_n1)+K1*R_n1*transpose(K1);

                %%% mode conditioned likelihood
                Lambda(j)=(1/sqrt(2*pi*det(S1(j))))*exp(-0.5*(y_n_mes1(i,mis)-y_n_n11(i,j,mis))*transpose(y_n_mes1(i,mis)-y_n_n11(i,j,mis))/S1(j));
            end   
        end

        %%% if the differance of one of the estimated measurments and the mesuarment is larger than 4*SD we ignore the measurments
        if sum(active)==2        
            if abs(y_n_mes(1,i)-y_n_n1(1,i,1,mis))>4*sqrt(S(1,1,1)) || abs(y_n_mes(2,i)-y_n_n1(2,i,1,mis))>4*sqrt(S(2,2,1)) || abs(y_n_mes(1,i)-y_n_n1(1,i,2,mis))>4*sqrt(S(1,1,2)) || abs(y_n_mes(2,i)-y_n_n1(2,i,2,mis))>4*sqrt(S(2,2,2))
                Lambda=0*Lambda;            
                x_n_n(:,:,mis)=x_n_n1(:,:,mis);
                p_n_n(:,:,:,mis)=p_n_n1(:,:,:,mis);
            end
        else
            %%% last missile left - non sharing
            if abs(y_n_mes1(i,mis)-y_n_n11(i,1,mis))>4*sqrt(S1(1)) || abs(y_n_mes1(i,mis)-y_n_n11(i,2,mis))>4*sqrt(S1(2))
                Lambda=0*Lambda;            
                x_n_n(:,:,mis)=x_n_n1(:,:,mis);
                p_n_n(:,:,:,mis)=p_n_n1(:,:,:,mis);
            end
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
        x_est(:,i,mis)=x_n_n(:,1,mis)*miu(1,i,mis)+x_n_n(:,2,mis)*miu(2,i,mis);
        p_est(:,:,i,mis)=miu(1,i,mis)*(p_n_n(:,:,1,mis)+(x_n_n(:,1,mis)-x_est(:,i,mis))*transpose((x_n_n(:,1,mis)-x_est(:,i,mis))))+miu(2,i,mis)*(p_n_n(:,:,2,mis)+(x_n_n(:,2,mis)-x_est(:,i,mis))*transpose((x_n_n(:,2,mis)-x_est(:,i,mis))));

        if mean(t_now_est_temp(:,1,mis))<mean(t_now_est_temp(:,2,mis))
            t_now_est(:,mis)=t_now_est_temp(:,2,mis);
        else
            t_now_est(:,mis)=t_now_est_temp(:,1,mis);
        end
        rho_now_est_temp1=rho_now_est_temp(:,1,mis);            
        rho_now_est_temp1(rho_now_est_temp1==0)=[];
        t_now_est_temp1=t_now_est_temp(1:length(rho_now_est_temp1),1,mis);

        rho_now_est_temp2=rho_now_est_temp(:,2,mis);            
        rho_now_est_temp2(rho_now_est_temp2==0)=[];
        t_now_est_temp2=t_now_est_temp(1:length(rho_now_est_temp2),2,mis);

        rho_now_est(:,mis)=interp1(t_now_est_temp1,rho_now_est_temp1,t_now_est(:,mis))*miu(1,i,mis)+interp1(t_now_est_temp2,rho_now_est_temp2,t_now_est(:,mis))*miu(2,i,mis);

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
        nc(i,mis) = DGL1(t_go_est(i,mis),am_n_est(i,mis),at_n_est(i,mis),am_max,at_max,tau,theta,k,v_rho_est(i,mis),dlambda_est(i,mis));

    end    
end

end