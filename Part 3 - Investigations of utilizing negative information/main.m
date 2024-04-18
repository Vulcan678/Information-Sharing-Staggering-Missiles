clc
clear all
close all

%% Scenario parameters

g=9.81;

% pursuers
Dx=1500; % m (staggering distance)
xm0=[-50,50]; % m
ym0=[0,-Dx]; % m
tau=0.2; % sec
am_max=45*g; % m/sec^2
vm=2500; % m/sec
k=0.7; % DGL1 law design parameter

% target
xt0=0; % m
yt0=15000; % m
theta=0.2; % sec
at_max=20*g; % m/sec^2
T_sw=2.5; % sec
vt=2500; % m/sec
gamma_t0=-pi/2; % rad

% radar
xr=1000; % m
yr=0; % m

% simulation
dt=1e-3; % sec
dt_samp=1e-2; % sec
Nmis=2;

% noises
sigmas=[50,pi/180,3*pi/180,10]; % [m,rad,rad,m/sec^2]
delta_m_sig=(5e-4); % rad

rng(4)
noise_int=randn(1,4).*sigmas;
noise_mes=randn(400,2).*delta_m_sig;

% estimation variables
pi_mat=[0.99,0.01;0.01,0.99];
Q_n=diag([0.1,1e-9,1e-7,0.1*g].^2);
Q_n_red=1;
TPM_new=pi_mat;
TPM_new_sim=pi_mat;

% switch time estimation variables
h=5e-2;

load("miss1_DB_500.mat");

% expand DB of miss vs T_sw, using fit
dDB=Sharing_miss1(:,2:end)-Sharing_miss1(:,1:end-1);
max_d=max(max(dDB));
M=ceil(5*max_d/h);
DB=zeros(length(Sharing_miss1(:,1)),length([1:1/M:500]));
for i=1:length(Sharing_miss1(:,1))
    f=fit(transpose([1:500]),transpose(Sharing_miss1(i,:)),'pchipinterp');
    DB(i,:)=[f([1:1/M:500])];
end

%% Single run - Test
% % uncomment the type of simulation you want to perform
% 
% [t,xm,ym,xt,yt,nc,miu,miss]=NonSharing(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n);
% % [t,xm,ym,xt,yt,nc,miu,miss,x_true,x_est,p_est] = NonSharing_EstimationCheck(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n);
% % [t,xm,ym,xt,yt,nc,miu,miss]=Sharing(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n);
% % [t,xm,ym,xt,yt,nc,miu,miss,x_true,x_est,p_est] = Sharing_EstimationCheck(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n);
% % [t,xm,ym,xt,yt,nc,miu,miss]=Sharing_SingleMode(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n,DB,h,S_miss1_mean,S_miss1_SD,Q_n_red);
% % [t,xm,ym,xt,yt,nc,miu,miss,x_true,x_est,p_est]=Sharing_SingleMode_EstimationCheck(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n,DB,h,S_miss1_mean,S_miss1_SD,Q_n_red);
% % [t,xm,ym,xt,yt,nc,miu,miss]=Sharing_IMM_Sim(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n,DB,h,S_miss1_mean,S_miss1_SD,Q_n_red,TPM_new,TPM_new_sim);
% % [t,xm,ym,xt,yt,nc,miu,miss,x_true,x_est,p_est]=Sharing_IMM_Sim_EstimationCheck(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n,DB,h,S_miss1_mean,S_miss1_SD,Q_n_red,TPM_new,TPM_new_sim);
% % [t,xm,ym,xt,yt,nc,miu,miss]=Sharing_KF_Sim(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n,DB,h,S_miss1_mean,S_miss1_SD,Q_n_red,TPM_new);
% % [t,xm,ym,xt,yt,nc,miu,miss,x_true,x_est,p_est]=Sharing_KF_Sim_EstimationCheck(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n,DB,h,S_miss1_mean,S_miss1_SD,Q_n_red,TPM_new);
% 
% % find the index when the first missile deactivates (len_1)
% for i=2:length(miu(1,:,1))
%     if miu(1,i,1)==0
%         len_1=i-1;
%         break
%     end
% end
% 
% % output of the miss distances and the time of the end of the interception
% miss
% t(end)
% 
% figure(1)
% plot(xm(1:len_1,1),ym(1:len_1,1),xm(:,2),ym(:,2),xt,yt)
% title('trajectoris')
% legend('Missile 1','Missile 2','Target')
% xlabel("x [m]")
% ylabel("y [m]")
% grid on
% 
% figure(2)
% plot(t(1:len_1),miu(1,1:len_1,1),t,miu(1,:,2))
% title('-20*g mod probebility vs t')
% legend('Missile 1','Missile 2')
% xlabel("t [sec]")
% ylabel("probebilty")
% grid on
% 
% figure(3)
% plot(t(1:len_1),nc(1:len_1,1),t,nc(:,2))
% title('Pursuer Acceleration Command vs t')
% xlabel("t [sec]")
% ylabel("u [m/sec^2]")
% legend('Missile 1','Missile 2')
% grid on

%% Check Estimation - Non Sharing vs Sharing
 
% tic 
% P=200;
% Nmis=2;
% Num=330;
% 
% x_true_array_NS=zeros(4,Num,Nmis,P);
% x_est_array_NS=zeros(4,Num,Nmis,P);
% p_est_array_NS=zeros(4,4,Num,Nmis,P);
% t_array_NS=zeros(Num,P);
% 
% x_true_array_S=zeros(4,Num,Nmis,P);
% x_est_array_S=zeros(4,Num,Nmis,P);
% p_est_array_S=zeros(4,4,Num,Nmis,P);
% t_array_S=zeros(Num,P);
% 
% parfor i=1:P
%     i
%     rng(i);
%     noise_int=randn(1,4).*sigmas;
%     noise_mes=randn(400,2)*delta_m_sig;
% 
%     [t,xm,ym,xt,yt,nc,miu,miss,x_true_NS,x_est_NS,p_est_NS] = NonSharing_EstimationCheck(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n);
% 
%     x_true_NS(2:3,:,:)=(180/pi)*x_true_NS(2:3,:,:);
%     x_est_NS(2:3,:,:)=(180/pi)*x_est_NS(2:3,:,:);
% 
%     x_true_array_NS(:,:,:,i)=x_true_NS(:,1:Num,:);
%     x_est_array_NS(:,:,:,i)=x_est_NS(:,1:Num,:);
%     p_est_array_NS(:,:,:,:,i)=p_est_NS(:,:,1:Num,:);
%     t_array_NS(:,i)=t(1:Num);
% 
%     [t,xm,ym,xt,yt,nc,miu,miss,x_true_S,x_est_S,p_est_S] = Sharing_EstimationCheck(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n);
% 
%     x_true_S(2:3,:,:)=(180/pi)*x_true_S(2:3,:,:);
%     x_est_S(2:3,:,:)=(180/pi)*x_est_S(2:3,:,:);
% 
%     x_true_array_S(:,:,:,i)=x_true_S(:,1:Num,:);
%     x_est_array_S(:,:,:,i)=x_est_S(:,1:Num,:);
%     p_est_array_S(:,:,:,:,i)=p_est_S(:,:,1:Num,:);
%     t_array_S(:,i)=t(1:Num);
% end
% 
% siz=Num;
% x_err_NS=x_est_array_NS(:,1:siz,:,:)-x_true_array_NS(:,1:siz,:,:); % calculate the error of the estimation
% x_err_mean_NS=zeros(4,siz,Nmis);
% x_true_mean_NS=zeros(4,siz,Nmis);
% x_est_mean_NS=zeros(4,siz,Nmis);
% x_err_SD_NS=zeros(4,siz,Nmis);
% x_est_SD_NS=zeros(4,siz,Nmis);
% for mis=1:Nmis
%     % calculate mean
%     for i=1:siz       
%         for k=1:P
%             x_err_mean_NS(:,i,mis)=x_err_mean_NS(:,i,mis)+x_err_NS(:,i,mis,k);
%             x_est_mean_NS(:,i,mis)=x_est_mean_NS(:,i,mis)+x_est_array_NS(:,i,mis,k);
%             x_true_mean_NS(:,i,mis)=x_true_mean_NS(:,i,mis)+x_true_array_NS(:,i,mis,k);
%         end
%         x_err_mean_NS(:,i,mis)=(1/P)*x_err_mean_NS(:,i,mis);
%         x_est_mean_NS(:,i,mis)=(1/P)*x_est_mean_NS(:,i,mis);
%         x_true_mean_NS(:,i,mis)=(1/P)*x_true_mean_NS(:,i,mis);
%     end
% 
%     % calculate SD
%     for i=1:siz
%         for k=1:P
%             x_err_SD_NS(:,i,mis)=x_err_SD_NS(:,i,mis)+(x_err_mean_NS(:,i,mis)-x_err_NS(:,i,mis,k)).^2;
%             x_est_SD_NS(:,i,mis)=x_est_SD_NS(:,i,mis)+(x_est_mean_NS(:,i,mis)-x_est_array_NS(:,i,mis,k)).^2;
%         end
%         x_err_SD_NS(:,i,mis)=(1/(P-1))*x_err_SD_NS(:,i,mis);
%         x_est_SD_NS(:,i,mis)=(1/(P-1))*x_est_SD_NS(:,i,mis);
%         x_err_SD_NS(:,i,mis)=x_err_SD_NS(:,i,mis).^0.5;
%         x_est_SD_NS(:,i,mis)=x_est_SD_NS(:,i,mis).^0.5;
%     end
% 
%     t_ar_NS=t_array_NS(1:siz,1);
% end
% 
% x_err_S=x_est_array_S(:,1:siz,:,:)-x_true_array_S(:,1:siz,:,:); % calculate the error of the estimation
% x_err_mean_S=zeros(4,siz,Nmis);
% x_true_mean_S=zeros(4,siz,Nmis);
% x_est_mean_S=zeros(4,siz,Nmis);
% x_err_SD_S=zeros(4,siz,Nmis);
% x_est_SD_S=zeros(4,siz,Nmis);
% for mis=1:Nmis
%     % calculate mean
%     for i=1:siz
%         for k=1:P
%             x_err_mean_S(:,i,mis)=x_err_mean_S(:,i,mis)+x_err_S(:,i,mis,k);
%             x_est_mean_S(:,i,mis)=x_est_mean_S(:,i,mis)+x_est_array_S(:,i,mis,k);
%             x_true_mean_S(:,i,mis)=x_true_mean_S(:,i,mis)+x_true_array_S(:,i,mis,k);
%         end
%         x_err_mean_S(:,i,mis)=(1/P)*x_err_mean_S(:,i,mis);
%         x_est_mean_S(:,i,mis)=(1/P)*x_est_mean_S(:,i,mis);
%         x_true_mean_S(:,i,mis)=(1/P)*x_true_mean_S(:,i,mis);
%     end
% 
%     % calculate SD
%     for i=1:siz
%         for k=1:P
%             x_err_SD_S(:,i,mis)=x_err_SD_S(:,i,mis)+(x_err_mean_S(:,i,mis)-x_err_S(:,i,mis,k)).^2;
%             x_est_SD_S(:,i,mis)=x_est_SD_S(:,i,mis)+(x_est_mean_S(:,i,mis)-x_est_array_S(:,i,mis,k)).^2;
%         end
%         x_err_SD_S(:,i,mis)=(1/(P-1))*x_err_SD_S(:,i,mis);
%         x_est_SD_S(:,i,mis)=(1/(P-1))*x_est_SD_S(:,i,mis);
%         x_err_SD_S(:,i,mis)=x_err_SD_S(:,i,mis).^0.5;
%         x_est_SD_S(:,i,mis)=x_est_SD_S(:,i,mis).^0.5;
%     end
% 
%     t_ar_S=t_array_S(1:siz,1);
% end
% 
% figure(20)
% subplot(2,2,1);
% plot(t_ar_NS,x_err_mean_NS(1,:,2),'m',t_ar_S,x_err_mean_S(1,:,2),'k')
% title("\rho error mean")
% legend('nonsharing','sharing')
% ylabel('\rho error mean [m]')
% xlabel('Time [sec]')
% grid on
% 
% subplot(2,2,2);
% plot(t_ar_NS,x_err_mean_NS(2,:,2),'m',t_ar_S,x_err_mean_S(2,:,2),'k')
% title("\lambda error mean")
% legend('nonsharing','sharing')
% ylabel('\lambda error mean [deg]')
% xlabel('Time [sec]')
% grid on
% 
% subplot(2,2,3);
% plot(t_ar_NS,x_err_SD_NS(1,:,2),'m',t_ar_S,x_err_SD_S(1,:,2),'k')
% title("\rho error SD")
% ylabel('\rho error SD [m]')
% xlabel('Time [sec]')
% grid on
% 
% subplot(2,2,4);
% plot(t_ar_NS,x_err_SD_NS(2,:,2),'m',t_ar_S,x_err_SD_S(2,:,2),'k')
% title("\lambda error SD")
% ylabel('\lambda error SD [deg]')
% xlabel('Time [sec]')
% grid on
% 
% figure(21)
% subplot(2,2,1);
% plot(t_ar_NS,x_err_mean_NS(3,:,2),'m',t_ar_S,x_err_mean_S(3,:,2),'k')
% title("\gamma_t error mean")
% legend('nonsharing','sharing')
% ylabel('\gamma_t error mean [deg]')
% xlabel('Time [sec]')
% grid on
% 
% subplot(2,2,2);
% plot(t_ar_NS,x_err_mean_NS(4,:,2),'m',t_ar_S,x_err_mean_S(4,:,2),'k')
% title("at error mean")
% legend('nonsharing','sharing')
% ylabel('at error mean [m/sec^2]')
% xlabel('Time [sec]')
% grid on
% 
% subplot(2,2,3);
% plot(t_ar_NS,x_err_SD_NS(3,:,2),'m',t_ar_S,x_err_SD_S(3,:,2),'k')
% title("\gamma_t error SD")
% ylabel('\gamma_t error SD [deg]')
% xlabel('Time [sec]')
% grid on
% 
% subplot(2,2,4);
% plot(t_ar_NS,x_err_SD_NS(4,:,2),'m',t_ar_S,x_err_SD_S(4,:,2),'k')
% title("at error SD")
% ylabel('at error SD [m/sec^2]')
% xlabel('Time [sec]')
% grid on
% 
% figure(24)
% subplot(2,1,1);
% plot(t_ar_NS,x_est_mean_NS(3,:,1),'m--',t_ar_NS,x_est_mean_NS(3,:,2),'k--',t_ar_S,x_est_mean_S(3,:,1),'m',t_ar_S,x_est_mean_S(3,:,2),'k',t_ar_NS,x_true_mean_NS(3,:,1),'g')
% title("\gamma_t mean")
% legend('m1, nonsharing','m2, nonsharing','m1, sharing','m2, sharing','true flight-path')
% ylabel('\gamma_t mean [deg]')
% xlabel('Time [sec]')
% xlim([2.4 3])
% grid on
% 
% subplot(2,1,2);
% plot(t_ar_NS,x_est_SD_NS(3,:,1),'m--',t_ar_NS,x_est_SD_NS(3,:,2),'k--',t_ar_S,x_est_SD_S(3,:,1),'m',t_ar_S,x_est_SD_S(3,:,2),'k')
% title("\gamma_t SD")
% ylabel('\gamma_t SD [deg]')
% xlabel('Time [sec]')
% xlim([2.4 3])
% grid on
% 
% figure(25)
% subplot(2,1,1);
% plot(t_ar_NS,x_est_mean_NS(4,:,1),'m--',t_ar_NS,x_est_mean_NS(4,:,2),'k--',t_ar_S,x_est_mean_S(4,:,1),'m',t_ar_S,x_est_mean_S(4,:,2),'k',t_ar_NS,x_true_mean_NS(4,:,1),'g')
% title("at mean")
% legend('m1, nonsharing','m2, nonsharing','m1, sharing','m2, sharing','true acc.')
% ylabel('at mean [m/sec^2]')
% xlabel('Time [sec]')
% xlim([2.4 3])
% grid on
% 
% subplot(2,1,2);
% plot(t_ar_NS,x_est_SD_NS(4,:,1),'m--',t_ar_NS,x_est_SD_NS(4,:,2),'k--',t_ar_S,x_est_SD_S(4,:,1),'m',t_ar_S,x_est_SD_S(4,:,2),'k')
% title("at SD")
% ylabel('at SD [m/sec^2]')
% xlabel('Time [sec]')
% xlim([2.4 3])
% grid on
% 
% toc

%% CDF creator
% tic
% M=500;
% T_switch=[0.1:0.05:3.4];
% 
% NS_miss=zeros(length(T_switch),3,M);
% S_miss=zeros(length(T_switch),3,M);
% 
% % run the MC simulations for different switch time
% for i=1:M
%     i
%     rng(i)
%     noise_int=randn(1,4).*sigmas;
%     noise_mes=randn(400,2)*delta_m_sig;
%     parfor j=1:length(T_switch)-1
%         T_sw=T_switch(j);
% 
%         [t,xm,ym,xt,yt,nc,miu,miss]=NonSharing(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n);
%         NS_miss(j,:,i)=[miss(1),miss(2),min(miss)];
%         
%         [t,xm,ym,xt,yt,nc,miu,miss]=Sharing(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n);
%         S_miss(j,:,i)=[miss(1),miss(2),min(miss)];
%     end    
%     NS_miss(length(T_switch),:,i)=NS_miss(length(T_switch)-1,:,i);
%     S_miss(length(T_switch),:,i)=S_miss(length(T_switch)-1,:,i);
% end
%  
% for j=1:length(T_switch)    
%     % sort the miss distances per switch time for the firs missile, second missile and the team miss distance
%     S_miss_s(j,:)=sort(S_miss(j,3,:));
%     S_miss2_s(j,:)=sort(S_miss(j,2,:));
%     S_miss1_s(j,:)=sort(S_miss(j,1,:));
%     NS_miss_s(j,:)=sort(NS_miss(j,3,:));
%     NS_miss2_s(j,:)=sort(NS_miss(j,2,:));
%     Single_miss_s(j,:)=sort(NS_miss(j,1,:));
%         
%     % calculate the mean miss per switch time for the firs missile, second missile and the team miss distance
%     S_miss_mean(j)=sum(S_miss_s(j,:))/M;
%     S_miss1_mean(j)=sum(S_miss1_s(j,:))/M;
%     S_miss2_mean(j)=sum(S_miss2_s(j,:))/M;
%     NS_miss_mean(j)=sum(NS_miss_s(j,:))/M;
%     NS_miss2_mean(j)=sum(NS_miss2_s(j,:))/M;
%     Single_miss_mean(j)=sum(Single_miss_s(j,:))/M;
% 
%     % calculate the SD miss per switch time for the firs missile, second missile and the team miss distance
%     S_miss_SD(j)=0;
%     S_miss1_SD(j)=0;
%     S_miss2_SD(j)=0;
%     NS_miss_SD(j)=0;
%     NS_miss2_SD(j)=0;
%     Single_miss_SD(j)=0;
%     for i=1:M
%         S_miss_SD(j)=S_miss_SD(j)+(S_miss_mean(j)-S_miss_s(j,i))^2;
%         S_miss1_SD(j)=S_miss1_SD(j)+(S_miss1_mean(j)-S_miss1_s(j,i))^2;
%         S_miss2_SD(j)=S_miss2_SD(j)+(S_miss2_mean(j)-S_miss2_s(j,i))^2;
%         NS_miss_SD(j)=NS_miss_SD(j)+(NS_miss_mean(j)-NS_miss_s(j,i))^2;
%         NS_miss2_SD(j)=NS_miss2_SD(j)+(NS_miss2_mean(j)-NS_miss2_s(j,i))^2;
%         Single_miss_SD(j)=Single_miss_SD(j)+(Single_miss_mean(j)-Single_miss_s(j,i))^2;
%     end
%     S_miss_SD(j)=sqrt(S_miss_SD(j)/(M-1));
%     S_miss1_SD(j)=sqrt(S_miss1_SD(j)/(M-1));
%     S_miss2_SD(j)=sqrt(S_miss2_SD(j)/(M-1));
%     NS_miss_SD(j)=sqrt(NS_miss_SD(j)/(M-1)); 
%     NS_miss2_SD(j)=sqrt(NS_miss2_SD(j)/(M-1)); 
%     Single_miss_SD(j)=sqrt(Single_miss_SD(j)/(M-1));   
% end
% 
% % create the values for the CDF figure for single missile, Non Sharing and Sharing missiles
% for i=1:M
%     Single_miss_s_comb(i)=sum(Single_miss_s(:,i))/length(T_switch);
%     NS_miss_s_comb(i)=sum(NS_miss_s(:,i))/length(T_switch);
%     S_miss_s_comb(i)=sum(S_miss_s(:,i))/length(T_switch);
% end
% 
% cdf_array=[1:M]/M;
% 
% figure(25)
% subplot(2,1,1);
% plot(T_switch,Single_miss_mean,'m',T_switch,S_miss1_mean,'k')
% title("Miss Distance Mean Of Missile 1")
% legend('nonsharing','sharing')
% ylabel('mean miss distance [m]')
% xlabel('Switch Time [sec]')
% grid on
% 
% subplot(2,1,2);
% plot(T_switch,Single_miss_SD,'m',T_switch,S_miss1_SD,'k')
% title("Miss Distance SD Of Missile 1")
% legend('nonsharing','sharing')
% ylabel('SD miss distance [m]')
% xlabel('Switch Time [sec]')
% grid on
% 
% figure(26)
% subplot(2,1,1);
% plot(T_switch,NS_miss2_mean,'m',T_switch,S_miss2_mean,'k')
% title("Miss Distance Mean Of Missile 2")
% legend('nonsharing','sharing')
% ylabel('mean miss distance [m]')
% xlabel('Switch Time [sec]')
% grid on
% 
% subplot(2,1,2);
% plot(T_switch,NS_miss2_SD,'m',T_switch,S_miss2_SD,'k')
% title("Miss Distance SD Of Missile 2")
% legend('nonsharing','sharing')
% ylabel('SD miss distance [m]')
% xlabel('Switch Time [sec]')
% grid on
% 
% figure(27)
% subplot(2,1,1);
% plot(T_switch,NS_miss_mean,'m',T_switch,S_miss_mean,'k')
% title("Team Miss Distance Mean")
% legend('nonsharing','sharing')
% ylabel('mean miss distance [m]')
% xlabel('Switch Time [sec]')
% grid on
% 
% subplot(2,1,2);
% plot(T_switch,NS_miss_SD,'m',T_switch,S_miss_SD,'k')
% title("Team Miss Distance SD")
% legend('nonsharing','sharing')
% ylabel('SD miss distance [m]')
% xlabel('Switch Time [sec]')
% grid on
% 
% figure(28)
% plot(S_miss_s_comb,cdf_array,'k',NS_miss_s_comb,cdf_array,'m',Single_miss_s_comb,cdf_array,'g')
% xlabel('Miss Distance (m)')
% ylabel('Miss CDF')
% title('Compare - Miss Distance CDF')
% legend('DGL1, sharing','DGL1, nonsharing','DGL1, single missile')
% grid on
% 
% save("resualt_CDF")
% 
% toc

%% New methods Check Estimation - same idea as before, but comparing with different scenarios
 
% tic
% 
% P=200;
% Nmis=2;
% Num=330;
% 
% Q_n_red_ar=[1,1e2];
% TPM_new_ar(:,:,1)=[0.99,0.01;0.01,0.99];
% TPM_new_ar(:,:,2)=[1-1e-3,1e-3;1e-3,1-1e-3];
% TPM_new_sim_ar=TPM_new_ar;
% 
% Q_n_len=length(Q_n_red_ar);
% TPM_len=length(TPM_new_ar(1,1,:));
% TPM_sim_len=length(TPM_new_sim_ar(1,1,:));
% 
% x_true_array_NS=zeros(4,Num,Nmis,P);
% x_est_array_NS=zeros(4,Num,Nmis,P);
% p_est_array_NS=zeros(4,4,Num,Nmis,P);
% t_array_NS=zeros(Num,P);
% 
% x_true_array_S=zeros(4,Num,Nmis,P);
% x_est_array_S=zeros(4,Num,Nmis,P);
% p_est_array_S=zeros(4,4,Num,Nmis,P);
% t_array_S=zeros(Num,P);
% 
% x_true_array_SM=zeros(4,Num,Nmis,P,Q_n_len);
% x_est_array_SM=zeros(4,Num,Nmis,P,Q_n_len);
% p_est_array_SM=zeros(4,4,Num,Nmis,P,Q_n_len);
% t_array_SM=zeros(Num,P,Q_n_len);
% 
% x_true_array_Sim_KF=zeros(4,Num,Nmis,P,Q_n_len,TPM_len);
% x_est_array_Sim_KF=zeros(4,Num,Nmis,P,Q_n_len,TPM_len);
% p_est_array_Sim_KF=zeros(4,4,Num,Nmis,P,Q_n_len,TPM_len);
% t_array_Sim_KF=zeros(Num,P,Q_n_len,TPM_len);
% 
% x_true_array_Sim_IMM=zeros(4,Num,Nmis,P,Q_n_len,TPM_len,TPM_sim_len);
% x_est_array_Sim_IMM=zeros(4,Num,Nmis,P,Q_n_len,TPM_len,TPM_sim_len);
% p_est_array_Sim_IMM=zeros(4,4,Num,Nmis,P,Q_n_len,TPM_len,TPM_sim_len);
% t_array_Sim_IMM=zeros(Num,P,Q_n_len,TPM_len,TPM_sim_len);
% 
% parfor i=1:P
%     i
%     rng(i);
%     noise_int=randn(1,4).*sigmas;
%     noise_mes=randn(400,2)*delta_m_sig; 
%     
%     [t,xm,ym,xt,yt,nc,miu,miss,x_true_NS,x_est_NS,p_est_NS] = NonSharing_EstimationCheck(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n);
% 
%     x_true_NS(2:3,:,:)=(180/pi)*x_true_NS(2:3,:,:);
%     x_est_NS(2:3,:,:)=(180/pi)*x_est_NS(2:3,:,:);
% 
%     x_true_array_NS(:,:,:,i)=x_true_NS(:,1:Num,:);
%     x_est_array_NS(:,:,:,i)=x_est_NS(:,1:Num,:);
%     p_est_array_NS(:,:,:,:,i)=p_est_NS(:,:,1:Num,:);
%     t_array_NS(:,i)=t(1:Num);
% 
%     [t,xm,ym,xt,yt,nc,miu,miss,x_true_S,x_est_S,p_est_S] = Sharing_EstimationCheck(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n);
% 
%     x_true_S(2:3,:,:)=(180/pi)*x_true_S(2:3,:,:);
%     x_est_S(2:3,:,:)=(180/pi)*x_est_S(2:3,:,:);
% 
%     x_true_array_S(:,:,:,i)=x_true_S(:,1:Num,:);
%     x_est_array_S(:,:,:,i)=x_est_S(:,1:Num,:);
%     p_est_array_S(:,:,:,:,i)=p_est_S(:,:,1:Num,:);
%     t_array_S(:,i)=t(1:Num);
%     
%     for p=1:Q_n_len
%         Q_n_red=Q_n_red_ar(p);
%         
%         [t,xm,ym,xt,yt,nc,miu,miss,x_true_SM,x_est_SM,p_est_SM] = Sharing_SingleMode_EstimationCheck(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n,DB,h,S_miss1_mean,S_miss1_SD,Q_n_red);
% 
%         x_true_SM(2:3,:,:)=(180/pi)*x_true_SM(2:3,:,:);
%         x_est_SM(2:3,:,:)=(180/pi)*x_est_SM(2:3,:,:);
% 
%         x_true_array_SM(:,:,:,i,p)=x_true_SM(:,1:Num,:);
%         x_est_array_SM(:,:,:,i,p)=x_est_SM(:,1:Num,:);
%         p_est_array_SM(:,:,:,:,i,p)=p_est_SM(:,:,1:Num,:);
%         t_array_SM(:,i,p)=t(1:Num);
%         
%         for n=1:TPM_len
%             TPM_new=TPM_new_ar(:,:,n);            
%             
%                 [t,xm,ym,xt,yt,nc,miu,miss,x_true_Sim_KF,x_est_Sim_KF,p_est_Sim_KF] = Sharing_KF_Sim_EstimationCheck(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n,DB,h,S_miss1_mean,S_miss1_SD,Q_n_red,TPM_new);
% 
%                 x_true_Sim_KF(2:3,:,:)=(180/pi)*x_true_Sim_KF(2:3,:,:);
%                 x_est_Sim_KF(2:3,:,:)=(180/pi)*x_est_Sim_KF(2:3,:,:);
% 
%                 x_true_array_Sim_KF(:,:,:,i,p,n)=x_true_Sim_KF(:,1:Num,:);
%                 x_est_array_Sim_KF(:,:,:,i,p,n)=x_est_Sim_KF(:,1:Num,:);
%                 p_est_array_Sim_KF(:,:,:,:,i,p,n)=p_est_Sim_KF(:,:,1:Num,:);
%                 t_array_Sim_KF(:,i,p,n)=t(1:Num);
%                 
%                     for m=1:TPM_sim_len
%                         TPM_new_sim=TPM_new_sim_ar(:,:,m);
% 
%                         [t,xm,ym,xt,yt,nc,miu,miss,x_true_Sim_IMM,x_est_Sim_IMM,p_est_Sim_IMM] = Sharing_IMM_Sim_EstimationCheck(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n,DB,h,S_miss1_mean,S_miss1_SD,Q_n_red,TPM_new,TPM_new_sim);
% 
%                         x_true_Sim_IMM(2:3,:,:)=(180/pi)*x_true_Sim_IMM(2:3,:,:);
%                         x_est_Sim_IMM(2:3,:,:)=(180/pi)*x_est_Sim_IMM(2:3,:,:);
% 
%                         x_true_array_Sim_IMM(:,:,:,i,p,n,m)=x_true_Sim_IMM(:,1:Num,:);
%                         x_est_array_Sim_IMM(:,:,:,i,p,n,m)=x_est_Sim_IMM(:,1:Num,:);
%                         p_est_array_Sim_IMM(:,:,:,:,i,p,n,m)=p_est_Sim_IMM(:,:,1:Num,:);
%                         t_array_Sim_IMM(:,i,p,n,m)=t(1:Num);
% 
%                     end             
%         end
%     end
% end
% 
% siz=Num;
% x_err_NS=x_est_array_NS(:,1:siz,:,:)-x_true_array_NS(:,1:siz,:,:);
% x_err_mean_NS=zeros(4,siz,Nmis);
% x_true_mean_NS=zeros(4,siz,Nmis);
% x_est_mean_NS=zeros(4,siz,Nmis);
% x_err_SD_NS=zeros(4,siz,Nmis);
% x_est_SD_NS=zeros(4,siz,Nmis);
% for mis=1:Nmis
%     for i=1:siz
%         for k=1:P
%             x_err_mean_NS(:,i,mis)=x_err_mean_NS(:,i,mis)+x_err_NS(:,i,mis,k);
%             x_est_mean_NS(:,i,mis)=x_est_mean_NS(:,i,mis)+x_est_array_NS(:,i,mis,k);
%             x_true_mean_NS(:,i,mis)=x_true_mean_NS(:,i,mis)+x_true_array_NS(:,i,mis,k);
%         end
%         x_err_mean_NS(:,i,mis)=(1/P)*x_err_mean_NS(:,i,mis);
%         x_est_mean_NS(:,i,mis)=(1/P)*x_est_mean_NS(:,i,mis);
%         x_true_mean_NS(:,i,mis)=(1/P)*x_true_mean_NS(:,i,mis);
%     end
% 
%     % calculate SD
%     for i=1:siz
%         for k=1:P
%             x_err_SD_NS(:,i,mis)=x_err_SD_NS(:,i,mis)+(x_err_mean_NS(:,i,mis)-x_err_NS(:,i,mis,k)).^2;
%             x_est_SD_NS(:,i,mis)=x_est_SD_NS(:,i,mis)+(x_est_mean_NS(:,i,mis)-x_est_array_NS(:,i,mis,k)).^2;
%         end
%         x_err_SD_NS(:,i,mis)=(1/(P-1))*x_err_SD_NS(:,i,mis);
%         x_est_SD_NS(:,i,mis)=(1/(P-1))*x_est_SD_NS(:,i,mis);
%         x_err_SD_NS(:,i,mis)=x_err_SD_NS(:,i,mis).^0.5;
%         x_est_SD_NS(:,i,mis)=x_est_SD_NS(:,i,mis).^0.5;
%     end
% 
%     t_ar_NS=t_array_NS(1:siz,1);
% end
% 
% x_err_S=x_est_array_S(:,1:siz,:,:)-x_true_array_S(:,1:siz,:,:);
% x_err_mean_S=zeros(4,siz,Nmis);
% x_true_mean_S=zeros(4,siz,Nmis);
% x_est_mean_S=zeros(4,siz,Nmis);
% x_err_SD_S=zeros(4,siz,Nmis);
% x_est_SD_S=zeros(4,siz,Nmis);
% for mis=1:Nmis
%     for i=1:siz
%         for k=1:P
%             x_err_mean_S(:,i,mis)=x_err_mean_S(:,i,mis)+x_err_S(:,i,mis,k);
%             x_est_mean_S(:,i,mis)=x_est_mean_S(:,i,mis)+x_est_array_S(:,i,mis,k);
%             x_true_mean_S(:,i,mis)=x_true_mean_S(:,i,mis)+x_true_array_S(:,i,mis,k);
%         end
%         x_err_mean_S(:,i,mis)=(1/P)*x_err_mean_S(:,i,mis);
%         x_est_mean_S(:,i,mis)=(1/P)*x_est_mean_S(:,i,mis);
%         x_true_mean_S(:,i,mis)=(1/P)*x_true_mean_S(:,i,mis);
%     end
% 
%     % calculate SD
%     for i=1:siz
%         for k=1:P
%             x_err_SD_S(:,i,mis)=x_err_SD_S(:,i,mis)+(x_err_mean_S(:,i,mis)-x_err_S(:,i,mis,k)).^2;
%             x_est_SD_S(:,i,mis)=x_est_SD_S(:,i,mis)+(x_est_mean_S(:,i,mis)-x_est_array_S(:,i,mis,k)).^2;
%         end
%         x_err_SD_S(:,i,mis)=(1/(P-1))*x_err_SD_S(:,i,mis);
%         x_est_SD_S(:,i,mis)=(1/(P-1))*x_est_SD_S(:,i,mis);
%         x_err_SD_S(:,i,mis)=x_err_SD_S(:,i,mis).^0.5;
%         x_est_SD_S(:,i,mis)=x_est_SD_S(:,i,mis).^0.5;
%     end
% 
%     t_ar_S=t_array_S(1:siz,1);
% end
% 
% siz=Num;
% x_err_SM=x_est_array_SM(:,1:siz,:,:,:)-x_true_array_SM(:,1:siz,:,:,:);
% x_err_mean_SM=zeros(4,siz,Nmis,Q_n_len);
% x_true_mean_SM=zeros(4,siz,Nmis,Q_n_len);
% x_est_mean_SM=zeros(4,siz,Nmis,Q_n_len);
% x_err_SD_SM=zeros(4,siz,Nmis,Q_n_len);
% x_est_SD_SM=zeros(4,siz,Nmis,Q_n_len);
% 
% x_err_Sim_KF=x_est_array_Sim_KF(:,1:siz,:,:,:,:)-x_true_array_Sim_KF(:,1:siz,:,:,:,:);
% x_err_mean_Sim_KF=zeros(4,siz,Nmis,Q_n_len,TPM_len);
% x_true_mean_Sim_KF=zeros(4,siz,Nmis,Q_n_len,TPM_len);
% x_est_mean_Sim_KF=zeros(4,siz,Nmis,Q_n_len,TPM_len);
% x_err_SD_Sim_KF=zeros(4,siz,Nmis,Q_n_len,TPM_len);
% x_est_SD_Sim_KF=zeros(4,siz,Nmis,Q_n_len,TPM_len);
% 
% x_err_Sim_IMM=x_est_array_Sim_IMM(:,1:siz,:,:,:,:,:)-x_true_array_Sim_IMM(:,1:siz,:,:,:,:,:);
% x_err_mean_Sim_IMM=zeros(4,siz,Nmis,Q_n_len,TPM_len,TPM_sim_len);
% x_true_mean_Sim_IMM=zeros(4,siz,Nmis,Q_n_len,TPM_len,TPM_sim_len);
% x_est_mean_Sim_IMM=zeros(4,siz,Nmis,Q_n_len,TPM_len,TPM_sim_len);
% x_err_SD_Sim_IMM=zeros(4,siz,Nmis,Q_n_len,TPM_len,TPM_sim_len);
% x_est_SD_Sim_IMM=zeros(4,siz,Nmis,Q_n_len,TPM_len,TPM_sim_len);
% 
% for p=1:Q_n_len
% 
%     % calculate mean
%     for mis=1:Nmis
%         for i=1:siz
%             for k=1:P
%                 x_err_mean_SM(:,i,mis,p)=x_err_mean_SM(:,i,mis,p)+x_err_SM(:,i,mis,k,p);
%                 x_est_mean_SM(:,i,mis,p)=x_est_mean_SM(:,i,mis,p)+x_est_array_SM(:,i,mis,k,p);
%                 x_true_mean_SM(:,i,mis,p)=x_true_mean_SM(:,i,mis,p)+x_true_array_SM(:,i,mis,k,p);
%             end
%             x_err_mean_SM(:,i,mis,p)=(1/P)*x_err_mean_SM(:,i,mis,p);
%             x_est_mean_SM(:,i,mis,p)=(1/P)*x_est_mean_SM(:,i,mis,p);
%             x_true_mean_SM(:,i,mis,p)=(1/P)*x_true_mean_SM(:,i,mis,p);
%         end
% 
%         % calculate SD
%         for i=1:siz
%             for k=1:P
%                 x_err_SD_SM(:,i,mis,p)=x_err_SD_SM(:,i,mis,p)+(x_err_mean_SM(:,i,mis,p)-x_err_SM(:,i,mis,k,p)).^2;
%                 x_est_SD_SM(:,i,mis,p)=x_est_SD_SM(:,i,mis,p)+(x_est_mean_SM(:,i,mis,p)-x_est_array_SM(:,i,mis,k,p)).^2;
%             end
%             x_err_SD_SM(:,i,mis,p)=(1/(P-1))*x_err_SD_SM(:,i,mis,p);
%             x_est_SD_SM(:,i,mis,p)=(1/(P-1))*x_est_SD_SM(:,i,mis,p);
%             x_err_SD_SM(:,i,mis,p)=x_err_SD_SM(:,i,mis,p).^0.5;
%             x_est_SD_SM(:,i,mis,p)=x_est_SD_SM(:,i,mis,p).^0.5;
%         end
% 
%         t_ar_SM=t_array_SM(1:siz,1,p);
%     end
%     
%     for n=1:TPM_len
%         
%         for mis=1:Nmis
%             for i=1:siz
%                 for k=1:P
%                     x_err_mean_Sim_KF(:,i,mis,p,n)=x_err_mean_Sim_KF(:,i,mis,p,n,r)+x_err_Sim_KF(:,i,mis,k,p,n,r);
%                     x_est_mean_Sim_KF(:,i,mis,p,n)=x_est_mean_Sim_KF(:,i,mis,p,n,r)+x_est_array_Sim_KF(:,i,mis,k,p,n,r);
%                     x_true_mean_Sim_KF(:,i,mis,p,n)=x_true_mean_Sim_KF(:,i,mis,p,n,r)+x_true_array_Sim_KF(:,i,mis,k,p,n,r);
%                 end
%                 x_err_mean_Sim_KF(:,i,mis,p,n)=(1/P)*x_err_mean_Sim_KF(:,i,mis,p,n,r);
%                 x_est_mean_Sim_KF(:,i,mis,p,n)=(1/P)*x_est_mean_Sim_KF(:,i,mis,p,n,r);
%                 x_true_mean_Sim_KF(:,i,mis,p,n)=(1/P)*x_true_mean_Sim_KF(:,i,mis,p,n,r);
%             end
% 
%             % calculate SD
%             for i=1:siz
%                 for k=1:P
%                     x_err_SD_Sim_KF(:,i,mis,p,n)=x_err_SD_Sim_KF(:,i,mis,p,n)+(x_err_mean_Sim_KF(:,i,mis,p,n)-x_err_Sim_KF(:,i,mis,k,p,n)).^2;
%                     x_est_SD_Sim_KF(:,i,mis,p,n)=x_est_SD_Sim_KF(:,i,mis,p,n)+(x_est_mean_Sim_KF(:,i,mis,p,n)-x_est_array_Sim_KF(:,i,mis,k,p,n)).^2;
%                 end
%                 x_err_SD_Sim_KF(:,i,mis,p,n)=(1/(P-1))*x_err_SD_Sim_KF(:,i,mis,p,n);
%                 x_est_SD_Sim_KF(:,i,mis,p,n)=(1/(P-1))*x_est_SD_Sim_KF(:,i,mis,p,n);
%                 x_err_SD_Sim_KF(:,i,mis,p,n)=x_err_SD_Sim_KF(:,i,mis,p,n).^0.5;
%                 x_est_SD_Sim_KF(:,i,mis,p,n)=x_est_SD_Sim_KF(:,i,mis,p,n).^0.5;
%             end
% 
%             t_ar_Sim_KF=t_array_Sim_KF(1:siz,1,p,n);
%         end
%         
%         for m=1:TPM_sim_len
%             
%             for mis=1:Nmis
%                 for i=1:siz
%                     for k=1:P
%                         x_err_mean_Sim_IMM(:,i,mis,p,n,m)=x_err_mean_Sim_IMM(:,i,mis,p,n,m)+x_err_Sim_IMM(:,i,mis,k,p,n,m);
%                         x_est_mean_Sim_IMM(:,i,mis,p,n,m)=x_est_mean_Sim_IMM(:,i,mis,p,n,m)+x_est_array_Sim_IMM(:,i,mis,k,p,n,m);
%                         x_true_mean_Sim_IMM(:,i,mis,p,n,m)=x_true_mean_Sim_IMM(:,i,mis,p,n,m)+x_true_array_Sim_IMM(:,i,mis,k,p,n,m);
%                     end
%                     x_err_mean_Sim_IMM(:,i,mis,p,n,m)=(1/P)*x_err_mean_Sim_IMM(:,i,mis,p,n,m);
%                     x_est_mean_Sim_IMM(:,i,mis,p,n,m)=(1/P)*x_est_mean_Sim_IMM(:,i,mis,p,n,m);
%                     x_true_mean_Sim_IMM(:,i,mis,p,n,m)=(1/P)*x_true_mean_Sim_IMM(:,i,mis,p,n,m);
%                 end
% 
%                 % calculate SD
%                 for i=1:siz
%                     for k=1:P
%                         x_err_SD_Sim_IMM(:,i,mis,p,n,m)=x_err_SD_Sim_IMM(:,i,mis,p,n,m)+(x_err_mean_Sim_IMM(:,i,mis,p,n,m)-x_err_Sim_IMM(:,i,mis,k,p,n,m)).^2;
%                         x_est_SD_Sim_IMM(:,i,mis,p,n,m)=x_est_SD_Sim_IMM(:,i,mis,p,n,m)+(x_est_mean_Sim_IMM(:,i,mis,p,n,m)-x_est_array_Sim_IMM(:,i,mis,k,p,n,m)).^2;
%                     end
%                     x_err_SD_Sim_IMM(:,i,mis,p,n,m)=(1/(P-1))*x_err_SD_Sim_IMM(:,i,mis,p,n,m);
%                     x_est_SD_Sim_IMM(:,i,mis,p,n,m)=(1/(P-1))*x_est_SD_Sim_IMM(:,i,mis,p,n,m);
%                     x_err_SD_Sim_IMM(:,i,mis,p,n,m)=x_err_SD_Sim_IMM(:,i,mis,p,n,m).^0.5;
%                     x_est_SD_Sim_IMM(:,i,mis,p,n,m)=x_est_SD_Sim_IMM(:,i,mis,p,n,m).^0.5;
%                 end
% 
%                 t_ar_Sim_IMM=t_array_Sim_IMM(1:siz,1,p,n,m);
%             end
%         end
%     end
% end
% 
% for p=1:Q_n_len
%     
%     figure(p*1e2+20)
%     subplot(2,2,1);
%     plot(t_ar_NS,x_err_mean_NS(1,:,2),'m',t_ar_S,x_err_mean_S(1,:,2),'k',t_ar_SM,x_err_mean_SM(1,:,2,p),'b')
%     title("\rho error mean")
%     legend('nonsharing','sharing','sharing - single mode')
%     ylabel('\rho error mean [m]')
%     xlabel('Time [sec]')
%     xlim([2.6,3.4])
%     grid on
% 
%     subplot(2,2,2);
%     plot(t_ar_NS,x_err_mean_NS(2,:,2),'m',t_ar_S,x_err_mean_S(2,:,2),'k',t_ar_SM,x_err_mean_SM(2,:,2,p),'b')
%     title("\lambda error mean")
%     legend('nonsharing','sharing','sharing - single mode')
%     ylabel('\lambda error mean [deg]')
%     xlabel('Time [sec]')
%     xlim([2.6,3.4])
%     grid on
% 
%     subplot(2,2,3);
%     plot(t_ar_NS,x_err_SD_NS(1,:,2),'m',t_ar_S,x_err_SD_S(1,:,2),'k',t_ar_SM,x_err_SD_SM(1,:,2,p),'b')
%     title("\rho error SD")
%     ylabel('\rho error SD [m]')
%     xlabel('Time [sec]')
%     xlim([2.6,3.4])
%     grid on
% 
%     subplot(2,2,4);
%     plot(t_ar_NS,x_err_SD_NS(2,:,2),'m',t_ar_S,x_err_SD_S(2,:,2),'k',t_ar_SM,x_err_SD_SM(2,:,2,p),'b')
%     title("\lambda error SD")
%     ylabel('\lambda error SD [deg]')
%     xlabel('Time [sec]')
%     xlim([2.6,3.4])
%     grid on
% 
%     figure(p*1e2+21)
%     subplot(2,2,1);
%     plot(t_ar_NS,x_err_mean_NS(3,:,2),'m',t_ar_S,x_err_mean_S(3,:,2),'k',t_ar_SM,x_err_mean_SM(3,:,2,p),'b')
%     title("\gamma_t error mean")
%     legend('nonsharing','sharing','sharing - single mode')
%     ylabel('\gamma_t error mean [deg]')
%     xlabel('Time [sec]')
%     xlim([2.6,3.4])
%     grid on
% 
%     subplot(2,2,2);
%     plot(t_ar_NS,x_err_mean_NS(4,:,2),'m',t_ar_S,x_err_mean_S(4,:,2),'k',t_ar_SM,x_err_mean_SM(4,:,2,p),'b')
%     title("at error mean")
%     legend('nonsharing','sharing','sharing - single mode')
%     ylabel('at error mean [m/sec^2]')
%     xlabel('Time [sec]')
%     xlim([2.6,3.4])
%     grid on
% 
%     subplot(2,2,3);
%     plot(t_ar_NS,x_err_SD_NS(3,:,2),'m',t_ar_S,x_err_SD_S(3,:,2),'k',t_ar_SM,x_err_SD_SM(3,:,2,p),'b')
%     title("\gamma_t error SD")
%     ylabel('\gamma_t error SD [deg]')
%     xlabel('Time [sec]')
%     xlim([2.6,3.4])
%     grid on
% 
%     subplot(2,2,4);
%     plot(t_ar_NS,x_err_SD_NS(4,:,2),'m',t_ar_S,x_err_SD_S(4,:,2),'k',t_ar_SM,x_err_SD_SM(4,:,2,p),'b')
%     title("at error SD")
%     ylabel('at error SD [m/sec^2]')
%     xlabel('Time [sec]')
%     xlim([2.6,3.4])
%     grid on
% end
% 
% for p=1:Q_n_len
%     for n=1:TPM_len   
%         
%         figure(n*1e3+p*1e2+20)
%         subplot(2,2,1);
%         plot(t_ar_NS,x_err_mean_NS(1,:,2),'m',t_ar_S,x_err_mean_S(1,:,2),'k',t_ar_Sim_KF,x_err_mean_Sim_KF(1,:,2,p,n),'b')
%         title("\rho error mean")
%         legend('nonsharing','sharing','sharing - KF simulation')
%         ylabel('\rho error mean [m]')
%         xlabel('Time [sec]')
%         xlim([2.6,3.4])
%         grid on
% 
%         subplot(2,2,2);
%         plot(t_ar_NS,x_err_mean_NS(2,:,2),'m',t_ar_S,x_err_mean_S(2,:,2),'k',t_ar_Sim_KF,x_err_mean_Sim_KF(2,:,2,p,n),'b')
%         title("\lambda error mean")
%         legend('nonsharing','sharing','sharing - KF simulation')
%         ylabel('\lambda error mean [deg]')
%         xlabel('Time [sec]')
%         xlim([2.6,3.4])
%         grid on
% 
%         subplot(2,2,3);
%         plot(t_ar_NS,x_err_SD_NS(1,:,2),'m',t_ar_S,x_err_SD_S(1,:,2),'k',t_ar_Sim_KF,x_err_SD_Sim_KF(1,:,2,p,n),'b')
%         title("\rho error SD")
%         ylabel('\rho error SD [m]')
%         xlabel('Time [sec]')
%         xlim([2.6,3.4])
%         grid on
% 
%         subplot(2,2,4);
%         plot(t_ar_NS,x_err_SD_NS(2,:,2),'m',t_ar_S,x_err_SD_S(2,:,2),'k',t_ar_Sim_KF,x_err_SD_Sim_KF(2,:,2,p,n),'b')
%         title("\lambda error SD")
%         ylabel('\lambda error SD [deg]')
%         xlabel('Time [sec]')
%         xlim([2.6,3.4])
%         grid on
% 
%         figure(n*1e3+p*1e2+21)
%         subplot(2,2,1);
%         plot(t_ar_NS,x_err_mean_NS(3,:,2),'m',t_ar_S,x_err_mean_S(3,:,2),'k',t_ar_Sim_KF,x_err_mean_Sim_KF(3,:,2,p,n),'b')
%         title("\gamma_t error mean")
%         legend('nonsharing','sharing','sharing - KF simulation')
%         ylabel('\gamma_t error mean [deg]')
%         xlabel('Time [sec]')
%         xlim([2.6,3.4])
%         grid on
% 
%         subplot(2,2,2);
%         plot(t_ar_NS,x_err_mean_NS(4,:,2),'m',t_ar_S,x_err_mean_S(4,:,2),'k',t_ar_Sim_KF,x_err_mean_Sim_KF(4,:,2,p,n),'b')
%         title("at error mean")
%         legend('nonsharing','sharing','sharing - KF simulation')
%         ylabel('at error mean [m/sec^2]')
%         xlabel('Time [sec]')
%         xlim([2.6,3.4])
%         grid on
% 
%         subplot(2,2,3);
%         plot(t_ar_NS,x_err_SD_NS(3,:,2),'m',t_ar_S,x_err_SD_S(3,:,2),'k',t_ar_Sim_KF,x_err_SD_Sim_KF(3,:,2,p,n),'b')
%         title("\gamma_t error SD")
%         ylabel('\gamma_t error SD [deg]')
%         xlabel('Time [sec]')
%         xlim([2.6,3.4])
%         grid on
% 
%         subplot(2,2,4);
%         plot(t_ar_NS,x_err_SD_NS(4,:,2),'m',t_ar_S,x_err_SD_S(4,:,2),'k',t_ar_Sim_KF,x_err_SD_Sim_KF(4,:,2,p,n),'b')
%         title("at error SD")
%         ylabel('at error SD [m/sec^2]')
%         xlabel('Time [sec]')
%         xlim([2.6,3.4])
%         grid on
%     end
%     end
% end
% 
% for p=1:Q_n_len
%     for n=1:TPM_len   
%         for m=1:TPM_sim_len
%             
%             figure(m*1e4+n*1e3+p*1e2+20)
%             subplot(2,2,1);
%             plot(t_ar_NS,x_err_mean_NS(1,:,2),'m',t_ar_S,x_err_mean_S(1,:,2),'k',t_ar_Sim_IMM,x_err_mean_Sim_IMM(1,:,2,p,n,m),'b')
%             title("\rho error mean")
%             legend('nonsharing','sharing','sharing - IMM simulation')
%             ylabel('\rho error mean [m]')
%             xlabel('Time [sec]')
%             xlim([2.6,3.4])
%             grid on
% 
%             subplot(2,2,2);
%             plot(t_ar_NS,x_err_mean_NS(2,:,2),'m',t_ar_S,x_err_mean_S(2,:,2),'k',t_ar_Sim_IMM,x_err_mean_Sim_IMM(2,:,2,p,n,m),'b')
%             title("\lambda error mean")
%             legend('nonsharing','sharing','sharing - IMM simulation')
%             ylabel('\lambda error mean [deg]')
%             xlabel('Time [sec]')
%             xlim([2.6,3.4])
%             grid on
% 
%             subplot(2,2,3);
%             plot(t_ar_NS,x_err_SD_NS(1,:,2),'m',t_ar_S,x_err_SD_S(1,:,2),'k',t_ar_Sim_IMM,x_err_SD_Sim_IMM(1,:,2,p,n,m),'b')
%             title("\rho error SD")
%             ylabel('\rho error SD [m]')
%             xlabel('Time [sec]')
%             xlim([2.6,3.4])
%             grid on
% 
%             subplot(2,2,4);
%             plot(t_ar_NS,x_err_SD_NS(2,:,2),'m',t_ar_S,x_err_SD_S(2,:,2),'k',t_ar_Sim_IMM,x_err_SD_Sim_IMM(2,:,2,p,n,m),'b')
%             title("\lambda error SD")
%             ylabel('\lambda error SD [deg]')
%             xlabel('Time [sec]')
%             xlim([2.6,3.4])
%             grid on
% 
%             figure(m*1e4+n*1e3+p*1e2+21)
%             subplot(2,2,1);
%             plot(t_ar_NS,x_err_mean_NS(3,:,2),'m',t_ar_S,x_err_mean_S(3,:,2),'k',t_ar_Sim_IMM,x_err_mean_Sim_IMM(3,:,2,p,n,m),'b')
%             title("\gamma_t error mean")
%             legend('nonsharing','sharing','sharing - IMM simulation')
%             ylabel('\gamma_t error mean [deg]')
%             xlabel('Time [sec]')
%             xlim([2.6,3.4])
%             grid on
% 
%             subplot(2,2,2);
%             plot(t_ar_NS,x_err_mean_NS(4,:,2),'m',t_ar_S,x_err_mean_S(4,:,2),'k',t_ar_Sim_IMM,x_err_mean_Sim_IMM(4,:,2,p,n,m),'b')
%             title("at error mean")
%             legend('nonsharing','sharing','sharing - IMM simulation')
%             ylabel('at error mean [m/sec^2]')
%             xlabel('Time [sec]')
%             xlim([2.6,3.4])
%             grid on
% 
%             subplot(2,2,3);
%             plot(t_ar_NS,x_err_SD_NS(3,:,2),'m',t_ar_S,x_err_SD_S(3,:,2),'k',t_ar_Sim_IMM,x_err_SD_Sim_IMM(3,:,2,p,n,m),'b')
%             title("\gamma_t error SD")
%             ylabel('\gamma_t error SD [deg]')
%             xlabel('Time [sec]')
%             xlim([2.6,3.4])
%             grid on
% 
%             subplot(2,2,4);
%             plot(t_ar_NS,x_err_SD_NS(4,:,2),'m',t_ar_S,x_err_SD_S(4,:,2),'k',t_ar_Sim_IMM,x_err_SD_Sim_IMM(4,:,2,p,n,m),'b')
%             title("at error SD")
%             ylabel('at error SD [m/sec^2]')
%             xlabel('Time [sec]')
%             xlim([2.6,3.4])
%             grid on
%             
%         end
%     end
% end
% 
% save("resualt_estimation_new")
% 
% toc

%% New methods CDF creator - same idea as before, but comparing with different scenarios
% tic
% 
% M=200;
% T_switch=[0.1:0.05:3.4];
% 
% Q_n_red_ar=[1,1e1,1e2,1e3];
% TPM_new_ar(:,:,1)=[0.99,0.01;0.01,0.99];
% TPM_new_ar(:,:,2)=[1-1e-3,1e-3;1e-3,1-1e-3];
% TPM_new_ar(:,:,3)=[1-1e-4,1e-4;1e-4,1-1e-4];
% TPM_new_sim_ar=TPM_new_ar;
% 
% Q_n_len=length(Q_n_red_ar);
% TPM_len=length(TPM_new_ar(1,1,:));
% TPM_sim_len=length(TPM_new_sim_ar(1,1,:));
% 
% NS_miss=zeros(length(T_switch),3,M);
% S_miss=zeros(length(T_switch),3,M);
% SM_miss=zeros(length(T_switch),3,M,Q_n_len);
% Sim_KF_miss=zeros(length(T_switch),3,M,Q_n_len,TPM_len);
% Sim_IMM_miss=zeros(length(T_switch),3,M,Q_n_len,TPM_len,TPM_sim_len);
% 
% for i=1:M
%     i
%     rng(i)
%     noise_int=randn(1,4).*sigmas;
%     noise_mes=randn(400,2)*delta_m_sig;
%     parfor j=1:length(T_switch)%-1
%         j
%         T_sw=T_switch(j); 
%
%         [t,xm,ym,xt,yt,nc,miu,miss]=NonSharing(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n);
%         NS_miss(j,:,i)=[miss(1),miss(2),min(miss)];
%         
%         [t,xm,ym,xt,yt,nc,miu,miss]=Sharing(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n);
%         S_miss(j,:,i)=[miss(1),miss(2),min(miss)];
%         
%         for p=1:Q_n_len            
%             Q_n_red=Q_n_red_ar(p);
%             
%             [t,xm,ym,xt,yt,nc,miu,miss]=Sharing_SingleMode(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n,DB,h,S_miss1_mean,S_miss1_SD,Q_n_red);
%             SM_miss(j,:,i,p)=[miss(1),miss(2),min(miss)];
%             
%             for n=1:TPM_len
%                 TPM_new=TPM_new_ar(:,:,n);
%                 
%                 [t,xm,ym,xt,yt,nc,miu,miss]=Sharing_KF_Sim(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n,DB,h,S_miss1_mean,S_miss1_SD,Q_n_red,TPM_new);
%                 Sim_KF_miss(j,:,i,p,n)=[miss(1),miss(2),min(miss)];
%                 
%                 for m=1:TPM_sim_len
%                     TPM_new_sim=TPM_new_sim_ar(:,:,m);
%         
%                     [t,xm,ym,xt,yt,nc,miu,miss]=Sharing_IMM_Sim(xm0,ym0,tau,am_max,vm,k,xt0,yt0,theta,at_max,T_sw,vt,gamma_t0,xr,yr,dt,dt_samp,Nmis,sigmas,delta_m_sig,noise_int,noise_mes,pi_mat,Q_n,DB,h,S_miss1_mean,S_miss1_SD,Q_n_red,TPM_new,TPM_new_sim);
%                     Sim_IMM_miss(j,:,i,p,n,m)=[miss(1),miss(2),min(miss)];
%                 end
%             end
%         end
%     end       
%     NS_miss(length(T_switch),:,i)=NS_miss(length(T_switch)-1,:,i);
%     S_miss(length(T_switch),:,i)=S_miss(length(T_switch)-1,:,i); 
%     SM_miss(length(T_switch),:,i)=SM_miss(length(T_switch)-1,:,i,:);
%     Sim_KF_miss(j,:,i,:,:)=Sim_KF_miss(length(T_switch)-1,:,i,:,:);
%     Sim_IMM_miss(length(T_switch),:,i,:,:,:)=Sim_IMM_miss(length(T_switch)-1,:,i,:,:,:);
% end
% 
% for j=1:length(T_switch)
%     S_miss_s(j,:)=sort(S_miss(j,3,:));
%     S_miss2_s(j,:)=sort(S_miss(j,2,:));
%     S_miss1_s(j,:)=sort(S_miss(j,1,:));
%     NS_miss_s(j,:)=sort(NS_miss(j,3,:));
%     NS_miss2_s(j,:)=sort(NS_miss(j,2,:));
%     Single_miss_s(j,:)=sort(NS_miss(j,1,:));
%         
%     S_miss_mean(j)=sum(S_miss_s(j,:))/M;
%     S_miss1_mean(j)=sum(S_miss1_s(j,:))/M;
%     S_miss2_mean(j)=sum(S_miss2_s(j,:))/M;
%     NS_miss_mean(j)=sum(NS_miss_s(j,:))/M;
%     NS_miss2_mean(j)=sum(NS_miss2_s(j,:))/M;
%     Single_miss_mean(j)=sum(Single_miss_s(j,:))/M;
%
%     S_miss_SD(j)=0;
%     S_miss1_SD(j)=0;
%     S_miss2_SD(j)=0;
%     NS_miss_SD(j)=0;
%     NS_miss2_SD(j)=0;
%     Single_miss_SD(j)=0;
%     for i=1:M
%         S_miss_SD(j)=S_miss_SD(j)+(S_miss_mean(j)-S_miss_s(j,i))^2;
%         S_miss1_SD(j)=S_miss1_SD(j)+(S_miss1_mean(j)-S_miss1_s(j,i))^2;
%         S_miss2_SD(j)=S_miss2_SD(j)+(S_miss2_mean(j)-S_miss2_s(j,i))^2;
%         NS_miss_SD(j)=NS_miss_SD(j)+(NS_miss_mean(j)-NS_miss_s(j,i))^2;
%         NS_miss2_SD(j)=NS_miss2_SD(j)+(NS_miss2_mean(j)-NS_miss2_s(j,i))^2;
%         Single_miss_SD(j)=Single_miss_SD(j)+(Single_miss_mean(j)-Single_miss_s(j,i))^2;
%     end
%     S_miss_SD(j)=sqrt(S_miss_SD(j)/(M-1));
%     S_miss1_SD(j)=sqrt(S_miss1_SD(j)/(M-1));
%     S_miss2_SD(j)=sqrt(S_miss2_SD(j)/(M-1));
%     NS_miss_SD(j)=sqrt(NS_miss_SD(j)/(M-1)); 
%     NS_miss2_SD(j)=sqrt(NS_miss2_SD(j)/(M-1)); 
%     Single_miss_SD(j)=sqrt(Single_miss_SD(j)/(M-1));  
%
%     for p=1:Q_n_len
%         SM_miss_s(j,:,p)=sort(SM_miss(j,3,:,p));
%         SM_miss2_s(j,:,p)=sort(SM_miss(j,2,:,p));
%         SM_miss1_s(j,:,p)=sort(SM_miss(j,1,:,p));
%         SM_miss_mean(j,p)=sum(SM_miss_s(j,:,p))/M;
%         SM_miss2_mean(j,p)=sum(SM_miss2_s(j,:,p))/M;
%         SM_miss1_mean(j,p)=sum(SM_miss1_s(j,:,p))/M;    
%         SM_miss_SD(j,p)=0;
%         SM_miss2_SD(j,p)=0;
%         SM_miss1_SD(j,p)=0;
%         for n=1:TPM_len
%             Sim_KF_miss_s(j,:,p,n)=sort(Sim_KF_miss(j,3,:,p,n));
%             Sim_KF_miss2_s(j,:,p,n)=sort(Sim_KF_miss(j,2,:,p,n));
%             Sim_KF_miss1_s(j,:,p,n)=sort(Sim_KF_miss(j,1,:,p,n));  
%             Sim_KF_miss_mean(j,p,n)=sum(Sim_KF_miss_s(j,:,p,n))/M;
%             Sim_KF_miss1_mean(j,p,n)=sum(Sim_KF_miss1_s(j,:,p,n))/M;
%             Sim_KF_miss2_mean(j,p,n)=sum(Sim_KF_miss2_s(j,:,p,n))/M;
%             Sim_KF_miss_SD(j,p,n)=0;
%             Sim_KF_miss1_SD(j,p,n)=0;
%             Sim_KF_miss2_SD(j,p,n)=0;
%             for m=1:TPM_sim_len
%                 Sim_IMM_miss_s(j,:,p,n,m)=sort(Sim_IMM_miss(j,3,:,p,n,m));
%                 Sim_IMM_miss2_s(j,:,p,n,m)=sort(Sim_IMM_miss(j,2,:,p,n,m));
%                 Sim_IMM_miss1_s(j,:,p,n,m)=sort(Sim_IMM_miss(j,1,:,p,n,m));
%                 Sim_IMM_miss_mean(j,p,n,m)=sum(Sim_IMM_miss_s(j,:,p,n,m))/M;
%                 Sim_IMM_miss1_mean(j,p,n,m)=sum(Sim_IMM_miss1_s(j,:,p,n,m))/M;
%                 Sim_IMM_miss2_mean(j,p,n,m)=sum(Sim_IMM_miss2_s(j,:,p,n,m))/M;
%                 Sim_IMM_miss_SD(j,p,n,m)=0;
%                 Sim_IMM_miss1_SD(j,p,n,m)=0;
%                 Sim_IMM_miss2_SD(j,p,n,m)=0;
%             end
%         end
%     end  
%     
%     for i=1:M
%         for p=1:Q_n_len
%             SM_miss_SD(j,p)=SM_miss_SD(j,p)+(SM_miss_mean(j,p)-SM_miss_s(j,i,p))^2;
%             SM_miss2_SD(j,p)=SM_miss2_SD(j,p)+(SM_miss2_mean(j,p)-SM_miss2_s(j,i,p))^2;
%             SM_miss1_SD(j,p)=SM_miss1_SD(j,p)+(SM_miss1_mean(j,p)-SM_miss1_s(j,i,p))^2;
%             for n=1:TPM_len
%                 Sim_KF_miss_SD(j,p,n)=Sim_KF_miss_SD(j,p,n)+(Sim_KF_miss_mean(j,p,n)-Sim_KF_miss_s(j,i,p,n))^2;
%                 Sim_KF_miss1_SD(j,p,n)=Sim_KF_miss1_SD(j,p,n)+(Sim_KF_miss1_mean(j,p,n)-Sim_KF_miss1_s(j,i,p,n))^2;
%                 Sim_KF_miss2_SD(j,p,n)=Sim_KF_miss2_SD(j,p,n)+(Sim_KF_miss2_mean(j,p,n)-Sim_KF_miss2_s(j,i,p,n))^2;
%                 for m=1:TPM_sim_len
%                     Sim_IMM_miss_SD(j,p,n,m)=Sim_IMM_miss_SD(j,p,n,m)+(Sim_IMM_miss_mean(j,p,n,m)-Sim_IMM_miss_s(j,i,p,n,m))^2;
%                     Sim_IMM_miss1_SD(j,p,n,m)=Sim_IMM_miss1_SD(j,p,n,m)+(Sim_IMM_miss1_mean(j,p,n,m)-Sim_IMM_miss1_s(j,i,p,n,m))^2;
%                     Sim_IMM_miss2_SD(j,p,n,m)=Sim_IMM_miss2_SD(j,p,n,m)+(Sim_IMM_miss2_mean(j,p,n,m)-Sim_IMM_miss2_s(j,i,p,n,m))^2;
%                 end
%             end
%         end        
%     end  
%     for p=1:Q_n_len
%         SM_miss_SD(j,p)=sqrt(SM_miss_SD(j,p)/(M-1)); 
%         SM_miss2_SD(j,p)=sqrt(SM_miss2_SD(j,p)/(M-1)); 
%         SM_miss1_SD(j,p)=sqrt(SM_miss1_SD(j,p)/(M-1)); 
%         for n=1:TPM_len
%             Sim_KF_miss_SD(j,p,n)=sqrt(Sim_KF_miss_SD(j,p,n)/(M-1));
%             Sim_KF_miss1_SD(j,p,n)=sqrt(Sim_KF_miss1_SD(j,p,n)/(M-1));
%             Sim_KF_miss2_SD(j,p,n)=sqrt(Sim_KF_miss2_SD(j,p,n)/(M-1));
%             for m=1:TPM_sim_len
%                 Sim_IMM_miss_SD(j,p,n,m)=sqrt(Sim_IMM_miss_SD(j,p,n,m)/(M-1));
%                 Sim_IMM_miss1_SD(j,p,n,m)=sqrt(Sim_IMM_miss1_SD(j,p,n,m)/(M-1));
%                 Sim_IMM_miss2_SD(j,p,n,m)=sqrt(Sim_IMM_miss2_SD(j,p,n,m)/(M-1));
%             end
%         end
%     end
% end
% 
% for i=1:M     
%     Single_miss_s_comb(i)=sum(Single_miss_s(:,i))/length(T_switch);
%     NS_miss_s_comb(i)=sum(NS_miss_s(:,i))/length(T_switch);
%     S_miss_s_comb(i)=sum(S_miss_s(:,i))/length(T_switch);
%     for p=1:Q_n_len
%         SM_miss_s_comb(i,p)=sum(SM_miss_s(:,i,p))/length(T_switch);
%         for n=1:TPM_len
%             Sim_KF_miss_s_comb(i,p,n)=sum(Sim_KF_miss_s(:,i,p,n))/length(T_switch);
%             for m=1:TPM_sim_len
%                 Sim_IMM_miss_s_comb(i,p,n,m)=sum(Sim_IMM_miss_s(:,i,p,n,m))/length(T_switch);
%             end
%         end
%     end
% end
% 
% cdf_array=[1:M]/M;
% 
% for p=1:Q_n_len
%     
%     figure(p*1e2+25)
%     subplot(2,1,1);
%     plot(T_switch,Single_miss_mean,'m',T_switch,S_miss1_mean,'k',T_switch,SM_miss1_mean(:,p),'b')
%     title("Miss Distance Mean Of Missile 1")
%     legend('nonsharing','sharing','single mode')
%     ylabel('mean miss distance [m]')
%     xlabel('Switch Time [sec]')
%     grid on
% 
%     subplot(2,1,2);
%     plot(T_switch,Single_miss_SD,'m',T_switch,S_miss1_SD,'k',T_switch,SM_miss1_SD(:,p),'b')
%     title("Miss Distance SD Of Missile 1")
%     legend('nonsharing','sharing','single mode')
%     ylabel('SD miss distance [m]')
%     xlabel('Switch Time [sec]')
%     grid on
% 
%     figure(p*1e2+26)
%     subplot(2,1,1);
%     plot(T_switch,NS_miss2_mean,'m',T_switch,S_miss2_mean,'k',T_switch,SM_miss2_mean(:,p),'b')
%     title("Miss Distance Mean Of Missile 2")
%     legend('nonsharing','sharing','single mode')
%     ylabel('mean miss distance [m]')
%     xlabel('Switch Time [sec]')
%     grid on
% 
%     subplot(2,1,2);
%     plot(T_switch,NS_miss2_SD,'m',T_switch,S_miss2_SD,'k',T_switch,SM_miss2_SD(:,p),'b')
%     title("Miss Distance SD Of Missile 2")
%     legend('nonsharing','sharing','single mode')
%     ylabel('SD miss distance [m]')
%     xlabel('Switch Time [sec]')
%     grid on
% 
%     figure(p*1e2+27)
%     subplot(2,1,1);
%     plot(T_switch,NS_miss_mean,'m',T_switch,S_miss_mean,'k',T_switch,SM_miss_mean(:,p),'b')
%     title("Team Miss Distance Mean")
%     legend('nonsharing','sharing','single mode')
%     ylabel('mean miss distance [m]')
%     xlabel('Switch Time [sec]')
%     grid on
% 
%     subplot(2,1,2);
%     plot(T_switch,NS_miss_SD,'m',T_switch,S_miss_SD,'k',T_switch,SM_miss_SD(:,p),'b')
%     title("Team Miss Distance SD")
%     legend('nonsharing','sharing','single mode')
%     ylabel('SD miss distance [m]')
%     xlabel('Switch Time [sec]')
%     grid on
% 
%     figure(p*1e2+28)
%     plot(SM_miss_s_comb(:,p),cdf_array,'b',S_miss_s_comb,cdf_array,'k',NS_miss_s_comb,cdf_array,'m',Single_miss_s_comb,cdf_array,'g')
%     xlabel('Miss Distance (m)')
%     ylabel('Miss CDF')
%     title('Compare - Miss Distance CDF')
%     legend('DGL1, sharing - single mode','DGL1, sharing','DGL1, nonsharing','DGL1, single missile')
%     grid on
%     
%     for n=1:TPM_len
%         
%         figure(n*1e4+p*1e2+25)
%         subplot(2,1,1);
%         plot(T_switch,Single_miss_mean,'m',T_switch,S_miss1_mean,'k',T_switch,Sim_KF_miss1_mean(:,p,n),'b')
%         title(["Miss Distance Mean Of Missile 1, p=",num2str(p),", n=",num2str(n)])
%         legend('nonsharing','sharing','KF simulation')
%         ylabel('mean miss distance [m]')
%         xlabel('Switch Time [sec]')
%         grid on
% 
%         subplot(2,1,2);
%         plot(T_switch,Single_miss_SD,'m',T_switch,S_miss1_SD,'k',T_switch,Sim_KF_miss1_SD(:,p,n),'b')
%         title(["Miss Distance SD Of Missile 1, p=",num2str(p),", n=",num2str(n)])
%         legend('nonsharing','sharing','KF simulation')
%         ylabel('SD miss distance [m]')
%         xlabel('Switch Time [sec]')
%         grid on
% 
%         figure(n*1e4+p*1e2+26)
%         subplot(2,1,1);
%         plot(T_switch,NS_miss2_mean,'m',T_switch,S_miss2_mean,'k',T_switch,Sim_KF_miss2_mean(:,p,n),'b')
%         title(["Miss Distance Mean Of Missile 2, p=",num2str(p),", n=",num2str(n)])
%         legend('nonsharing','sharing','KF simulation')
%         ylabel('mean miss distance [m]')
%         xlabel('Switch Time [sec]')
%         grid on
% 
%         subplot(2,1,2);
%         plot(T_switch,NS_miss2_SD,'m',T_switch,S_miss2_SD,'k',T_switch,Sim_KF_miss2_SD(:,p,n),'b')
%         title(["Miss Distance SD Of Missile 2, p=",num2str(p),", n=",num2str(n)])
%         legend('nonsharing','sharing','KF simulation')
%         ylabel('SD miss distance [m]')
%         xlabel('Switch Time [sec]')
%         grid on
% 
%         figure(n*1e4+p*1e2+27)
%         subplot(2,1,1);
%         plot(T_switch,NS_miss_mean,'m',T_switch,S_miss_mean,'k',T_switch,Sim_KF_miss_mean(:,p,n),'b')
%         title(["Team Miss Distance Mean, p=",num2str(p),", n=",num2str(n)])
%         legend('nonsharing','sharing','KF simulation')
%         ylabel('mean miss distance [m]')
%         xlabel('Switch Time [sec]')
%         grid on
% 
%         subplot(2,1,2);
%         plot(T_switch,NS_miss_SD,'m',T_switch,S_miss_SD,'k',T_switch,Sim_KF_miss_SD(:,p,n),'b')
%         title(["Team Miss Distance SD, p=",num2str(p),", n=",num2str(n)])
%         legend('nonsharing','sharing','KF simulation')
%         ylabel('SD miss distance [m]')
%         xlabel('Switch Time [sec]')
%         grid on
% 
%         figure(n*1e4+p*1e2+28)
%         plot(Sim_KF_miss_s_comb(:,p,n),cdf_array,'b',S_miss_s_comb,cdf_array,'k',NS_miss_s_comb,cdf_array,'m',Single_miss_s_comb,cdf_array,'g')
%         xlabel('Miss Distance (m)')
%         ylabel('Miss CDF')
%         title(["Compare - Miss Distance CDF, p=",num2str(p),", n=",num2str(n)])
%         legend('DGL1, sharing - KF simulation','DGL1, sharing','DGL1, nonsharing','DGL1, single missile')
%         grid on
%         
%         for m=1:TPM_sim_len
%             
%             figure(m*1e6+n*1e4+p*1e2+25)
%             subplot(2,1,1);
%             plot(T_switch,Single_miss_mean,'m',T_switch,S_miss1_mean,'k',T_switch,Sim_IMM_miss1_mean(:,p,n,m),'b')
%             title(["Miss Distance Mean Of Missile 1, p=",num2str(p),", n=",num2str(n)," m=",num2str(m)])
%             legend('nonsharing','sharing','IMM simulation')
%             ylabel('mean miss distance [m]')
%             xlabel('Switch Time [sec]')
%             grid on
% 
%             subplot(2,1,2);
%             plot(T_switch,Single_miss_SD,'m',T_switch,S_miss1_SD,'k',T_switch,Sim_IMM_miss1_SD(:,p,n,m),'b')
%             title(["Miss Distance SD Of Missile 1, p=",num2str(p),", n=",num2str(n)," m=",num2str(m)])
%             legend('nonsharing','sharing','IMM simulation')
%             ylabel('SD miss distance [m]')
%             xlabel('Switch Time [sec]')
%             grid on
% 
%             figure(m*1e6+n*1e4+p*1e2+26)
%             subplot(2,1,1);
%             plot(T_switch,NS_miss2_mean,'m',T_switch,S_miss2_mean,'k',T_switch,Sim_IMM_miss2_mean(:,p,n,m),'b')
%             title(["Miss Distance Mean Of Missile 2, p=",num2str(p),", n=",num2str(n)," m=",num2str(m)])
%             legend('nonsharing','sharing','IMM simulation')
%             ylabel('mean miss distance [m]')
%             xlabel('Switch Time [sec]')
%             grid on
% 
%             subplot(2,1,2);
%             plot(T_switch,NS_miss2_SD,'m',T_switch,S_miss2_SD,'k',T_switch,Sim_IMM_miss2_SD(:,p,n,m),'b')
%             title(["Miss Distance SD Of Missile 2, p=",num2str(p),", n=",num2str(n)," m=",num2str(m)])
%             legend('nonsharing','sharing','IMM simulation')
%             ylabel('SD miss distance [m]')
%             xlabel('Switch Time [sec]')
%             grid on
% 
%             figure(m*1e6+n*1e4+p*1e2+27)
%             subplot(2,1,1);
%             plot(T_switch,NS_miss_mean,'m',T_switch,S_miss_mean,'k',T_switch,Sim_IMM_miss_mean(:,p,n,m),'b')
%             title(["Team Miss Distance Mean, p=",num2str(p),", n=",num2str(n)," m=",num2str(m)])
%             legend('nonsharing','sharing','IMM simulation')
%             ylabel('mean miss distance [m]')
%             xlabel('Switch Time [sec]')
%             grid on
% 
%             subplot(2,1,2);
%             plot(T_switch,NS_miss_SD,'m',T_switch,S_miss_SD,'k',T_switch,Sim_IMM_miss_SD(:,p,n,m),'b')
%             title(["Team Miss Distance SD, p=",num2str(p),", n=",num2str(n)," m=",num2str(m)])
%             legend('nonsharing','sharing','IMM simulation')
%             ylabel('SD miss distance [m]')
%             xlabel('Switch Time [sec]')
%             grid on
% 
%             figure(m*1e6+n*1e4+p*1e2+28)
%             plot(Sim_IMM_miss_s_comb(:,p,n,m),cdf_array,'b',S_miss_s_comb,cdf_array,'k',NS_miss_s_comb,cdf_array,'m',Single_miss_s_comb,cdf_array,'g')
%             xlabel('Miss Distance (m)')
%             ylabel('Miss CDF')
%             title(["Compare - Miss Distance CDF, p=",num2str(p),", n=",num2str(n)," m=",num2str(m)])
%             legend('DGL1, sharing - IMM simulation','DGL1, sharing','DGL1, nonsharing','DGL1, single missile')
%             grid on
%             
%         end
%     end
% end
%
% save("resualt_CDF_new")
% 
% toc
