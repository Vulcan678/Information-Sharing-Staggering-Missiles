clc
clear all
close all

% problem constant parameters
g=9.81; % m/sec^2
N=4.6; % PN nad APN design parameter
dt=1e-3; % sec
nt_amp=20*g; % m/sec^2
vm0=2500; % m/sec
vt0=2500; % m/sec
HE=0*pi/180; % rad (initial heading error)
beta0=-90*pi/180;
maxnc=45*g; % m/sec^2 (maximal interceptor acceleration)
maxnt=20*g; % m/sec^2 (maximal target acceleration)
tau=0.2; % sec (interceptor's control time lag)
theta=0.2; % sec (target's true control time lag)
theta_mes=0.4; % sec (target's measured control time lag)
gem1=2.25; % ideal LQDG design parameter
gem2=1.5; % semi-ideal LQDG design parameter
b2=10^7; % semi-ideal LQDG design parameter
gem3=1.45; % non-ideal LQDG design parameter
b3=10^6; % non-ideal LQDG design parameter
manuver=3; % 1=sin,2=cos,3=const,4=bang-bang
T=2.5; % sec (periodic maneuver frequency time)
w=2*pi/T; % rad/sec (periodic maneuver frequency)
phase=0; % rad (periodic manuver initial phase)
k_DGL0=0.96; % DGL0 design parameter
k_DGL1=0.92; % DGL1 design parameter
sigmas=[50,pi/180,3*pi/180,10]; % initial SD of state variables: rho (m), lambda (rad), gamma_t (rad), a_t (m/sec^2)
lambda_sigma=(5e-4); % should be delta_sigma - LOS angle SD of measurment (rad)
at_sigma=10; % target accelerarion SD of measurment (m/sec^2)
w_sigma=2*g; % target acceleracion noise assumption (m/sec^2)
real=0; % use true state (1) or measurements (0)
flag=1; % limit interceptor acceleration to maxnc (1) don't limit (0)

% setting seed and calculating random noise to initial state and measurments
rng(162);

%% Balistic Missile Defence

xt0=0;
yt0=15000;
xm0=-50;
ym0=0;

%% test each method seperatly

[ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Ideal( N,1,manuver,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc);
%[ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Semi_Ideal( [maxnc,maxnt,k_DGL0],8,manuver,tau,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
%[ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal( [maxnc,maxnt,k_DGL1],9,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
%[ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true( N,3,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma,at_sigma );
%[ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_theta( N,3,manuver,tau,theta,theta_mes,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
%[ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true_theta( N,3,manuver,tau,theta,theta_mes,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma,at_sigma );
%[ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman([maxnc,maxnt,k_DGL1],9,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma,at_sigma,w_sigma );
%[ t,xm,ym,xt,yt,rtm,vc,nc,t_go,var,y_n_true,y_n_est,y_n_mes ] = Kalman_comp(N,3,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma,at_sigma,w_sigma,real );
%[ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman_theta( N,3,manuver,tau,theta,theta_mes,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma,at_sigma,w_sigma );

%% index list of parameters to pass to simulation function per each guidance law

% PN: N,1  
% APN: N,2
% OGL: N,3
% LQDG-ideal: gem1,4
% LQDG-semi-ideal: [gem2,b2],5
% LQDG-non-ideal: [gem3,b3],6
% DGL0-ideal: [maxnc,maxnt],7
% DGL0-semi/non-ideal: [maxnc,maxnt,k_DGL0],8
% DGL1: [maxnc,maxnt,k_DGL1],9

%% plot scenario analysis for single interceptor

% print miss distance
min(abs(rtm))

% print time of scenario
t(end-1)

% plot trajectoris of the missile and target
figure(1)
plot(xm,ym,xt,yt)
title('trajectoris')
legend('Missile','Target')
xlabel("x [m]")
ylabel("y [m]")

% plot relative distance
figure(2)
plot(t,rtm)
title('Relative Distance vs t')
xlabel("t [sec]")
ylabel("\rho [m]")

% plot closing velocity
figure(3)
plot(t,vc)
title('closing velocity vs t')
xlabel("t [sec]")
ylabel("closing velocity (m/sec)")

% plot missile acceleration command
figure(4)
plot(t,nc)
title('Pursuer Acceleration Command vs t')
xlabel("t [sec]")
ylabel("u [m/sec^2]")

% plot t_go propogation in time during the scenraio
figure(5)
plot(t,t_go)
title('t_go vs t')

% % only for Kalman_comp - comparing measurement vs estimation results
% figure(6)
% plot(t,y_n_mes(1,:)-y_n_true(1,:),t,y_n_est(1,:)-y_n_true(1,:),t,var(:,2).^(0.5),'k:',t,-var(:,2).^(0.5),'k:')
% title('\delta_m vs t')
% legend('mesured','estimated','bound')
% 
% figure(7)
% plot(t,y_n_mes(2,:)-y_n_true(2,:),t,y_n_est(2,:)-y_n_true(2,:),t,var(:,4).^(0.5),'k:',t,-var(:,4).^(0.5),'k:')
% title('Target Acceleration vs t')
% legend('mesured','estimated','bound')

 %% find N for PN law

% tic 
% N_arr=[3:0.2:5];
% 
% parfor i=1:length(N_arr)
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Ideal( N_arr(i),1,manuver,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc);
%     miss(i)=abs(rtm(end-1));
%     time(i)=t(end-1);
%     max_nc(i)=max(nc);
%     figure(1)
%     plot(xm,ym)
%     hold on
% end
% 
% figure(12)
% plot(N_arr,miss)
% title('Miss-Distance vs N')
% xlabel('N')
% ylabel('Miss-Distance [m]')
% 
% figure(13)
% plot(N_arr,time)
% title('End-Time vs N')
% xlabel('N')
% ylabel('End-Time [sec]')
% 
% figure(14)
% plot(N_arr,max_nc)
% title('Maximal-Acceleration vs N')
% xlabel('N')
% ylabel('Maximal-Acceleration [m^2/sec]')
% toc

%% choose gem for Ideal LQDG law

% tic
% gem_arr=[1.05:0.05:3];
% 
% parfor i=1:length(gem_arr)
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Ideal( gem_arr(i),4,manuver,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc);
%     miss(i)=abs(rtm(end-1));
%     max_nc(i)=max(nc);
% end
% 
% figure(21)
% plot(gem_arr,miss)
% title('Miss-Distance vs \gamma')
% xlabel('\gamma')
% ylabel('Miss-Distance [m]')
% 
% figure(22)
% plot(gem_arr,max_nc)
% title('Maximal-Acceleration vs \gamma')
% xlabel('\gamma')
% ylabel('Maximal-Acceleration [m^2/sec]')
% toc

%% choosse b & gem for Semi and Non Ideal LQDG laws

% tic
% gem_arr=[1.25:0.05:20];
% b_arr=[1e2,1e3,1e4,1e5,1e6,1e7,1e8];
% for k=1:length(b_arr)
%     b=b_arr(k);
%     parfor i=1:length(gem_arr)
%         [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Semi_Ideal( [gem_arr(i),b_arr(k)],5,manuver,tau,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
%         miss1(k,i)=abs(rtm(end-1));
%         max_nc1(k,i)=max(nc);
%         [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal( [gem_arr(i),b_arr(k)],6,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
%         miss2(k,i)=rtm(end-1);
%         max_nc2(k,i)=max(nc);
%     end
% end
% 
% figure(31)
% contourf(gem_arr,b_arr,miss1,20)
% hcb=colorbar
% xlabel(hcb,'Miss (m)')
% set(gca,'YScale','log')
% title('Miss-Distance vs \gamma & b')
% xlabel('\gamma')
% ylabel('b')
% 
% figure(32)
% contourf(gem_arr,b_arr,max_nc1,20)
% hcb=colorbar
% xlabel(hcb,'Acceleration (m/sec^2)')
% set(gca,'YScale','log')
% title('Maximal-Acceleration vs \gamma & b')
% xlabel('\gamma')
% ylabel('b')
% 
% figure(33)
% contourf(gem_arr,b_arr,miss2,30)
% hcb=colorbar
% xlabel(hcb,'Miss (m)')
% set(gca,'YScale','log')
% title('LQDG with non ideal Pursuer and Target - Miss-Distance vs \gamma & b')
% xlabel('\gamma')
% ylabel('b')
% 
% figure(34)
% contourf(gem_arr,b_arr,max_nc2,30)
% hcb=colorbar
% xlabel(hcb,'Acceleration (m/sec^2)')
% set(gca,'YScale','log')
% title('LQDG with non ideal Pursuer and Target - Maximal-Acceleration vs \gamma & b')
% xlabel('\gamma')
% ylabel('b')
% toc

%% choose k for DGL0 and DGL1 laws

% tic 
% k_arr=[0.1:0.02:1];
% 
% parfor i=1:length(k_arr)
%     k=k_arr(i);
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Semi_Ideal( [maxnc,maxnt,k],8,manuver,tau,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
%     miss11(i)=abs(rtm(end-1));
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal( [maxnc,maxnt,k],8,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
%     miss12(i)=abs(rtm(end-1));
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal( [maxnc,maxnt,k],9,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
%     miss13(i)=abs(rtm(end-1));
% end
% 
% figure(41)
% semilogx(k_arr,miss11)
% title('DGL0 - Miss-Distance vs k')
% xlabel('k')
% ylabel('Miss (m)')
% 
% figure(42)
% semilogx(k_arr,miss12)
% title('DGL0 with non ideal target- Miss-Distance vs k')
% xlabel('k')
% ylabel('Miss (m)')
% 
% figure(43)
% semilogx(k_arr,miss13)
% title('DGL1 - Miss vs k')
% xlabel('k')
% ylabel('Miss (m)')
% toc

%% compare methods - ideal

% [ t_PN,xm_PN,ym_PN,xt_PN,yt_PN,rtm_PN,vc_PN,nc_PN,t_go_PN ] = Ideal( N,1,manuver,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc);
% [ t_APN,xm_APN,ym_APN,xt_APN,yt_APN,rtm_APN,vc_APN,nc_APN,t_go_APN ] = Ideal( N,2,manuver,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc);
% [ t_LQDG,xm_LQDG,ym_LQDG,xt_LQDG,yt_LQDG,rtm_LQDG,vc_LQDG,nc_LQDG,t_go_LQDG ] = Ideal( gem1,4,manuver,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc);
% [ t_DGL0,xm_DGL0,ym_DGL0,xt_DGL0,yt_DGL0,rtm_DGL0,vc_DGL0,nc_DGL0,t_go_DGL0 ] = Ideal( [maxnc,maxnt],7,manuver,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc);
% 
% figure(51)
% plot(xm_PN,ym_PN,xm_APN,ym_APN,xm_LQDG,ym_LQDG,xm_DGL0,ym_DGL0,xt_PN,yt_PN)
% title('Trajectoris')
% legend('Missile (PN)','Missile (APN)','Missile (LQDG)','Missile (DGL0)','Target')
% xlabel('X [m]')
% ylabel('Y [m]')
% 
% figure(52)
% plot(t_PN,rtm_PN,t_APN,rtm_APN,t_LQDG,rtm_LQDG,t_DGL0,rtm_DGL0)
% title('\rho vs t')
% legend('PN','APN','LQDG','DGL0')
% xlabel('time [sec]')
% ylabel('\rho [m]')
% 
% figure(53)
% plot(t_PN,vc_PN,t_APN,vc_APN,t_LQDG,vc_LQDG,t_DGL0,vc_DGL0)
% title('Vc vs t')
% legend('PN','APN','LQDG','DGL0')
% xlabel('time [sec]')
% ylabel('Vc [m/sec]')
% 
% figure(54)
% plot(t_PN,nc_PN,t_APN,nc_APN,t_LQDG,nc_LQDG,t_DGL0,nc_DGL0)
% title('u vs t')
% legend('PN','APN','LQDG','DGL0')
% xlabel('time [sec]')
% ylabel('u [m/sec^2]')

%% compare methods - semi ideal

% [ t_PN,xm_PN,ym_PN,xt_PN,yt_PN,rtm_PN,vc_PN,nc_PN,t_go_PN ] = Semi_Ideal( N,1,manuver,tau,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
% [ t_APN,xm_APN,ym_APN,xt_APN,yt_APN,rtm_APN,vc_APN,nc_APN,t_go_APN ] = Semi_Ideal( N,2,manuver,tau,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
% [ t_OGL,xm_OGL,ym_OGL,xt_OGL,yt_OGL,rtm_OGL,vc_OGL,nc_OGL,t_go_OGL ] = Semi_Ideal( N,3,manuver,tau,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
% [ t_LQDG,xm_LQDG,ym_LQDG,xt_LQDG,yt_LQDG,rtm_LQDG,vc_LQDG,nc_LQDG,t_go_LQDG ] = Semi_Ideal( [gem2,b2],5,manuver,tau,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
% [ t_DGL0,xm_DGL0,ym_DGL0,xt_DGL0,yt_DGL0,rtm_DGL0,vc_DGL0,nc_DGL0,t_go_DGL0 ] = Semi_Ideal( [maxnc,maxnt,k_DGL0],8,manuver,tau,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
% 
% figure(61)
% plot(xm_PN,ym_PN,xm_APN,ym_APN,xm_OGL,ym_OGL,xm_LQDG,ym_LQDG,xm_DGL0,ym_DGL0,xt_PN,yt_PN)
% title('Trajectoris')
% legend('Missile (PN)','Missile (APN)','Missile (OGL)','Missile (LQDG)','Missile (DGL0)','Target')
% xlabel('X [m]')
% ylabel('Y [m]')
% 
% figure(62)
% plot(t_PN,rtm_PN,t_APN,rtm_APN,t_OGL,rtm_OGL,t_LQDG,rtm_LQDG,t_DGL0,rtm_DGL0)
% title('\rho vs t')
% legend('PN','APN','OGL','LQDG','DGL0')
% xlabel('time [sec]')
% ylabel('\rho [m]')
% 
% figure(63)
% plot(t_PN,vc_PN,t_APN,vc_APN,t_OGL,vc_OGL,t_LQDG,vc_LQDG,t_DGL0,vc_DGL0)
% title('Vc vs t')
% legend('PN','APN','OGL','LQDG','DGL0')
% xlabel('time [sec]')
% ylabel('Vc [m/sec]')
% 
% figure(64)
% plot(t_PN,nc_PN,t_APN,nc_APN,t_OGL,nc_OGL,t_LQDG,nc_LQDG,t_DGL0,nc_DGL0)
% title('u vs t')
% legend('PN','APN','OGL','LQDG','DGL0')
% xlabel('time [sec]')
% ylabel('u [m/sec^2]')

%% compare methods - non ideal

% [ t_PN,xm_PN,ym_PN,xt_PN,yt_PN,rtm_PN,vc_PN,nc_PN,t_go_PN ] = Non_Ideal( N,1,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
% [ t_APN,xm_APN,ym_APN,xt_APN,yt_APN,rtm_APN,vc_APN,nc_APN,t_go_APN ] = Non_Ideal( N,2,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
% [ t_OGL,xm_OGL,ym_OGL,xt_OGL,yt_OGL,rtm_OGL,vc_OGL,nc_OGL,t_go_OGL ] = Non_Ideal( N,3,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
% [ t_LQDG,xm_LQDG,ym_LQDG,xt_LQDG,yt_LQDG,rtm_LQDG,vc_LQDG,nc_LQDG,t_go_LQDG ] = Non_Ideal( [gem3,b3],6,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
% [ t_DGL0,xm_DGL0,ym_DGL0,xt_DGL0,yt_DGL0,rtm_DGL0,vc_DGL0,nc_DGL0,t_go_DGL0 ] = Non_Ideal( [maxnc,maxnt,k_DGL0],8,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
% [ t_DGL1,xm_DGL1,ym_DGL1,xt_DGL1,yt_DGL1,rtm_DGL1,vc_DGL1,nc_DGL1,t_go_DGL1 ] = Non_Ideal( [maxnc,maxnt,k_DGL1],9,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc );
% 
% figure(71)
% plot(xm_PN,ym_PN,xm_APN,ym_APN,xm_OGL,ym_OGL,xm_LQDG,ym_LQDG,xm_DGL0,ym_DGL0,xm_DGL1,ym_DGL1,xt_PN,yt_PN)
% title('Trajectoris')
% legend('Missile (PN)','Missile (APN)','Missile (OGL)','Missile (LQDG)','Missile (DGL0)','Missile (DGL1)','Target')
% xlabel('X [m]')
% ylabel('Y [m]')
% 
% figure(72)
% plot(t_PN,rtm_PN,t_APN,rtm_APN,t_OGL,rtm_OGL,t_LQDG,rtm_LQDG,t_DGL0,rtm_DGL0,t_DGL1,rtm_DGL1)
% title('\rho vs t')
% legend('PN','APN','OGL','LQDG','DGL0','DGL1')
% xlabel('time [sec]')
% ylabel('\rho [m]')
% 
% figure(73)
% plot(t_PN,vc_PN,t_APN,vc_APN,t_OGL,vc_OGL,t_LQDG,vc_LQDG,t_DGL0,vc_DGL0,t_DGL1,vc_DGL1)
% title('Vc vs t')
% legend('PN','APN','OGL','LQDG','DGL0','DGL1')
% xlabel('time [sec]')
% ylabel('Vc [m/sec]')
% 
% figure(74)
% plot(t_PN,nc_PN,t_APN,nc_APN,t_OGL,nc_OGL,t_LQDG,nc_LQDG,t_DGL0,nc_DGL0,t_DGL1,nc_DGL1)
% title('u vs t')
% legend('PN','APN','OGL','LQDG','DGL0','DGL1')
% xlabel('time [sec]')
% ylabel('u [m/sec^2]')

%% compare methods - non ideal sin

% manuver_new = 1;
% w_arr=[5e-2, 1e-1, 5e-1, 1:20];
% 
% for i=1:length(w_arr)
%     w_new=w_arr(i);
%     [ t_PN,xm_PN,ym_PN,xt_PN,yt_PN,rtm_PN,vc_PN,nc_PN,t_go_PN ] = Non_Ideal( N,1,manuver_new,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w_new,phase,flag,maxnc );
%     [ t_APN,xm_APN,ym_APN,xt_APN,yt_APN,rtm_APN,vc_APN,nc_APN,t_go_APN ] = Non_Ideal( N,2,manuver_new,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w_new,phase,flag,maxnc );
%     [ t_OGL,xm_OGL,ym_OGL,xt_OGL,yt_OGL,rtm_OGL,vc_OGL,nc_OGL,t_go_OGL ] = Non_Ideal( N,3,manuver_new,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w_new,phase,flag,maxnc );
%     [ t_LQDG,xm_LQDG,ym_LQDG,xt_LQDG,yt_LQDG,rtm_LQDG,vc_LQDG,nc_LQDG,t_go_LQDG ] = Non_Ideal( [gem3,b3],6,manuver_new,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w_new,phase,flag,maxnc );
%     [ t_DGL0,xm_DGL0,ym_DGL0,xt_DGL0,yt_DGL0,rtm_DGL0,vc_DGL0,nc_DGL0,t_go_DGL0 ] = Non_Ideal( [maxnc,maxnt,k_DGL0],8,manuver_new,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w_new,phase,flag,maxnc );
%     [ t_DGL1,xm_DGL1,ym_DGL1,xt_DGL1,yt_DGL1,rtm_DGL1,vc_DGL1,nc_DGL1,t_go_DGL1 ] = Non_Ideal( [maxnc,maxnt,k_DGL1],9,manuver_new,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w_new,phase,flag,maxnc );
% 
%   miss_PN(i)=min(abs(rtm_PN));
%   maxnc_PN(i)=max(nc_PN);
% 
%   miss_APN(i)=min(abs(rtm_APN));
%   maxnc_APN(i)=max(nc_APN);
%   
%   miss_OGL(i)=min(abs(rtm_OGL));
%   maxnc_OGL(i)=max(nc_OGL);
%   
%   miss_LQDG(i)=min(abs(rtm_LQDG));
%   maxnc_LQDG(i)=max(nc_LQDG);
% 
%   miss_DGL0(i)=min(abs(rtm_DGL0));
%   maxnc_DGL0(i)=max(nc_DGL0);
%   
%   miss_DGL1(i)=min(abs(rtm_DGL1));
%   maxnc_DGL1(i)=max(nc_DGL1);
% end
% 
% figure(81)
% semilogx(w_arr,miss_PN,w_arr,miss_APN,w_arr,miss_OGL,w_arr,miss_LQDG,w_arr,miss_DGL0,w_arr,miss_DGL1)
% title('Miss distance vs \omega')
% xlabel('\omega [rad/sec]')
% ylabel('Miss Distance [m]')
% legend('PN','APN','OGL','LQDG','DGL0','DGL1')
% 
% figure(82)
% semilogx(w_arr,maxnc_PN,w_arr,maxnc_APN,w_arr,maxnc_OGL,w_arr,maxnc_LQDG,w_arr,maxnc_DGL0,w_arr,maxnc_DGL1)
% title('Maximal Acceleration vs \omega')
% xlabel('\omega [rad/sec]')
% ylabel('Maximal Acceleration [m/sec^2]')
% legend('PN','APN','OGL','LQDG','DGL0','DGL1')

%% compare methods - non ideal sin - scenario example
 
% manuver_new = 1;
% w_new = 2;
% 
% [ t_PN,xm_PN,ym_PN,xt_PN,yt_PN,rtm_PN,vc_PN,nc_PN,t_go_PN ] = Non_Ideal( N,1,manuver_new,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w_new,phase,flag,maxnc );
% [ t_APN,xm_APN,ym_APN,xt_APN,yt_APN,rtm_APN,vc_APN,nc_APN,t_go_APN ] = Non_Ideal( N,2,manuver_new,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w_new,phase,flag,maxnc );
% [ t_OGL,xm_OGL,ym_OGL,xt_OGL,yt_OGL,rtm_OGL,vc_OGL,nc_OGL,t_go_OGL ] = Non_Ideal( N,3,manuver_new,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w_new,phase,flag,maxnc );
% [ t_LQDG,xm_LQDG,ym_LQDG,xt_LQDG,yt_LQDG,rtm_LQDG,vc_LQDG,nc_LQDG,t_go_LQDG ] = Non_Ideal( [gem3,b3],6,manuver_new,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w_new,phase,flag,maxnc );
% [ t_DGL0,xm_DGL0,ym_DGL0,xt_DGL0,yt_DGL0,rtm_DGL0,vc_DGL0,nc_DGL0,t_go_DGL0 ] = Non_Ideal( [maxnc,maxnt,k_DGL0],8,manuver_new,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w_new,phase,flag,maxnc );
% [ t_DGL1,xm_DGL1,ym_DGL1,xt_DGL1,yt_DGL1,rtm_DGL1,vc_DGL1,nc_DGL1,t_go_DGL1 ] = Non_Ideal( [maxnc,maxnt,k_DGL1],9,manuver_new,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w_new,phase,flag,maxnc );
% 
% figure(91)
% plot(xm_PN,ym_PN,xm_APN,ym_APN,xm_OGL,ym_OGL,xm_LQDG,ym_LQDG,xm_DGL0,ym_DGL0,xm_DGL1,ym_DGL1,xt_PN,yt_PN)
% title('Trajectoris')
% legend('Missile (PN)','Missile (APN)','Missile (OGL)','Missile (LQDG)','Missile (DGL0)','Missile (DGL1)','Target')
% xlabel('X [m]')
% ylabel('Y [m]')
% 
% figure(92)
% plot(t_PN,abs(rtm_PN),t_APN,abs(rtm_APN),t_OGL,abs(rtm_OGL),t_LQDG,abs(rtm_LQDG),t_DGL0,abs(rtm_DGL0),t_DGL1,abs(rtm_DGL1))
% title('\rho vs t')
% legend('PN','APN','OGL','LQDG','DGL0','DGL1')
% xlabel('time [sec]')
% ylabel('\rho [m]')
% 
% figure(93)
% plot(t_PN,vc_PN,t_APN,vc_APN,t_OGL,vc_OGL,t_LQDG,vc_LQDG,t_DGL0,vc_DGL0,t_DGL1,vc_DGL1)
% title('Vc vs t')
% legend('PN','APN','OGL','LQDG','DGL0','DGL1')
% xlabel('time [sec]')
% ylabel('Vc [m/sec]')
% 
% figure(94)
% plot(t_PN,nc_PN,t_APN,nc_APN,t_OGL,nc_OGL,t_LQDG,nc_LQDG,t_DGL0,nc_DGL0,t_DGL1,nc_DGL1)
% title('u vs t')
% legend('PN','APN','OGL','LQDG','DGL0','DGL1')
% xlabel('time [sec]')
% ylabel('u [m/sec^2]')

%% choose worst w_sigma for sin manuver

% tic
% P=200;
% at_sigma_new=50;
% lambda_sigma_new=5e-3;
% w_sigma_arr=[1e-2,0.1,1,10,100,1e3,1e4,1e5]*g;
% M=length(w_sigma_arr);
% parfor i=1:P
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true( N,1,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new);
%     miss_PN_true1(i)=rtm(end-1);
%     for k=1:M
%         [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman(N,1,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma_arr(k) );
%         miss_PN_kalman1(i,k)=rtm(end-1);
%     end
% end
% miss_PN_true1=sort(miss_PN_true1);
% miss_PN_kalman001=sort(miss_PN_kalman1(:,1));
% miss_PN_kalman01=sort(miss_PN_kalman1(:,2));
% miss_PN_kalman1e0=sort(miss_PN_kalman1(:,3));
% miss_PN_kalman1e1=sort(miss_PN_kalman1(:,4));
% miss_PN_kalman1e2=sort(miss_PN_kalman1(:,5));
% miss_PN_kalman1e3=sort(miss_PN_kalman1(:,6));
% miss_PN_kalman1e4=sort(miss_PN_kalman1(:,7));
% miss_PN_kalman1e5=sort(miss_PN_kalman1(:,8));
% 
% cdf_array=(1/P)*[1:P];
% 
% % large errors
% figure(101)
% semilogx(miss_PN_true1,cdf_array,'b--',miss_PN_kalman001,cdf_array,'r',miss_PN_kalman01,cdf_array,'r:',miss_PN_kalman1e0,cdf_array,'g',miss_PN_kalman1e1,cdf_array,'g-.',miss_PN_kalman1e2,cdf_array,'m',miss_PN_kalman1e3,cdf_array,'b:',miss_PN_kalman1e4,cdf_array,'b',miss_PN_kalman1e5,cdf_array,'m-.')
% xlabel('Miss Distance (m)')
% ylabel('Miss CDF')
% legend('without kalman filter','\sigma_w=0.01g','\sigma_w=0.1g','\sigma_w=1g','\sigma_w=10g','\sigma_w=100g','\sigma_w=1000g','\sigma_w=10000g','\sigma_w=100000g')
% title('Mean Miss Distance vs Target Command SD')
% toc

%% Kalman SD

% real_new=1;
% [ t,xm,ym,xt,yt,rtm,vc,nc,t_go,var,y_n_true,y_n_est,y_n_mes ] = Kalman_comp(N,1,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma,at_sigma,w_sigma,real_new );
% 
% figure(111)
% plot(t,y_n_mes(1,:)-y_n_true(1,:),t,y_n_est(1,:)-y_n_true(1,:),t,var(:,3).^(0.5),'k:',t,-var(:,3).^(0.5),'k:')
% title('\delta_m vs t - Open Loop')
% legend('Mesured','Estimated','SD')
% xlabel('time [sec]')
% ylabel('\delta_m [deg]')
% 
% figure(112)
% plot(t,y_n_mes(2,:)-y_n_true(2,:),t,y_n_est(2,:)-y_n_true(2,:),t,var(:,4).^(0.5),'k:',t,-var(:,4).^(0.5),'k:')
% title('Target Acceleration vs t - Open Loop')
% legend('Mesured','Estimated','SD')
% xlabel('time [sec]')
% ylabel('a_t [deg]')
% 
% real_new=0;
% [ t,xm,ym,xt,yt,rtm,vc,nc,t_go,var,y_n_true,y_n_est,y_n_mes ] = Kalman_comp(N,1,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma,at_sigma,w_sigma,real_new );
% 
% figure(113)
% plot(t,y_n_mes(1,:)-y_n_true(1,:),t,y_n_est(1,:)-y_n_true(1,:),t,var(:,3).^(0.5),'k:',t,-var(:,3).^(0.5),'k:')
% title('\delta_m vs t - Close Loop')
% legend('Mesured','Estimated','SD')
% xlabel('time [sec]')
% ylabel('\delta_m [deg]')
% 
% figure(114)
% plot(t,y_n_mes(2,:)-y_n_true(2,:),t,y_n_est(2,:)-y_n_true(2,:),t,var(:,4).^(0.5),'k:',t,-var(:,4).^(0.5),'k:')
% title('Target Acceleration vs t - Close Loop')
% legend('Mesured','Estimated','SD')
% xlabel('time [sec]')
% ylabel('a_t [deg]')

%% hits pracentege

% clear miss_PN_kalman1
% 
% tic
% M=500;
% count=0;
% parfor i=1:M   
%     i
%     %small errors
%     lambda_sigma_new=5e-4;
%     at_sigma_new=10;
%     theta_mes_plus=0.25;    
%     theta_mes_minus=0.15;
%     
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true( N,1,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_PN_true1(i)=abs(rtm(end-1));
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman(N,1,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_PN_kalman1(i)=abs(rtm(end-1));
%     
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true( N,2,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_APN_true1(i)=abs(rtm(end-1));
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman(N,2,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_APN_kalman1(i)=abs(rtm(end-1));
%     
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true( N,3,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_OGL_true1(i)=abs(rtm(end-1));
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman(N,3,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_OGL_kalman1(i)=abs(rtm(end-1));
%     
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true( [gem3,b3],6,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_LQDG_true1(i)=abs(rtm(end-1));
% 	  [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman([gem3,b3],6,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_LQDG_kalman1(i)=abs(rtm(end-1));
% 	  [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true_theta( [gem3,b3],6,manuver,tau,theta,theta_mes_plus,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_LQDG_plus_true1(i)=abs(rtm(end-1));
% 	  [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman_theta( [gem3,b3],6,manuver,tau,theta,theta_mes_plus,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_LQDG_plus_kalman1(i)=abs(rtm(end-1));
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true_theta( [gem3,b3],6,manuver,tau,theta,theta_mes_minus,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new);
%     miss_LQDG_minus_true1(i)=abs(rtm(end-1));
% 	  [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman_theta( [gem3,b3],6,manuver,tau,theta,theta_mes_minus,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_LQDG_minus_kalman1(i)=abs(rtm(end-1));
%     
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true( [maxnc,maxnt,k_DGL0],8,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_DGL0_true1(i)=abs(rtm(end-1));
% 	  [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman([maxnc,maxnt,k_DGL0],8,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_DGL0_kalman1(i)=abs(rtm(end-1));
%     
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true( [maxnc,maxnt,k_DGL1],9,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_DGL1_true1(i)=abs(rtm(end-1));
% 	  [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman([maxnc,maxnt,k_DGL1],9,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_DGL1_kalman1(i)=abs(rtm(end-1));
% 	  [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true_theta( [maxnc,maxnt,k_DGL1],9,manuver,tau,theta,theta_mes_plus,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_DGL1_plus_true1(i)=abs(rtm(end-1));
% 	  [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman_theta( [maxnc,maxnt,k_DGL1],9,manuver,tau,theta,theta_mes_plus,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_DGL1_plus_kalman1(i)=abs(rtm(end-1));
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true_theta( [maxnc,maxnt,k_DGL1],9,manuver,tau,theta,theta_mes_minus,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_DGL1_minus_true1(i)=abs(rtm(end-1));
% 	  [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman_theta( [maxnc,maxnt,k_DGL1],9,manuver,tau,theta,theta_mes_minus,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_DGL1_minus_kalman1(i)=abs(rtm(end-1));
%     
%     %large errors
%     lambda_sigma_new=5e-3;
%     at_sigma_new=50;
%     theta_mes_plus=0.3;    
%     theta_mes_minus=0.1;
%     
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true( N,1,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_PN_true2(i)=abs(rtm(end-1));
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman(N,1,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_PN_kalman2(i)=abs(rtm(end-1));
%     
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true( N,2,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_APN_true2(i)=abs(rtm(end-1));
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman(N,2,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_APN_kalman2(i)=abs(rtm(end-1));
%     
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true( N,3,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_OGL_true2(i)=abs(rtm(end-1));
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman(N,3,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_OGL_kalman2(i)=abs(rtm(end-1));
%     
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true( [gem3,b3],6,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_LQDG_true2(i)=abs(rtm(end-1));
% 	  [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman([gem3,b3],6,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_LQDG_kalman2(i)=abs(rtm(end-1));
% 	  [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true_theta( [gem3,b3],6,manuver,tau,theta,theta_mes_plus,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_LQDG_plus_true2(i)=abs(rtm(end-1));
% 	  [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman_theta( [gem3,b3],6,manuver,tau,theta,theta_mes_plus,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_LQDG_plus_kalman2(i)=abs(rtm(end-1));
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true_theta( [gem3,b3],6,manuver,tau,theta,theta_mes_minus,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_LQDG_minus_true2(i)=abs(rtm(end-1));
% 	  [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman_theta( [gem3,b3],6,manuver,tau,theta,theta_mes_minus,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_LQDG_minus_kalman2(i)=abs(rtm(end-1));
%     
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true( [maxnc,maxnt,k_DGL0],8,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_DGL0_true2(i)=abs(rtm(end-1));
% 	  [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman([maxnc,maxnt,k_DGL0],8,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_DGL0_kalman2(i)=abs(rtm(end-1));
%     
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true( [maxnc,maxnt,k_DGL1],9,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_DGL1_true2(i)=abs(rtm(end-1));
% 	  [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman([maxnc,maxnt,k_DGL1],9,manuver,tau,theta,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_DGL1_kalman2(i)=abs(rtm(end-1));
% 	  [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true_theta( [maxnc,maxnt,k_DGL1],9,manuver,tau,theta,theta_mes_plus,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_DGL1_plus_true2(i)=abs(rtm(end-1));
% 	  [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman_theta( [maxnc,maxnt,k_DGL1],9,manuver,tau,theta,theta_mes_plus,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_DGL1_plus_kalman2(i)=abs(rtm(end-1));
%     [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Non_Ideal_true_theta( [maxnc,maxnt,k_DGL1],9,manuver,tau,theta,theta_mes_minus,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new );
%     miss_DGL1_minus_true2(i)=abs(rtm(end-1));
% 	  [ t,xm,ym,xt,yt,rtm,vc,nc,t_go ] = Kalman_theta( [maxnc,maxnt,k_DGL1],9,manuver,tau,theta,theta_mes_minus,dt,xt0,yt0,xm0,ym0,vm0,vt0,HE,beta0,nt_amp,w,phase,flag,maxnc,sigmas,lambda_sigma_new,at_sigma_new,w_sigma );
%     miss_DGL1_minus_kalman2(i)=abs(rtm(end-1));
% end
% 
% miss_PN_true1=sort(miss_PN_true1);
% miss_PN_kalman1=sort(miss_PN_kalman1);
% miss_APN_true1=sort(miss_APN_true1);
% miss_APN_kalman1=sort(miss_APN_kalman1);
% miss_OGL_true1=sort(miss_OGL_true1);
% miss_OGL_kalman1=sort(miss_OGL_kalman1);
% miss_LQDG_true1=sort(miss_LQDG_true1);
% miss_LQDG_kalman1=sort(miss_LQDG_kalman1);
% miss_LQDG_plus_true1=sort(miss_LQDG_plus_true1);
% miss_LQDG_plus_kalman1=sort(miss_LQDG_plus_kalman1);
% miss_LQDG_minus_true1=sort(miss_LQDG_minus_true1);
% miss_LQDG_minus_kalman1=sort(miss_LQDG_minus_kalman1);
% miss_DGL0_true1=sort(miss_DGL0_true1);
% miss_DGL0_kalman1=sort(miss_DGL0_kalman1);
% miss_DGL1_true1=sort(miss_DGL1_true1);
% miss_DGL1_kalman1=sort(miss_DGL1_kalman1);
% miss_DGL1_plus_true1=sort(miss_DGL1_plus_true1);
% miss_DGL1_plus_kalman1=sort(miss_DGL1_plus_kalman1);
% miss_DGL1_minus_true1=sort(miss_DGL1_minus_true1);
% miss_DGL1_minus_kalman1=sort(miss_DGL1_minus_kalman1);
% miss_PN_true2=sort(miss_PN_true2);
% miss_PN_kalman2=sort(miss_PN_kalman2);
% miss_APN_true2=sort(miss_APN_true2);
% miss_APN_kalman2=sort(miss_APN_kalman2);
% miss_OGL_true2=sort(miss_OGL_true2);
% miss_OGL_kalman2=sort(miss_OGL_kalman2);
% miss_LQDG_true2=sort(miss_LQDG_true2);
% miss_LQDG_kalman2=sort(miss_LQDG_kalman2);
% miss_LQDG_plus_true2=sort(miss_LQDG_plus_true2);
% miss_LQDG_plus_kalman2=sort(miss_LQDG_plus_kalman2);
% miss_LQDG_minus_true2=sort(miss_LQDG_minus_true2);
% miss_LQDG_minus_kalman2=sort(miss_LQDG_minus_kalman2);
% miss_DGL0_true2=sort(miss_DGL0_true2);
% miss_DGL0_kalman2=sort(miss_DGL0_kalman2);
% miss_DGL1_true2=sort(miss_DGL1_true2);
% miss_DGL1_kalman2=sort(miss_DGL1_kalman2);
% miss_DGL1_plus_true2=sort(miss_DGL1_plus_true2);
% miss_DGL1_plus_kalman2=sort(miss_DGL1_plus_kalman2);
% miss_DGL1_minus_true2=sort(miss_DGL1_minus_true2);
% miss_DGL1_minus_kalman2=sort(miss_DGL1_minus_kalman2);
% 
% cdf_array=(1/M)*[1:M];
% 
% % small noise
% figure(131)
% semilogx(miss_PN_true1,cdf_array,'b',miss_PN_kalman1,cdf_array,'b--')
% xlabel('Miss Distance (m)')
% ylabel('Miss CDF')
% title('PN - Miss Distance CDF')
% legend('without kalman filter','with kalman filter')
% 
% figure(132)
% semilogx(miss_APN_true1,cdf_array,'b',miss_APN_kalman1,cdf_array,'b--')
% xlabel('Miss Distance (m)')
% ylabel('Miss CDF')
% title('APN - Miss Distance CDF')
% legend('without kalman filter','with kalman filter')
% 
% figure(133)
% semilogx(miss_OGL_true1,cdf_array,'b',miss_OGL_kalman1,cdf_array,'b--')
% xlabel('Miss Distance (m)')
% ylabel('Miss CDF')
% title('OGL - Miss Distance CDF')
% legend('without kalman filter','with kalman filter')
% 
% figure(134)
% semilogx(miss_LQDG_true1,cdf_array,'b',miss_LQDG_kalman1,cdf_array,'b--',miss_LQDG_plus_true1,cdf_array,'r',miss_LQDG_plus_kalman1,cdf_array,'r--',miss_LQDG_minus_true1,cdf_array,'m',miss_LQDG_minus_kalman1,cdf_array,'m--')
% xlabel('Miss Distance (m)')
% ylabel('Miss CDF')
% title('LQDG - Miss Distance CDF')
% legend('true theta - without kalman filter','true theta - with kalman filter','higher theta - without kalman filter','higher theta - with kalman filter','lower theta - without kalman filter','lower theta - with kalman filter')
% 
% figure(135)
% semilogx(miss_DGL0_true1,cdf_array,'b',miss_DGL0_kalman1,cdf_array,'b--')
% xlabel('Miss Distance (m)')
% ylabel('Miss CDF')
% title('DGL0 - Miss Distance CDF')
% legend('without kalman filter','with kalman filter');
% 
% figure(136)
% semilogx(miss_DGL1_true1,cdf_array,'b',miss_DGL1_kalman1,cdf_array,'b--',miss_DGL1_plus_true1,cdf_array,'r',miss_DGL1_plus_kalman1,cdf_array,'r--',miss_DGL1_minus_true1,cdf_array,'m',miss_DGL1_minus_kalman1,cdf_array,'m--')
% xlabel('Miss Distance (m)')
% ylabel('Miss CDF')
% title('DGL1 - Miss Distance CDF')
% legend('true theta - without kalman filter','true theta - with kalman filter','higher theta - without kalman filter','higher theta - with kalman filter','lower theta - without kalman filter','lower theta - with kalman filter')
% 
% % large noise
% figure(137)
% semilogx(miss_PN_true2,cdf_array,'b',miss_PN_kalman2,cdf_array,'b--')
% xlabel('Miss Distance (m)')
% ylabel('Miss CDF')
% title('PN - Miss Distance CDF')
% legend('without kalman filter','with kalman filter')
% 
% figure(138)
% semilogx(miss_APN_true2,cdf_array,'b',miss_APN_kalman2,cdf_array,'b--')
% xlabel('Miss Distance (m)')
% ylabel('Miss CDF')
% title('APN - Miss Distance CDF')
% legend('without kalman filter','with kalman filter')
% 
% figure(139)
% semilogx(miss_OGL_true2,cdf_array,'b',miss_OGL_kalman2,cdf_array,'b--')
% xlabel('Miss Distance (m)')
% ylabel('Miss CDF')
% title('OGL - Miss Distance CDF')
% legend('without kalman filter','with kalman filter')
% 
% figure(140)
% semilogx(miss_LQDG_true2,cdf_array,'b',miss_LQDG_kalman2,cdf_array,'b--',miss_LQDG_plus_true2,cdf_array,'r',miss_LQDG_plus_kalman2,cdf_array,'r--',miss_LQDG_minus_true2,cdf_array,'m',miss_LQDG_minus_kalman2,cdf_array,'m--')
% xlabel('Miss Distance (m)')
% ylabel('Miss CDF')
% title('LQDG - Miss Distance CDF')
% legend('true theta - without kalman filter','true theta - with kalman filter','higher theta - without kalman filter','higher theta - with kalman filter','lower theta - without kalman filter','lower theta - with kalman filter')
% 
% figure(141)
% semilogx(miss_DGL0_true2,cdf_array,'b',miss_DGL0_kalman2,cdf_array,'b--')
% xlabel('Miss Distance (m)')
% ylabel('Miss CDF')
% title('DGL0 - Miss Distance CDF')
% legend('without kalman filter','with kalman filter');
% 
% figure(142)
% semilogx(miss_DGL1_true2,cdf_array,'b',miss_DGL1_kalman2,cdf_array,'b--',miss_DGL1_plus_true2,cdf_array,'r',miss_DGL1_plus_kalman2,cdf_array,'r--',miss_DGL1_minus_true2,cdf_array,'m',miss_DGL1_minus_kalman2,cdf_array,'m--')
% xlabel('Miss Distance (m)')
% ylabel('Miss CDF')
% title('DGL1 - Miss Distance CDF')
% legend('true theta - without kalman filter','true theta - with kalman filter','higher theta - without kalman filter','higher theta - with kalman filter','lower theta - without kalman filter','lower theta - with kalman filter')
% toc