function [T_sim_sw,i_sw,mode_before,mode_after] = switch_time_est(miss_dis,miss_STAG_SW,h,Dt,miu_k,mean_ar,SD_ar)
% function estimating the initial mode and the switch time of the target
% Inputs: 
% miss_dis [m] - the estimated miss distance of the first missile
% miss_STAG [m] - database of miss distances of the first missile as function of T_sw
% h [m] - the resolution of the database search
% Dt [sec] - the time between two samplings
% miu_k - the modal probabilities calculated by the pursuers
% mean_ar [m] - the mean miss distance of the first missile as function of T_sw
% SD_ar [m] - the SD of the miss distance of the first missile as function of T_sw
% Outputs: 
% T_sim_sw [sec] - the estimated switch time
% i_sw - the index of the estimated switch time
% mode_before - the estimated mode of the target before the switch
% mode_after - the estimated mode of the target after the switch

% find where the mode change

jb=1;
ja=1;
jc=1;
js=1;
k_before_best(jb)=-1;
k_after_best(ja)=-1;
k_best(jc)=-1;
k_strike(js)=-1;
before_best(jb)=mean(miu_k(1,:,1));
after_best(ja)=mean(miu_k(1,:,1));
comb_best(jc)=abs(mean(miu_k(1,:,1))-0.5);

for k=2:length(miu_k(1,:,1))
    mean_before=mean(miu_k(1,1:k-1,1));
    mean_after=mean(miu_k(1,k:end,1));
    if (mean_before-0.5)*(mean_after-0.5)<0
        if abs(mean_before-0.5)>abs(before_best(jb)-0.5)
            jb=jb+1;
            k_before_best(jb)=k;
            before_best(jb)=mean_before;
        end
        if abs(mean_after-0.5)>abs(after_best(ja)-0.5)
            ja=ja+1;
            k_after_best(ja)=k;
            after_best(ja)=mean_after;
        end
        if abs(mean_before-0.5)+abs(mean_after-0.5)>comb_best(jc)
            jc=jc+1;
            k_best(jc)=k;
            comb_best(jc)=abs(mean_before-0.5)+abs(mean_after-0.5);
            if k_before_best(jb)==k && k_after_best(ja)==k
                js=js+1;
                k_strike(js)=k;
            end
        end
    end
end

if k_strike(js)<0
    max_i_sw=k_best(jc);
else
    max_i_sw=k_strike(js);
end

if max_i_sw==-1
    mode_before=2-round(mean(miu_k(1,1:end,1)));
    mode_after=3-mode_before;
    
    i_sw=length(miu_k(1,:,1))+5;
    T_sim_sw=(i_sw-1)*Dt;
else  
    mode_before=2-round(mean(miu_k(1,1:max_i_sw,1)));
    mode_after=3-mode_before;
    
    max_i_sw=ceil((max_i_sw-1)/5)*5+1;
    max_T_sw=(max_i_sw-1)*Dt;
    
    T_switch=[0.1:0.05:max_T_sw];
        
    % use mean+SD graph as pdf    
    p_T_sw=mean_ar+SD_ar;
    p_T_sw=p_T_sw(1:length(T_switch));
    p_T_sw=p_T_sw/sum(p_T_sw);
    
    p_miss_t=zeros(1,length(T_switch));
    p_miss_sw=zeros(1,length(T_switch));
    dmiss=h;
    while sum(p_miss_t)==0
        for j=1:length(T_switch)
            for k=1:length(miss_STAG_SW(j,:))
                if abs(miss_dis-miss_STAG_SW(j,k))<=dmiss
                    p_miss_t(j)=p_miss_t(j)+1;
                elseif sign(miss_dis-miss_STAG_SW(j,k))<0
                    break;
                end                    
            end
            p_miss_t(j)=p_miss_t(j)/length(miss_STAG_SW(j,:)); % the probabaility that the miss is around miss_dis, for specific T_Sw
            p_miss_sw(j)=p_miss_t(j)*p_T_sw(j);
        end
        temp1=T_switch.*p_miss_sw;
        if sum(p_miss_sw)==0
            % enlarge lower the resolution in case of lack of close miss
            % distances
            dmiss=1.5*dmiss;
        else
            T_est=sum(temp1)/sum(p_miss_sw);
        end
    end
    
    T_sim_sw=round(T_est/Dt)*Dt;
    i_sw=(T_sim_sw/Dt)+1;
    
    T_sim_sw=min(T_sim_sw,max_T_sw);
    i_sw=min(i_sw,max_i_sw);
end

end

