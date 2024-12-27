function [BatCharge_night] = NightPlan(p,l,p_next,l_next,t_initial,t_final,Inv0,Inv1_d,NumNodes,NumHouses,LoadHigh_1min,d_least,BESS_eff)
%{
%動作確認用
clear
Date = 20170502;
PVDir = 'D:\data\CRESTデータセット\44071_東京都練馬区\住宅PV実測\'; %PV出力のフォルダ
LoadDir = 'D:\data\CRESTデータセット\44071_東京都練馬区\住宅負荷実測\';%負荷データのフォルダ    
p=readmatrix([PVDir,'Individual_ResidentialPV_Real_1m_44071_',num2str(Date+1),'.csv']);%元の範囲：A1:TN24->A1:TZ24
l=readmatrix([LoadDir,'Individual_ResidentialLoad_Real_1m_44071_',num2str(Date+1),'.csv']);
LoadHighDir = 'C:\Users\Sojun_Iwashina\OneDrive - 東京理科大学\ドキュメント\卒研\database\demand\kanto\middle_buildings_lifestyle_30min\data_20241217161503\OPEN DATA\';%負荷データのフォルダ
p = p(:,1:528);
l = l(:,1:528);
p  = p.*2.5;
%disp(size(p));
LoadHigh_original=readmatrix([LoadHighDir,'G18000869_5.2','.xlsx']);
LoadHigh_1min = linear_interp(LoadHigh_original);

p_next=readmatrix([PVDir,'Individual_ResidentialPV_Real_1m_44071_',num2str(Date+1),'.csv']);%元の範囲：A1:TN24->A1:TZ24
l_next=readmatrix([LoadDir,'Individual_ResidentialLoad_Real_1m_44071_',num2str(Date+1),'.csv']);
p_next = p_next(:,1:528);
%disp(size(p));
l_next = l_next(:,1:528);
p_next  = p_next.*2.5;
NumNodes = 44;
NumHouses = NumNodes*12;
Inv0 = 5;
d_least = -3.95;
[~,t_initial,t_final] = period_calc(p,l,p_next,l_next,Inv0,NumNodes,NumHouses);
t_initial_first = t_initial;
t_final_first = t_final;
%BatRemain = BatRemainChecker(p,l);
[BatRemain,~,~,Inv1_d]=BatModel(l,p,NumNodes);
%}
    for t=1:1440
        LoadHigh_1min_sum(t) = sum(LoadHigh_1min(t,:));
    end
    
    %BatCharge_nightの初期化
    BatCharge_night = zeros(1440,NumHouses);
    
    %翌日充電する電力量の算出
    total_BatCharge_next = FlucBatRemain(l_next,p_next,NumNodes,NumHouses);

    %夜間の自家消費量の最大値
    self_con = SelfConsumption(p,l,p_next,l_next,t_initial,t_final,Inv0,NumHouses);

    %自家消費で消費できる電力と翌日充電する電力量の比較
    extra = self_con - total_BatCharge_next;
    extra(extra>=0) = 0;
    extra_in = extra;
    extra_sum = 0;

    for t=min(t_initial):1440
        extra_sum = 0;
        for h=1:NumHouses
            if p(t,h) - l(t,h) <= 0
                extra_sum = extra_sum + extra(h);
            end
        end
        if t==min(t_initial)
            extra_sum_1st = extra_sum;
        end
        for h=1:NumHouses
            if p(t,h) - l(t,h) <= 0
                BatCharge_night(t,h) = -LoadHigh_1min_sum(t)*extra(h)/extra_sum;
                if BatCharge_night(t,h)<extra(h)
                    BatCharge_night(t,h) = extra(h);
                end
                if BatCharge_night(t,h)*BESS_eff<-Inv1_d(t,h)
                    BatCharge_night(t,h) = -Inv1_d(t,h)/BESS_eff;
                end

            end
        end
        BatCharge_night(BatCharge_night<d_least) = d_least;
        extra = extra - BatCharge_night(t,:)./60;
    end

    %{
    for t=1:1440-t_initial+t_final+1
        extra = extra - BatCharge_night(t,:);
    end
    %}
end