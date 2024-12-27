function [self_con] = SelfConsumption(p,l,p_next,l_next,t_initial,t_final,Inv0,NumHouses,BatRemain)

%動作確認用
clear
Date = 20170502;
PVDir = 'D:\data\CRESTデータセット\44071_東京都練馬区\住宅PV実測\'; %PV出力のフォルダ
LoadDir = 'D:\data\CRESTデータセット\44071_東京都練馬区\住宅負荷実測\';%負荷データのフォルダ    
p=readmatrix([PVDir,'Individual_ResidentialPV_Real_1m_44071_',num2str(Date+1),'.csv']);%元の範囲：A1:TN24->A1:TZ24
l=readmatrix([LoadDir,'Individual_ResidentialLoad_Real_1m_44071_',num2str(Date+1),'.csv']);
p = p(:,1:528);
l = l(:,1:528);
p  = p.*2.5;
%disp(size(p));

p_next=readmatrix([PVDir,'Individual_ResidentialPV_Real_1m_44071_',num2str(Date+1),'.csv']);%元の範囲：A1:TN24->A1:TZ24
l_next=readmatrix([LoadDir,'Individual_ResidentialLoad_Real_1m_44071_',num2str(Date+1),'.csv']);
p_next = p_next(:,1:528);
%disp(size(p));
l_next = l_next(:,1:528);
p_next  = p_next.*2.5;
NumNodes = 44;
NumHouses = NumNodes*12;
Inv0 = 5;
[~,t_initial,t_final] = period_calc(p,l,p_next,l_next,Inv0,NumNodes,NumHouses);
t_initial_first = t_initial;
t_final_first = t_final;
BatRemain = BatRemainChecker(p,l);

%}
    self_con = zeros(1,NumHouses);
    %BatRemainPre = BatRemain(t_initial,:);
    %BatDischarge = zeros(1,NumHouses);
    for h=1:NumHouses
        BatRemainNow = BatRemain(t_initial(h),h);
        %BatRemainPre = BatRemainNow;
        BatDischarge = cat(1,l(t_initial:1440,h)-p(t_initial:1440,h),l_next(1:t_final,h)-p_next(1:t_final,h));
        BatDischarge(BatDischarge>Inv0) = Inv0;
        
        for t =1:1440-t_initial+t_final+1
            if BatRemainNow - BatDischarge(t) < 0
                BatDischarge(t) = BatRemainNow;
            end
            BatRemainNow = BatRemainNow - BatDischarge(t);
        end
        %}
        self_con(h) = sum(BatDischarge);
    end
    %{
    for h=1:NumHouses
        self_con(h) = sum(BatDischarge(:,h));
    end
    %}
end