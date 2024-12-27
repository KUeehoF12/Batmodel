function [period,t_initial,t_final] = period_calc(p,l,p_next,l_next,Inv0,NumNodes,NumHouses)
    
    clear
    Date = 20170502;
    PVDir = 'D:\data\CRESTデータセット\44071_東京都練馬区\住宅PV実測\'; %PV出力のフォルダ
    LoadDir = 'D:\data\CRESTデータセット\44071_東京都練馬区\住宅負荷実測\';%負荷データのフォルダ    

    p=readmatrix([PVDir,'Individual_ResidentialPV_Real_1m_44071_',num2str(Date),'.csv']);%元の範囲：A1:TN24->A1:TZ24
    l=readmatrix([LoadDir,'Individual_ResidentialLoad_Real_1m_44071_',num2str(Date),'.csv']);
    p = p(:,1:528);
    %disp(size(p));
    l = l(:,1:528);
    p  = p.*2.5;
    p_next=readmatrix([PVDir,'Individual_ResidentialPV_Real_1m_44071_',num2str(Date+1),'.csv']);%元の範囲：A1:TN24->A1:TZ24
    l_next=readmatrix([LoadDir,'Individual_ResidentialLoad_Real_1m_44071_',num2str(Date+1),'.csv']);
    p_next = p_next(:,1:528);
    %disp(size(p));
    l_next = l_next(:,1:528);
    p_next  = p_next.*2.5;
    NumNodes = 44;
    NumHouses = NumNodes*12;
    %}
    t_initial = zeros(1,NumHouses);
    t_final = zeros(1,NumHouses);
    period = zeros(1,NumHouses);

    for t = 2:1440
        for i = 1:NumHouses
            if (p(t,i)<l(t,i) & p(t-1,i)>l(t-1,i) & t>=720)
                t_initial(i) = t;
                continue
            end
            if (p_next(t,i)>l_next(t,i) & p_next(t-1,i)<l_next(t-1,i) & t<=720)
                t_final(i) = t;
            end
        end
    end
    period(:) = (1440 - t_initial(:)) + t_final(:);
end