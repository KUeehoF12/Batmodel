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

NumNodes = 44;
NumHouses = NumNodes*12;

surplus = p - l - 3.95*ones(1440,NumHouses);
surplus(surplus<0) = 0;
SurplusSum = zeros(1440,NumHouses);

for h=1:NumHouses
    SurplusSum(:,h) = sum(surplus(:,h))*ones(1440,1)./60;
end

for t=1:1440
    SurplusSum(t,:) = SurplusSum(t,:) - (surplus(t,:)/60);
end
%}