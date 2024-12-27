function [maxBatRemain] = FlucBatRemain(l_next,p_next,NumNodes,NumHouses)
%{
%動作確認用
clear
Date = 20170502;
PVDir = 'D:\data\CRESTデータセット\44071_東京都練馬区\住宅PV実測\'; %PV出力のフォルダ
LoadDir = 'D:\data\CRESTデータセット\44071_東京都練馬区\住宅負荷実測\';%負荷データのフォルダ    

p_next=readmatrix([PVDir,'Individual_ResidentialPV_Real_1m_44071_',num2str(Date+1),'.csv']);%元の範囲：A1:TN24->A1:TZ24
l_next=readmatrix([LoadDir,'Individual_ResidentialLoad_Real_1m_44071_',num2str(Date+1),'.csv']);
p_next = p_next(:,1:528);
%disp(size(p));
l_next = l_next(:,1:528);
p_next  = p_next.*2.5;
NumNodes = 44;
NumHouses = NumNodes*12;
%}
    l = l_next;
    p = p_next;
    % シミュレーション期間
    %Day.Start = 123;
    %Day.End = 153;
    Day.Span = 1;%Day.End - Day.Start + 1;

    % 1日の時間粒度と時間軸
    data.gran = 1; % x分値
    time.gran = 60 / data.gran; % time granularity 1時間あたりの時間粒度
    for i = 1:24
        time.h(time.gran*i-5:time.gran*i,1) = i;
    end
    time.h = repmat(time.h,Day.Span,1);
    Sim.Span = size(time.h,1);

    d_least = -4.3;%-2.1?-4.3?
    d=l-p; %正味の需要->必要に応じて単位調整
    [row,column] = size(d);
    d(Sim.Span+1:row,:) = []; %サイズ調整
    %writematrix(d,'BatModel.xlsx','Sheet','d','Range','A1')
    row = Sim.Span;
    d1_acd = zeros(row,column); % After Charging and Discharging
    d2_acd = zeros(row,column); % After Charging and Discharging
    hd1_agt = zeros(row,column); % After Group Transaction
    hd2_agt = zeros(row,column); % After Group Transaction
    hd2_agt_EV = zeros(row,column); % After Group Transaction ＋ EV潮流
    % hd_agt_temp = zeros(row,column); % After Group Transaction ＋ EV潮流
    hd_fin = zeros(row,column); %ある時刻における最終的な正味の需要^d
    d_nEfG = zeros(row,column); %最終的な正味の需要（EV系統充電負荷なし）^d without EV from Grid
    maxBatRemain = zeros(1,NumHouses);
    %記録用
    SBSOC_array = zeros(row,column);
    SBACV_array = zeros(row,column);
    SBADV_array = zeros(row,column);
    SBreq1first_array = zeros(row,column);
    SBreq1_added_array = zeros(row,column);
    SBreq1second_array = zeros(row,column);
    SBreq2second_array = zeros(row,column);
    SBreq1third_array = zeros(row,column);
    SBreq2Inv = zeros(row,column);
    SBreq2Cap = zeros(row,column);
    SBreq1_array = zeros(row,column);
    SBreq2 = zeros(row,column);
    SBreq1_acd_array = zeros(row,column);
    SBreq1_eff_array = zeros(row,column);
    battey_array = zeros(row,column);
    item1 = zeros(row,column);
    item2 = zeros(row,column);
    item3 = zeros(row,column);
    item4 = zeros(row,column);
    item5 = zeros(row,column);

    battery.P_list=zeros(row,column);
    battery.add_charge=zeros(row,column);

    % 蓄電設備（BESS）の設定
    [EV,SB,BESS_eff,A1,A2,B1,B2,C1,C2] = BandE_predict(row,column); %これらの文字から始まる変数は，特に定義されていなければここで取得

    % BESS潮流
    SB.req1_acd = zeros(row,column);
    SB.req2_acd = zeros(row,column);

    % 実効充放電量(バッテリー増減量)
    SB.req1_eff = zeros(row,column);
    SB.req2_eff = zeros(row,column);

    for t=1:Sim.Span
        % 定置型バッテリー状態
        SB.SOC = (SB.remain / SB.C) *100;
        SB.ACV = SB.C * (SB.ub/100) - SB.remain; % Available Charge Value
        SB.ADV = SB.C * (SB.lb/100) - SB.remain; % Available Discharge Value
        
        SBSOC_array = SB.SOC;
        SBACV_array = SB.ACV;
        SBADV_array = SB.ADV;
        %}
        %% 充放電パターン
        % 充放電要求量
        [battery] = battery_pattern(EV,SB,d,t,battery,d_least);
        % case 1 %EV在宅，(充電)EV→SB    % case 2 %EV在宅，(充電)EV    % case 3 %EV在宅，(充電)SB    % case 4 %EV在宅，(充電)SB→EV    % case 5 %EV在宅，(充電)なし
        % case 6 %EV在宅，(放電)EV→SB    % case 7 %EV在宅，(放電)EV    % case 8 %EV在宅，(放電)SB    % case 9 %EV在宅，(放電)SB→EV    % case 10 %EV在宅，(放電)なし
        % case 11 %EV不在，(充電)SB    % case 12 %EV不在，(放電)SB    % case 13 %EV在宅，EV下限値以下

        temp_Plist = battery.P_list(t,:);

        %% 充放電計算
        SB.req1 = -d(t,:);
        SBreq1first_array(t,:) = SB.req1;
        % SBの充放電が先行
        SB.req1(SB.req1<=abs(d_least)&SB.req1>0) = 0;
        SB.req1(SB.req1>abs(d_least)) = SB.req1(SB.req1>abs(d_least))+d_least;
        %disp(abs(d_least));
        %SB.req1(SB.req1<=0) = SB.req1(SB.req1<=0);
        SBreq1_added_array(t,:)=SB.req1;
        % インバータ制約
        SB.req1(SB.req1>SB.Inv0/BESS_eff) = SB.Inv0/BESS_eff; %SB.Inv0，BESS_effは51行目で取得->EVの充電需要がインバータ容量を充電効率で割った値より大きいは場合->後者に需要を変更
        SB.req1(SB.req1<-SB.Inv0) = -SB.Inv0; %EVの放電可能な電力がインバータ容量を超える->EVの放電可能電力をインバータ容量に変更
        SBreq1second_array(t,:) = SB.req1;
        % 容量制約（余力）
        SB.req1 = (SB.req1) .* ( SB.req1 <= (time.gran*SB.ACV(t,:))/BESS_eff ) + (time.gran*SB.ACV(t,:))/BESS_eff .* ( SB.req1 > (time.gran*SB.ACV(t,:))/BESS_eff );%time.granは20行目，EV.ACVは109行目で定義
        SBreq1third_array(t,:) = SB.req1;
        SB.req1 = SB.req1 .* ( SB.req1 >= (time.gran*SB.ADV(t,:))*BESS_eff ) + (time.gran*SB.ADV(t,:))*BESS_eff .* ( SB.req1 < (time.gran*SB.ADV(t,:))*BESS_eff );%time.granは20行目，EV.ACVは110行目で定義
        %item1(t,:) = time.gran*SB.ADV(t,:);
        %item2(t,:) = (time.gran*SB.ADV(t,:))*BESS_eff;
        %item3(t,:) = (time.gran*SB.ADV(t,:))*BESS_eff .* ( SB.req1 < (time.gran*SB.ADV(t,:))*BESS_eff );
        %item4(t,:) = SB.req1 .* ( SB.req1 >= (time.gran*SB.ADV(t,:))*BESS_eff );
        %item5(t,:) = ( SB.req1 >= (time.gran*SB.ADV(t,:))*BESS_eff );

        available_cd = (battery.P_list(t,:)==3) | (battery.P_list(t,:)==4) | (battery.P_list(t,:)==8) | (battery.P_list(t,:)==9) | (battery.P_list(t,:)==11) | (battery.P_list(t,:)==12);
        battey_array(t,:) = battery.P_list(t,:);
        %{
        if (battery.P_list(t,:)==11) | (battery.P_list(t,:)==12)
            available_cd(:) = 1;
        else
            available_cd(:) = 0;
        end
        %}
        SB.req1_acd(t,:) = SB.req1 .* available_cd; %充放電1回目後のSB部の潮流
        SBreq1_acd_array(t,:) = SB.req1_acd(t,:);

        % 実効充放電量(バッテリー増減量) 充電効率と時間粒度を考慮し、実際のバッテリー増減量（req1_eff）を計算。
        SB.req1_eff(t,:) = SB.req1_acd(t,:).*(SB.req1_acd(t,:)>=0) * BESS_eff / time.gran + SB.req1_acd(t,:).*(SB.req1_acd(t,:)<0) / BESS_eff / time.gran;
        SBreq1_eff_array(t,:) = SB.req1_eff(t,:);

        %% 充放電による変化
        d1_acd(t,:) = d(t,:) + SB.req1_acd(t,:);

        % インバータ変化計算
        SB.Inv1_c = SB.Inv0 - (SB.req1_acd(t,:).*(SB.req1_acd(t,:)>=0) * BESS_eff);
        SB.Inv1_d = SB.Inv0 + (SB.req1_acd(t,:).*(SB.req1_acd(t,:)<0));

        % 定置型バッテリー状態
        SB.remain(t,:) = SB.remain(t,:) + SB.req1_eff(t,:);
        SB.SOC = (SB.remain / SB.C) *100;
        SB.ACV = SB.C * (SB.ub/100) - SB.remain; % Available Charge Value
        SB.ADV = SB.C * (SB.lb/100) - SB.remain; % Available Discharge Value

        if t~=Sim.Span
            SB.remain(t+1,:) = SB.remain(t,:);
        end
        SBreq1_array(t,:)=SB.req1;

            
        %% 充放電計算2回目
            SB.req2 = -d1_acd(t,:);
            %disp(size(SBreq2));
            
            SB.req2(SB.req2<=abs(d_least)) = 0;
            SB.req2(SB.req2>abs(d_least)) = SB.req2(SB.req2>abs(d_least))+d_least;
            SBreq2(t,:) = SB.req2;
            %逆潮流の上限を超える余剰を出す需要家の余剰を，出さない需要家に分配
            for i=1:NumNodes %ノードことに処理
                %A相
                SBreq2_phaseA = [SBreq2(t,12*(i-1)+1),SBreq2(t,12*(i-1)+2),SBreq2(t,12*(i-1)+3),SBreq2(t,12*(i-1)+4)]; %該当範囲の負荷抽出
                num_A = sum(find(SBreq2_phaseA>0))/max(size(find(SBreq2_phaseA==0))); %逆潮流の上限を超えた余剰を同じ低圧系統の他の需要家で均等に分けたときの，充電需要の増加分
                SBreq2_phaseA(SBreq2_phaseA==0) = num_A; %逆潮流の上限を超えなかった需要家の，充電需要の更新
                SBreq2_phaseA(SBreq2_phaseA~=num_A) = 0; %逆潮流の上限を超えた需要家の，充電需要の更新
                %B相
                SBreq2_phaseB = [SBreq2(t,12*(i-1)+5),SBreq2(t,12*(i-1)+6),SBreq2(t,12*(i-1)+7),SBreq2(t,12*(i-1)+8)];
                num_B = sum(find(SBreq2_phaseB>0))/max(size(find(SBreq2_phaseB==0)));
                SBreq2_phaseB(SBreq2_phaseB==0) = num_B;
                SBreq2_phaseB(SBreq2_phaseB~=num_B) = 0;
                %C相
                SBreq2_phaseC = [SBreq2(t,12*(i-1)+9),SBreq2(t,12*(i-1)+10),SBreq2(t,12*(i-1)+11),SBreq2(t,12*(i-1)+12)];
                num_C = sum(find(SBreq2_phaseC>0))/max(size(find(SBreq2_phaseC==0)));
                SBreq2_phaseC(SBreq2_phaseC==0) = num_C;
                SBreq2_phaseC(SBreq2_phaseC~=num_C) = 0;
                for j=1:12
                    if j>0 && j<=4
                        SBreq2(t,12*(i-1)+j) = SBreq2_phaseA(j);
                    elseif j>4 && j<=8
                        SBreq2(t,12*(i-1)+j) = SBreq2_phaseB(j-4);
                    else
                        SBreq2(t,12*(i-1)+j) = SBreq2_phaseC(j-8);
                    end
                end

            end
            %disp(SBreq2(t,:));
            SB.req2 = SBreq2(t,:);
            SBreq2second_array(t,:) = SB.req2;
            % SBの充放電が先行
            % インバータ制約
            SB.req2 = SB.req2 .* (SB.req2 <= SB.Inv1_c/BESS_eff) + SB.Inv1_c/BESS_eff .* (SB.req2 > SB.Inv1_c/BESS_eff);
            SB.req2 = SB.req2 .* (SB.req2 >= -SB.Inv1_d) - SB.Inv1_d .* (SB.req2 < -SB.Inv1_d);
            SBreq2Inv(t,:) = SB.req2;
            % 容量制約（余力）
            SB.req2 = SB.req2 .* ( SB.req2 <= (time.gran*SB.ACV(t,:))/BESS_eff ) + (time.gran*SB.ACV(t,:))/BESS_eff .* ( SB.req2 > (time.gran*SB.ACV(t,:))/BESS_eff );
            SB.req2 = SB.req2 .* ( SB.req2 >= (time.gran*SB.ADV(t,:))*BESS_eff ) + (time.gran*SB.ADV(t,:))*BESS_eff .* ( SB.req2 < (time.gran*SB.ADV(t,:))*BESS_eff );
            SBreq2Cap(t,:) = SB.req2;

            available_cd = (battery.P_list(t,:)==1) | (battery.P_list(t,:)==6);
            SB.req2_acd(t,:) = SB.req2_acd(t,:) + SB.req2 ;%.* available_cd; %充放電2回目後のSB部の潮流

            % 実効充放電量(バッテリー増減量)
            SB.req2_eff(t,:) = SB.req2_acd(t,:).*(SB.req2_acd(t,:)>=0) * BESS_eff / time.gran + SB.req2_acd(t,:).*(SB.req2_acd(t,:)<0) / BESS_eff / time.gran;

            %% 充放電による変化
            d2_acd(t,:) = d1_acd(t,:) + SB.req2_acd(t,:); %正味＋充放電

            % インバータ変化計算
            SB.Inv2_c = SB.Inv1_c - (SB.req2_acd(t,:).*(SB.req2_acd(t,:)>=0) * BESS_eff);
            SB.Inv2_d = SB.Inv1_d + (SB.req2_acd(t,:).*(SB.req2_acd(t,:)<0));


            % 定置型バッテリー状態
            SB.remain(t,:) = SB.remain(t,:) + SB.req2_eff(t,:);
            SB.SOC = (SB.remain / SB.C) *100;
            SB.ACV = SB.C * (SB.ub/100) - SB.remain; % Available Charge Value
            SB.ADV = SB.C * (SB.lb/100) - SB.remain; % Available Discharge Value
            %}
            if t~=Sim.Span
                SB.remain(t+1,:) = SB.remain(t,:);
            end
%}
    end
    %{
    writematrix(SB.remain,'BatModel.xlsx','Sheet','SB.remain','Range','A1')
    writematrix(SBreq1_array,'BatModel.xlsx','Sheet','SB.req1','Range','A1')
    writematrix(SB.req1_eff,'BatModel.xlsx','Sheet','SB.req1_eff','Range','A1')
    writematrix(d1_acd,'BatModel.xlsx','Sheet','d1_acd','Range','A1')
    writematrix(SB.req1_acd,'BatModel.xlsx','Sheet','SB.req1_acd','Range','A1')
    
    writematrix(available_cd,'BatModel.xlsx','Sheet','available_cd','Range','A1')
    writematrix(SBSOC_array,'BatModel.xlsx','Sheet','SB.SOC','Range','A1')
    writematrix(SBACV_array,'BatModel.xlsx','Sheet','SB.ACV','Range','A1')
    writematrix(SBADV_array,'BatModel.xlsx','Sheet','SB.ADV','Range','A1')
    writematrix(SBreq1first_array,'BatModel.xlsx','Sheet','SBreq1first','Range','A1')
    writematrix(SBreq1second_array,'BatModel.xlsx','Sheet','SBreq1second','Range','A1')
    writematrix(SBreq1third_array,'BatModel.xlsx','Sheet','SBreq1third','Range','A1')
    
    writematrix(item1,'BatModel.xlsx','Sheet','item1','Range','A1')
    writematrix(item2,'BatModel.xlsx','Sheet','item2','Range','A1')
    writematrix(item3,'BatModel.xlsx','Sheet','item3','Range','A1')
    writematrix(item4,'BatModel.xlsx','Sheet','item2','Range','A1')
    writematrix(item5,'BatModel.xlsx','Sheet','item3','Range','A1')
    %}

    
  
    BatRemain=SB.remain;
    BatCharge=(SB.req1_eff+SB.req2_eff)*time.gran;
    Load=l+SB.req1_acd+SB.req2_eff; %なぜか10/時間粒度でわらないとうまくいかない
    %writematrix(Load,'BatModel.xlsx','Sheet','Load','Range','A1')
    for h=1:NumHouses
        maxBatRemain(:,h) = max(BatRemain(:,h));
    end
    


end