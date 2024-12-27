%% 蓄電設備の設計
function [EV,SB,BESS_eff,A1,A2,B1,B2,C1,C2] = BandE_predict(row,column)
    
    %% EVの設計変数
    EV.C = 60; %Capacity
    EV.Inv0 = 3; %インバータ出力
    EV.lb = 20; %最小値
    %※バッテリーの寿命的な話も背景にあり%※
    %※ここでは緊急の運転を考えて「約48kmの移動」「8kWhの非常電源として可能」
    %※運転時は下限値の制限なし
    EV.ub = 100; %最大値
    EV.remain = zeros(row,column); %残存容量
    
    %% ピーク低減
    EV.peak_charge = ones(1,column);
    
    %% EVのドライブ関連の設計変数
    EV.CP = 6; %電費[km/kWh] ※CP:CostPerformance
    EV.park = zeros(row,column);
    EV.DB = zeros(row,column);
    EV.run_kW10m = zeros(row,column);
    
    % パターンA1
    A1.park=1; %在宅判定フラグ（在宅がデフォルト、在宅⇒1/不在⇒0）
    A1.st = 9;
    A1.et = 19;
    A1.distance = 150;
    A1.CaD = 82.5; % Constraint about Driving
    A1.CaD_span = 9;
    A1.run_total = 0;
    A1.drive_1dago = [0; 0; 0; 0; 0; 0; 1;];
    A1.drive_2dago = circshift(A1.drive_1dago,-1,1);
    A1.drive_today = circshift(A1.drive_1dago,1,1);
    
    % パターンA2
    A2.park=1; %在宅判定フラグ（在宅がデフォルト、在宅⇒1/不在⇒0）
    A2.st = 10;
    A2.et = 18;
    A2.distance = 50;
    A2.CaD = 41; % Constraint about Driving
    A2.CaD_span = 3;
    A2.run_total = 0;
    A2.drive_1dago = [0; 0; 0; 0; 0; 0; 1;];
    A2.drive_2dago = circshift(A2.drive_1dago,-1,1);
    A2.drive_today = circshift(A2.drive_1dago,1,1);
    
    % パターンB1
    B1.park=1; %在宅判定フラグ（在宅がデフォルト、在宅⇒1/不在⇒0）
    B1.st = 10;
    B1.et = 17;
    B1.distance = 50;
    B1.CaD = 41; % Constraint about Driving
    B1.CaD_span = 3;
    B1.run_total = 0;
    B1.drive_1dago = [1; 1; 0; 1; 0; 1; 0;];
    B1.drive_2dago = circshift(B1.drive_1dago,-1,1);
    B1.drive_today = circshift(B1.drive_1dago,1,1);
    
    % パターンB2
    B2.park=1; %在宅判定フラグ（在宅がデフォルト、在宅⇒1/不在⇒0）
    B2.st = 13;
    B2.et = 17;
    B2.distance = 5;
    B2.CaD = 22.5; % Constraint about Driving
    B2.CaD_span = 1;
    B2.run_total = 0;
    B2.drive_1dago = [1; 1; 0; 1; 0; 1; 0;];
    B2.drive_2dago = circshift(B2.drive_1dago,-1,1);
    B2.drive_today = circshift(B2.drive_1dago,1,1);
    
    % パターンC1
    C1.park=1; %在宅判定フラグ（在宅がデフォルト、在宅⇒1/不在⇒0）
    C1.st = 7;
    C1.et = 19;
    C1.distance = 50;
    C1.CaD = 41; % Constraint about Driving
    C1.CaD_span = 3;
    C1.run_total = 0;
    C1.drive_1dago = [0; 1; 1; 1; 1; 1; 0;];
    C1.drive_2dago = circshift(C1.drive_1dago,-1,1);
    C1.drive_today = circshift(C1.drive_1dago,1,1);
    
    % パターンC2
    C2.park=1; %在宅判定フラグ（在宅がデフォルト、在宅⇒1/不在⇒0）
    C2.st = 8;
    C2.et = 18;
    C2.distance = 15;
    C2.CaD = 26.5; % Constraint about Driving
    C2.CaD_span = 1;
    C2.run_total = 0;
    C2.drive_1dago = [0; 1; 1; 1; 1; 1; 0;];
    C2.drive_2dago = circshift(C2.drive_1dago,-1,1);
    C2.drive_today = circshift(C2.drive_1dago,1,1);
    
    %% SBの設計変数
    SB.C = 20; %0 or 5 or 20 %Capacity
    SB.Inv0 = 5; %3 or 6 %インバータ出力
    SB.lb = 0;%20; %最小値
    SB.ub = 100; %最大値
    SB.remain = zeros(row,column); %残存容量
    
    % Battery
    BESS_eff = 0.9; %BatteryEfficiency
    
    % 初期値
    EV.remain(1,:) = EV.C * 0;%0.5; %初期値50％
    SB.remain(1,:) = SB.C * 0;%0.5; %初期値50％
    
    
end