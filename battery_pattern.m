%% 蓄電池の充放電パターン
function [battery] = battery_pattern(EV,SB,d,t,battery,d_least)
    %d_least = -2.1; %許容できる逆潮流->0以下
    %{
    % 充電（EV在宅時）
    battery.pattern1 = ((d(t,:)<0) & (EV.park(t,:)==1) & (EV.lb<=EV.SOC(t,:)) & (EV.SOC(t,:)<EV.DB(t,:)) & (SB.lb<=SB.SOC(t,:)) & (SB.SOC(t,:)<SB.ub));
    battery.P_list(t,:) = battery.P_list(t,:) + battery.pattern1 * 1;

    battery.pattern2 = ((d(t,:)<0) & (EV.park(t,:)==1) & (EV.lb<=EV.SOC(t,:)) & (EV.SOC(t,:)<EV.ub) & (SB.SOC(t,:)==SB.ub));
    battery.P_list(t,:) = battery.P_list(t,:) + battery.pattern2 * 2;

    battery.pattern3 = ((d(t,:)<0) & (EV.park(t,:)==1) & (EV.SOC(t,:)==EV.ub) & (SB.lb<=SB.SOC(t,:)) & (SB.SOC(t,:)<SB.ub));
    battery.P_list(t,:) = battery.P_list(t,:) + battery.pattern3 * 3;

    battery.pattern4 = ((d(t,:)<0) & (EV.park(t,:)==1) & (EV.DB(t,:)<=EV.SOC(t,:)) & (EV.SOC(t,:)<EV.ub) & (SB.lb<=SB.SOC(t,:)) & (SB.SOC(t,:)<SB.ub));
    battery.P_list(t,:) = battery.P_list(t,:) + battery.pattern4 * 4;

    battery.pattern5 = ((d(t,:)<0) & (EV.park(t,:)==1) & (EV.SOC(t,:)==EV.ub) & (SB.SOC(t,:)==SB.ub));
    battery.P_list(t,:) = battery.P_list(t,:) + battery.pattern5 * 5;

    % 放電（EV在宅時）
    battery.pattern6 = ((d(t,:)>0) & (EV.park(t,:)==1) & (EV.SOC(t,:)==EV.ub) & (SB.lb<SB.SOC(t,:)) & (SB.SOC(t,:)<=SB.ub));
    battery.P_list(t,:) = battery.P_list(t,:) + battery.pattern6 * 6;

    battery.pattern7 = ((d(t,:)>0) & (EV.park(t,:)==1) & (EV.DB(t,:)<EV.SOC(t,:)) & (EV.SOC(t,:)<=EV.ub) & (SB.SOC(t,:)==SB.lb));
    battery.P_list(t,:) = battery.P_list(t,:) + battery.pattern7 * 7;

    battery.pattern8 = ((d(t,:)>0) & (EV.park(t,:)==1) & (EV.lb<=EV.SOC(t,:)) & (EV.SOC(t,:)<=EV.DB(t,:)) & (SB.lb<SB.SOC(t,:)) & (SB.SOC(t,:)<=SB.ub));
    battery.P_list(t,:) = battery.P_list(t,:) + battery.pattern8 * 8;

    battery.pattern9 = ((d(t,:)>0) & (EV.park(t,:)==1) & (EV.DB(t,:)<EV.SOC(t,:)) & (EV.SOC(t,:)<EV.ub) & (SB.lb<SB.SOC(t,:)) & (SB.SOC(t,:)<=SB.ub));
    battery.P_list(t,:) = battery.P_list(t,:) + battery.pattern9 * 9;

    battery.pattern10 = ((d(t,:)>0) & (EV.park(t,:)==1) & (EV.lb<=EV.SOC(t,:)) & (EV.SOC(t,:)<=EV.DB(t,:)) & (SB.SOC(t,:)==SB.lb));
    battery.P_list(t,:) = battery.P_list(t,:) + battery.pattern10 * 10;
    %}
    % 充電（EV不在時）
    battery.pattern_11 = ((d(t,:)<d_least) & (EV.park(t,:)==0));
    battery.P_list(t,:) = battery.P_list(t,:) + battery.pattern_11 * 11;

    % 放電（EV不在時）
    battery.pattern_12 = ((d(t,:)>0) & (EV.park(t,:)==0));
    battery.P_list(t,:) = battery.P_list(t,:) + battery.pattern_12 * 12;
    
    % 充放電なし
    battery.pattern_0 = (d(t,:)>=d_least & d(t,:)<=0);
    battery.P_list(t,:) = battery.P_list(t,:) + battery.pattern_0 * 13;
    
    

end