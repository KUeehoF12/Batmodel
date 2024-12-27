function [Load_1min]=linear_interp(Load_original)
    %{
    %テスト用
    clear
    Date = 20170502;
    LoadDir = 'C:\Users\Sojun_Iwashina\OneDrive - 東京理科大学\ドキュメント\卒研\database\demand\kanto\middle_buildings_lifestyle_30min\data_20241217161503\OPEN DATA\5月2日\';%負荷データのフォルダ
    Load_original=readmatrix([LoadDir,'G18000869_5.2','.xlsx']);
    %}
    %timestep_original = linspace(0,24,24);1時間値の場合
    Load_1min=zeros(1440,min(size(Load_original)));

    timestep_original = linspace(0,24,24*(60/30));
    timestep = linspace(0,24,1440);
    for h=1:min(size(Load_original))
        Load_1min(:,h) = interp1(timestep_original, Load_original(:,h), timestep);
    end
    %{
    %テスト用グラフ表示プログラム
    figure(1)
    plot(timestep,Load_1min)
    xlim([0 24]); set(gca, 'FontName', 'Helvetica', 'FontSize', 14, 'XTick',0:6:24,...
    'FontWeight','Bold')
    xlabel('Time [hour]'); ylabel('Load(1 min) [kW]'); grid on;

    figure(2)
    plot(timestep_original,Load_original)
    xlim([0 24]); set(gca, 'FontName', 'Helvetica', 'FontSize', 14, 'XTick',0:6:24,...
    'FontWeight','Bold')
    xlabel('Time [hour]'); ylabel('Load(30 min) [kW]'); grid on;
    %}
end