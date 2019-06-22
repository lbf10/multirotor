%% Generate table
clear all
%% Choose files
filename = '1';
controller = [];
while ~strcmp(filename,'')
    [filename, pathname] = uigetfile('*.mat')
    try
        aux = load([pathname,'/',filename]);
        controller = [controller aux.data];
    catch
        filename = '';
    end
end

%% Radar 1 - Overall robustness
sr = [];
Vpsim = [];
Vep = [];
Vp = [];
for it=1:length(controller)
    sr = [sr; controller(it).perfectSuccessRate];
    Vpsim = [Vpsim; controller(it).ssRobustness.variance];
    Vep = [Vep; controller(it).epRobustness.variance];
    Vp = [Vp; controller(it).PRobustness.variance];
end
sr = 1-sr/max(sr);
Vpsim = Vpsim/max(Vpsim);
Vep = Vep/max(Vep);
Vp = Vp/max(Vp);
radarPlot2([sr Vpsim Vep Vp]', 'o-','LineWidth', 3, 'MarkerFaceColor', [0,0,0])
% radarplot([sr Vpsim Vep Vp],{'S_r','\hat{V}_{p_{sim}}','\hat{V}_{e_{p,RMS}}','\hat{V}_{P_{RMS}}'},{},{},{},10)
legend('1.2.1 (PID)','1.2.2.3 (SOSMC Direct)','1.2.3.2 (R-LQR with rotor failures)','1.2.4 (Mode-Independent Markovian)','1.2.5.2 (Adaptive with PIDD)')

%% Radar 2 - Overall robustness
sr = [];
Vpsim = [];
Vep = [];
Vp = [];
for it=1:length(controller)
    sr = [sr; controller(it).perfectSuccessRate];
    Vpsim = [Vpsim; controller(it).ssRobustness.variance];
    Vep = [Vep; controller(it).epRobustness.variance];
    Vp = [Vp; controller(it).PRobustness.variance];
end
sr = 1-sr/max(sr);
Vpsim = Vpsim/max(Vpsim);
Vep = Vep/max(Vep);
Vp = Vp/max(Vp);
radarplot([sr Vpsim Vep Vp],{'S_r','\hat{V}_{p_{sim}}','\hat{V}_{e_{p,RMS}}','\hat{V}_{P_{RMS}}'})
legend('1.2.1 (PID)','1.2.2.3 (SOSMC Direct)','1.2.3.2 (R-LQR with rotor failures)','1.2.4 (Mode-Independent Markovian)','1.2.5.2 (Adaptive with PIDD)')
