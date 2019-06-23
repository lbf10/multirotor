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
figure
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
%radarplot([sr Vpsim Vep Vp],{'S_r','V_{p_{sim}}','V_{e_{p,RMS}}','V_{P_{RMS}}'},{},{'b','r','y','m','g'},{},10)
%legend('1.2.1 (PID)','1.2.2.3 (SOSMC Direct)','1.2.3.2 (R-LQR with rotor failures)','1.2.4 (Mode-Independent Markovian)','1.2.5.2 (Adaptive with PIDD)')
legend('2.2.1 (PID)','2.2.2.2 (SOSMC with PIDD)','2.2.3.2 (R-LQR with rotor failures)','2.2.4 (Mode-Dependent Markovian)')
title('Factor-independent robustness metrics')
%% Radar 2 - Simulation success STf
figure
ST1_ss = [];
ST2_ss = [];
ST3_ss = [];
ST4_ss = [];
ST5_ss = [];
ST6_ss = [];
ST7_ss = [];
for it=1:length(controller)
    ST1_ss = [ST1_ss; abs(controller(it).ssRobustness.STf1)];
    ST2_ss = [ST2_ss; abs(controller(it).ssRobustness.STf2)];
    ST3_ss = [ST3_ss; abs(controller(it).ssRobustness.STf3)];
    ST4_ss = [ST4_ss; abs(controller(it).ssRobustness.STf4)];
    ST5_ss = [ST5_ss; abs(controller(it).ssRobustness.STf5)];
    ST6_ss = [ST6_ss; abs(controller(it).ssRobustness.STf6)];
    ST7_ss = [ST7_ss; abs(controller(it).ssRobustness.STf7)];
end
% radarPlot2([ST1_ss ST2_ss ST3_ss ST4_ss ST5_ss ST6_ss ST7_ss]','o-','LineWidth', 3, 'MarkerFaceColor', [0,0,0])
radarplot([ST1_ss ST2_ss ST3_ss ST4_ss ST5_ss ST6_ss ST7_ss],{'|S_{T1,p_{sim}}|','|S_{T2,p_{sim}}|','|S_{T3,p_{sim}}|','|S_{T4,p_{sim}}|','|S_{T5,p_{sim}}|','|S_{T6,p_{sim}}|','|S_{T7,p_{sim}}|'},{},{'b','r','y','m','g'},{},10)
% legend('1.2.1 (PID)','1.2.2.3 (SOSMC Direct)','1.2.3.2 (R-LQR with rotor failures)','1.2.4 (Mode-Independent Markovian)','1.2.5.2 (Adaptive with PIDD)')
legend('2.2.1 (PID)','2.2.2.2 (SOSMC with PIDD)','2.2.3.2 (R-LQR with rotor failures)','2.2.4 (Mode-Dependent Markovian)')
title('Total effect indices for simulation success')
%% Radar 3 - Position error STf
figure
ST1_ep = [];
ST2_ep = [];
ST3_ep = [];
ST4_ep = [];
ST5_ep = [];
ST6_ep = [];
ST7_ep = [];
for it=1:length(controller)
    ST1_ep = [ST1_ep; abs(controller(it).epRobustness.STf1)];
    ST2_ep = [ST2_ep; abs(controller(it).epRobustness.STf2)];
    ST3_ep = [ST3_ep; abs(controller(it).epRobustness.STf3)];
    ST4_ep = [ST4_ep; abs(controller(it).epRobustness.STf4)];
    ST5_ep = [ST5_ep; abs(controller(it).epRobustness.STf5)];
    ST6_ep = [ST6_ep; abs(controller(it).epRobustness.STf6)];
    ST7_ep = [ST7_ep; abs(controller(it).epRobustness.STf7)];
end
% radarPlot2([ST1_ss ST2_ss ST3_ss ST4_ss ST5_ss ST6_ss ST7_ss]','o-','LineWidth', 3, 'MarkerFaceColor', [0,0,0])
radarplot([ST1_ep ST2_ep ST3_ep ST4_ep ST5_ep ST6_ep ST7_ep],{'|S_{T1,p_{pos}}|','|S_{T2,p_{pos}}|','|S_{T3,p_{pos}}|','|S_{T4,p_{pos}}|','|S_{T5,p_{pos}}|','|S_{T6,p_{pos}}|','|S_{T7,p_{pos}}|'},{},{'b','r','y','m','g'},{},10)
% legend('1.2.1 (PID)','1.2.2.3 (SOSMC Direct)','1.2.3.2 (R-LQR with rotor failures)','1.2.4 (Mode-Independent Markovian)','1.2.5.2 (Adaptive with PIDD)')
legend('2.2.1 (PID)','2.2.2.2 (SOSMC with PIDD)','2.2.3.2 (R-LQR with rotor failures)','2.2.4 (Mode-Dependent Markovian)')
title('Total effect indices for position error')
%% Radar 4 - RMS power STf
figure
ST1_p = [];
ST2_p = [];
ST3_p = [];
ST4_p = [];
ST5_p = [];
ST6_p = [];
ST7_p = [];
for it=1:length(controller)
    ST1_p = [ST1_p; abs(controller(it).PRobustness.STf1)];
    ST2_p = [ST2_p; abs(controller(it).PRobustness.STf2)];
    ST3_p = [ST3_p; abs(controller(it).PRobustness.STf3)];
    ST4_p = [ST4_p; abs(controller(it).PRobustness.STf4)];
    ST5_p = [ST5_p; abs(controller(it).PRobustness.STf5)];
    ST6_p = [ST6_p; abs(controller(it).PRobustness.STf6)];
    ST7_p = [ST7_p; abs(controller(it).PRobustness.STf7)];
end
% radarPlot2([ST1_ss ST2_ss ST3_ss ST4_ss ST5_ss ST6_ss ST7_ss]','o-','LineWidth', 3, 'MarkerFaceColor', [0,0,0])
radarplot([ST1_p ST2_p ST3_p ST4_p ST5_p ST6_p ST7_p],{'|S_{T1,p_{pow}}|','|S_{T2,p_{pow}}|','|S_{T3,p_{pow}}|','|S_{T4,p_{pow}}|','|S_{T5,p_{pow}}|','|S_{T6,p_{pow}}|','|S_{T7,p_{pow}}|'},{},{'b','r','y','m','g'},{},10)
% legend('1.2.1 (PID)','1.2.2.3 (SOSMC Direct)','1.2.3.2 (R-LQR with rotor failures)','1.2.4 (Mode-Independent Markovian)','1.2.5.2 (Adaptive with PIDD)')
legend('2.2.1 (PID)','2.2.2.2 (SOSMC with PIDD)','2.2.3.2 (R-LQR with rotor failures)','2.2.4 (Mode-Dependent Markovian)')
title('Total effect indices for RMS power')
%% Radar 5 - Performance
figure
meanEp = [];
varEp = [];
meanEAtt = [];
varEatt = [];
meanP = [];
varP = [];
compCost = [];
for it=1:length(controller)
    meanEp = [meanEp; abs(controller(it).bestPerformance.meanEp)];
    varEp = [varEp; abs(controller(it).bestPerformance.meanVarEp)];
    meanEAtt = [meanEAtt; abs(controller(it).bestPerformance.meanEatt)];
    varEatt = [varEatt; abs(controller(it).bestPerformance.meanVarEatt)];
    meanP = [meanP; abs(controller(it).bestPerformance.meanP)];
    varP = [varP; abs(controller(it).bestPerformance.meanVarP)];
    compCost = [compCost; abs(controller(it).bestPerformance.computationalCost)];
end
meanEp = meanEp/max(meanEp);
varEp = varEp/max(varEp);
meanEAtt = meanEAtt/max(meanEAtt);
varEatt = varEatt/max(varEatt);
meanP = meanP/max(meanP);
varP = varP/max(varP);
compCost = compCost/max(compCost);
% radarPlot2([ST1_ss ST2_ss ST3_ss ST4_ss ST5_ss ST6_ss ST7_ss]','o-','LineWidth', 3, 'MarkerFaceColor', [0,0,0])
radarplot([meanEp varEp meanEAtt varEatt meanP varP compCost],{'E(e_{p})','E(V(e_{p}))','E(e_{att})','E(V(e_{att}))','E(P)','E(V(P))','J_c'},{},{'b','r','y','m','g'},{},10)
% legend('1.2.1 (PID)','1.2.2.3 (SOSMC Direct)','1.2.3.2 (R-LQR with rotor failures)','1.2.4 (Mode-Independent Markovian)','1.2.5.2 (Adaptive with PIDD)')
legend('2.2.1 (PID)','2.2.2.2 (SOSMC with PIDD)','2.2.3.2 (R-LQR with rotor failures)','2.2.4 (Mode-Dependent Markovian)')
title('Performance metrics')
