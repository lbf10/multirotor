clc
addpath('../multiControl/')
addpath('../multiControl/utils')
warning('off','all')

if  isempty(gcp('nocreate'))
    poolobj = parpool('ClusterPandora',68);
%     poolobj = parpool('local',2);
end
addAttachedFiles(poolobj,{'evaluatePassiveController.m','paramsToMultirotor.m','../multiControl/'})

% filename = 'Results/2. Two cases: 4 sequential failures with and without 3kg load/1. PID_PassiveNMAC_PassiveNMAC/1a Tentativa/PID_Passive NMAC_Passive NMAC_30-Sep-2018 15:23:09_result.mat';
% evaluatePassiveController(filename)
filename = 'Results/2. Two cases: 4 sequential failures with and without 3kg load/2. RLQ-RPassive_PassiveNMAC_PassiveNMAC/1a Tentativa/RLQ-R Passive_Passive NMAC_Passive NMAC_01-Oct-2018 23:02:15_result.mat';
evaluatePassiveController(filename)
filename = 'Results/2. Two cases: 4 sequential failures with and without 3kg load/3. RLQ-RPassiveModified_PassiveNMAC_PassiveNMAC/1a Tentativa (Melhor)/RLQ-R Passive Modified_Passive NMAC_Passive NMAC_02-Oct-2018 22:33:01_result.mat';
evaluatePassiveController(filename)
filename = 'Results/2. Two cases: 4 sequential failures with and without 3kg load/4. RLQ-RPassiveModifiedWithPIDD_PassiveNMAC_PassiveNMAC/1a Tentativa/RLQ-R Passive Modified with PIDD_Passive NMAC_Passive NMAC_06-Oct-2018 11:57:20_result.mat';
evaluatePassiveController(filename)
filename = 'Results/2. Two cases: 4 sequential failures with and without 3kg load/5. SOSMCPassive_PassiveNMAC_PassiveNMAC/5a Tentativa (Melhor - depois da correcao do SOSMC)/SOSMC Passive_Passive NMAC_Passive NMAC_09-Apr-2019 18:43:35_result.mat';
evaluatePassiveController(filename)
filename = 'Results/2. Two cases: 4 sequential failures with and without 3kg load/6. SOSMCPassiveWithPIDD_PassiveNMAC_PassiveNMAC/5a Tentativa (melhor)/SOSMC Passive with PIDD_Passive NMAC_Passive NMAC_12-Apr-2019 17:06:23_result.mat';
evaluatePassiveController(filename)
filename = 'Results/2. Two cases: 4 sequential failures with and without 3kg load/7. SOSMCPassiveDirect_None_PassiveNMAC/3a Tentativa (melhor)/SOSMC Passive Direct_None_Passive NMAC_14-Apr-2019 04:43:00_result.mat';
evaluatePassiveController(filename)
filename = 'Results/2. Two cases: 4 sequential failures with and without 3kg load/8. Adaptive_PassiveNMAC_PassiveNMAC/3a Tentativa (melhor)/Adaptive_Passive NMAC_Passive NMAC_15-Apr-2019 10:22:27_iterations.mat';
evaluatePassiveController(filename)
filename = 'Results/2. Two cases: 4 sequential failures with and without 3kg load/9. AdaptiveWithPIDD_PassiveNMAC_PassiveNMAC/2a Tentativa/Adaptive with PIDD_Passive NMAC_Passive NMAC_17-Apr-2019 23:47:55_iterations.mat';
evaluatePassiveController(filename)
filename = 'Results/2. Two cases: 4 sequential failures with and without 3kg load/10. AdaptiveDirect_None_PassiveNMAC/2a Tentativa/Adaptive Direct_None_Passive NMAC_23-Apr-2019 20:31:30_result.mat';
evaluatePassiveController(filename)
filename = 'Results/2. Two cases: 4 sequential failures with and without 3kg load/11. MarkovianRLQ-RPassiveModified_PassiveNMAC_PassiveNMAC/5a Tentativa/Markovian RLQ-R Passive Modified_Passive NMAC_Passive NMAC_24-Apr-2019 08:00:21_iterations.mat';
evaluatePassiveController(filename)