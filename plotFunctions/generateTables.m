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


table1 = [0 0 0 0 0 0];
table2 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
table3 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
table4 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
table5 = [0 0 0 0 0 0 0 0];
for it=1:length(controller)
    printCase = controller(it);
    table1 = [table1; 0 printCase.perfectSuccessRate printCase.meanSuccessRate printCase.ssRobustness.variance printCase.epRobustness.variance printCase.PRobustness.variance];
    table2 = [table2; 0 printCase.ssRobustness.STf1 printCase.ssRobustness.STf2 ...
                        printCase.ssRobustness.STf3 printCase.ssRobustness.STf4 ...
                        printCase.ssRobustness.STf5 printCase.ssRobustness.STf6 ...
                        printCase.ssRobustness.STf7 printCase.ssRobustness.Sf1 ...
                        printCase.ssRobustness.Sf2 printCase.ssRobustness.Sf3 ...
                        printCase.ssRobustness.Sf4 printCase.ssRobustness.Sf5 ...
                        printCase.ssRobustness.Sf6 printCase.ssRobustness.Sf7];
    table3 = [table3; 0 printCase.epRobustness.STf1 printCase.epRobustness.STf2 ...
                        printCase.epRobustness.STf3 printCase.epRobustness.STf4 ...
                        printCase.epRobustness.STf5 printCase.epRobustness.STf6 ...
                        printCase.epRobustness.STf7 printCase.epRobustness.Sf1 ...
                        printCase.epRobustness.Sf2 printCase.epRobustness.Sf3 ...
                        printCase.epRobustness.Sf4 printCase.epRobustness.Sf5 ...
                        printCase.epRobustness.Sf6 printCase.epRobustness.Sf7];
    table4 = [table4; 0 printCase.PRobustness.STf1 printCase.PRobustness.STf2 ...
                        printCase.PRobustness.STf3 printCase.PRobustness.STf4 ...
                        printCase.PRobustness.STf5 printCase.PRobustness.STf6 ...
                        printCase.PRobustness.STf7 printCase.PRobustness.Sf1 ...
                        printCase.PRobustness.Sf2 printCase.PRobustness.Sf3 ...
                        printCase.PRobustness.Sf4 printCase.PRobustness.Sf5 ...
                        printCase.PRobustness.Sf6 printCase.PRobustness.Sf7];
    table5 = [table5; 0 printCase.bestPerformance.meanEp printCase.bestPerformance.meanVarEp ...
                        printCase.bestPerformance.meanEatt printCase.bestPerformance.meanVarEatt ...
                        printCase.bestPerformance.meanP printCase.bestPerformance.meanVarP ...
                        printCase.bestPerformance.computationalCost];
end
latexTable1 = latex(vpa(table1,4));
latexTable2 = latex(vpa(table2,2));
latexTable3 = latex(vpa(table3,2));
latexTable4 = latex(vpa(table4,2));
latexTable5 = latex(vpa(table5,4));
