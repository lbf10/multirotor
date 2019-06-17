%% Generate table
%% Choose files
filename = '1';
figure();
controller = [];
while ~strcmp(filename,'')
    [filename, pathname] = uigetfile('*.mat')
    try
        aux = load([pathname,'/',filename]);
        controller = [controller aux.controller];
    catch
        filename = '';
    end
end
for it=1:length(controller)
    disp(strjoin([num2str(it),' - ',controller(it).name]));
end
reply = input('Select controllers to plot tables of (numbers between commas):','s');
indexes = strsplit(reply,',');

table1 = [0 0 0 0 0];
table2 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
table3 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
table4 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
table5 = [0 0 0 0 0 0 0 0];
for it=indexes
    index = str2num(it{1});
    printCase = controller(index);
    table1 = [table1; 0 printCase.perfectSuccessRate printCase.ssRobustness.variance printCase.epRobustness.variance printCase.PRobustness.variance];
end



