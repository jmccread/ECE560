tdat = stepinfo(x(:,3), t, pi); 
sdat = stepinfo(x(:,1), t, 0);
close all;

tabledata = {' ' 'Settling Time' 'Rise Time' 'Settling Max'; ...
             'theta:' tdat.SettlingTime tdat.RiseTime tdat.SettlingMax; ...
             ' ' 'Settling Time' 'Peak Time' 'Peak Input';
             's:' sdat.SettlingTime sdat.PeakTime sdat.Peak}; 
f=figure(4)
% create the data
% Create the column and row names in cell arrays 
cnames = {'1','2','3','4'};
rnames = {'1','2','3','4'};
% Create the uitable
table = uitable(f,'Data',tabledata,...
            'ColumnName',cnames,... 
            'RowName',rnames,...
            'ColumnWidth',{180}, ...
            'FontSize', 16, ...
            'FontWeight', 'bold'); 
subplot(2,2,[3,4])
pos = get(subplot(2,2,[3,4]),'position');
delete(subplot(2,2,[3,4]))
set(table,'units','normalized')
set(table,'position',pos)