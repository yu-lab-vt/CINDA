function [file_name, dat_in] = writeCirculationInputFile(dat_in, n_nodes, folder_path, name_appendix)
% write the input (.inp) file for cs2 func
% dat_in: arcs with tail, head, cost (minCapacity=0, maxCapacity=1 is fixed)
% maxCost = abs(observationCost)*1e5;
% infIdx = dat_in(:,3)>maxCost;
% dat_in(infIdx,:) = [];

if nargin < 4
    name_appendix = '';
else
    name_appendix = ['_',name_appendix];
end
dat_in(isinf(dat_in(:,3)),:) = [];
dat_in(:,3) = floor(dat_in(:,3)*1e7); % only allow integer cost
dat_in(dat_in(:,2) == n_nodes, 2) = 1; % make circles by merging src and sink

% head of summary
fileID = fopen(fullfile(folder_path,'summary.txt'),'w');
% data explanation
fprintf(fileID,'p min %d %d\n', n_nodes-1, size(dat_in,1)); % n_nodes no longer exist

fprintf(fileID,'c min-cost circulation problem with %d nodes and %d arcs\n', n_nodes-1, size(dat_in,1));
fprintf(fileID,'n 1 %10ld\n', 0); % no imbalanced node exist
fprintf(fileID,'c supply of 1 at node %10ld\n', 0);

fprintf(fileID,'c arc list follows\n');
fprintf(fileID,'c arc has <tail> <head> <capacity l.b.> <capacity u.b> <cost>\n');
fclose(fileID);

fileID = fopen(fullfile(folder_path, 'arcs.txt'),'w');
for i=1:size(dat_in,1)
    fprintf(fileID,'a %d %d 0 1  %d\n', dat_in(i,1), dat_in(i,2), dat_in(i,3));
end
fclose(fileID);

file_name = ['input_circulation_',num2str(n_nodes),'_', ...
    num2str(size(dat_in,1)), name_appendix,'.txt'];

file_name = fullfile(folder_path, file_name);

system(['cat ', fullfile(folder_path,'summary.txt'), ' ',...
    fullfile(folder_path, 'arcs.txt') ' > ', file_name]);

delete(fullfile(folder_path,'summary.txt'));
delete(fullfile(folder_path, 'arcs.txt'));
end