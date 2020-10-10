addpath('src_matlab/', 'src_c/');

load('graph_detection_shared.mat','arcs');

n_nodes = max(max(arcs(:,1)), max(arcs(:,2)));

dat_in = arcs;
dat_in(isinf(dat_in(:,3)),:) = []; % remove inf arcs
dat_in(:,3) = floor(dat_in(:,3)*1e7); % change to integer cost
dat_in(dat_in(:,2) == n_nodes, 2) = 1; % make circles by merging src and sink

num_node = n_nodes - 1;
num_arc = size(dat_in,1);


excess_node = 1;
excess_flow = 0;

low = zeros(1,num_arc);
acap = ones(1,num_arc);

tail = dat_in(:,1)';
head = dat_in(:,2)';
cost = dat_in(:,3)';

scale = 12;
tic;
[cost_all,~,~,~, track_vec] = cinda_mex(scale, num_node, num_arc, ...
    excess_node, excess_flow, tail, head, low, acap, cost);
toc;


%% min-cost flow
n_nodes = max(max(arcs(:,1)), max(arcs(:,2)));

dat_in = arcs;
dat_in(isinf(dat_in(:,3)),:) = []; % remove inf arcs
dat_in(:,3) = floor(dat_in(:,3)*1e7); % change to integer cost

num_node = n_nodes;
num_arc = size(dat_in,1);


excess_node = [1 n_nodes];
excess_flow = [100 -100];

low = zeros(1,num_arc);
acap = ones(1,num_arc);

tail = dat_in(:,1)';
head = dat_in(:,2)';
cost = dat_in(:,3)';

scale = 12;
tic;
[cost_all,~,~,~, track_vec] = cinda_mex(scale, num_node, num_arc, ...
    excess_node, excess_flow, tail, head, low, acap, cost);
toc;




