function [trajectories, costs] = mcc4mot(detection_arcs,transition_arcs)
% The min-cost circulation formulation based MAP solver for multi-object
% tracking.
% INPUT: 
% Assuming we have n detections in the video, then
% detection_arcs: a n x 4 matrix, each row corresponds to a detection in the
% form of [detection_id, C_i^en, C_i^ex, C_i];
% transition_arcs: a m x 3 matrix, each row corresponds to a transition arc
% in the form of [detection_id_i, detection_id_j, C_i,j]
% NOTE that the id should be unique and in the range of 1 to n. Detailed 
% defintion can be found in the user manual.

% OUTPUT:
% trajectories: cells containing the linking results; each cell contains a
% set of ordered detection ids, which indicate a trajectory
% costs: costs of these trajectories

if ~isempty(find(detection_arcs(:,1) < 1, 1))
    error('Id should be positive integers from 1 to n.');
end

n = size(detection_arcs,1); 

m = n*3+size(transition_arcs,1);

arcs = zeros(m,3); % with the dummy node 

pre_id = detection_arcs(:,1) * 2;
post_id = detection_arcs(:,1) * 2 + 1;
% entering arcs
arcs(1:n,1) = 1; % tail: dummy node
arcs(1:n,2) = pre_id; % head 
arcs(1:n,3) = detection_arcs(:,2);

% exiting arcs
arcs(n+1:2*n,1) = post_id; % tail: dummy node
arcs(n+1:2*n,2) = 1; % head 
arcs(n+1:2*n,3) = detection_arcs(:,3);


% detection confidence
arcs(2*n+1:3*n,1) = pre_id; % tail: dummy node
arcs(2*n+1:3*n,2) = post_id; % head 
arcs(2*n+1:3*n,3) = detection_arcs(:,4);

% transition arcs
arcs(3*n+1:end,1) = transition_arcs(:,1)*2 + 1; % post nodes
arcs(3*n+1:end,2) = transition_arcs(:,2)*2; % pre nodes
arcs(3*n+1:end,3) = transition_arcs(:,3);

tail = arcs(:,1)';
head = arcs(:,2)';
cost = arcs(:,3)';

% check inf arcs
in_valid_arcs = find(isinf(abs(cost)));
if ~isempty(in_valid_arcs)
    warning('There are arcs with infinity cost. We will remove them first.');
    tail(in_valid_arcs) = [];
    head(in_valid_arcs) = [];
    cost(in_valid_arcs) = [];
    m = length(cost);
    n = max(max(tail), max(head));
end

it_flag = false;
if ~isempty(find(cost > floor(cost), 1))
    it_flag = true;
    cost = round(cost * 1e7); % assume integer cost
end
low = zeros(1,m);
acap = ones(1,m);

num_node = 2*n+1;
num_arc = m ;
scale = 12;
excess_node = 1;
excess_flow = 0;

% num_node = 2*n+2;
% num_arc = m + 1;
% tail = [tail, 2*n+2];
% head = [head, 1];
% cost = [cost, 0];
% low = [low, 0];
% acap = [acap, n];
% call the manipulated cs2 function for min-cost circulation
tic;
[cost_all,~,~,~, track_vec] = cinda_mex(scale, num_node, num_arc, excess_node, excess_flow, tail, head, low, acap, cost);
toc;
% handle the null set
if cost_all >= 0 || isempty(track_vec)
    warning(['Null trajectory set! Please check the cost design. ',...
        'Note that high detection confidence corresponds to negative arc cost.']);
    costs = 0;
    trajectories = {};
    return;
end

locs = find(track_vec<=0);
if length(track_vec) > locs(end)
    track_vec = track_vec(1:locs(end));
end
costs = track_vec(locs);
track_len = locs(2:end) - locs(1:end-1) - 1;
track_len = [locs(1)-1; track_len]/2;
track_vec(locs) = [];
track_vec = track_vec(1:2:end)/2;
trajectories = mat2cell(track_vec, track_len,1);

if it_flag
    costs = costs / 1e7;
end

%disp(sum(costs));
