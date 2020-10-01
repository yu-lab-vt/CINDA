addpath('src_matlab/', 'src_c/');
%% load affinity scores
d_m = dlmread('sample/detecton_mat.txt');
t_m = dlmread('sample/transition_mat.txt');

%% call mcc4mot
[trajectories, costs] = mcc4mot(d_m,t_m);

cost_s = sanity_check(d_m, t_m, trajectories);

