function cost_s = sanity_check(det_mat, tran_mat, trajectories)

cost_s = zeros(numel(trajectories), 1);
for i=1:numel(trajectories)
    disp(i);
    cost_s(i) = cost_s(i) + sum(det_mat(det_mat(:,1)==trajectories{i}(1),2));
    cost_s(i) = cost_s(i) + sum(det_mat(det_mat(:,1)==trajectories{i}(end),3));
    for j=1:length(trajectories{i})
        cost_s(i) = cost_s(i) + sum(det_mat(det_mat(:,1)==trajectories{i}(j),4));
    end
    for j=1:length(trajectories{i}) - 1
        loc = tran_mat(:,1) == trajectories{i}(j) & tran_mat(:,2) == trajectories{i}(j+1);
        cost_s(i) = cost_s(i) + tran_mat(loc,3);
    end
end
end