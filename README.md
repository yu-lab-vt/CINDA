# Welcome to CINDA
CINDA (**Ci**rculation **N**etwork based **D**ata-**a**ssociation) is a minimum-cost circulation framework for solving the global data association problem, which plays a key role in the tracking-by-detection paradigm of multi-object tracking (MOT). CINDA maintains the same optimal solution as the previously widely used minimum-cost flow framework, while enjoys both a better theoretical complexity bound and orders of practical efficiency improvement. The improved computational efficiency is expected to enable more sophisticated tracking framework and yields better tracking accuracy.

![Overview of CINDA](img/fig1_mot_min_cost_v3.png)
Overview of CINDA and its comparison with minmum-cost flow-based framework.
# Supports to Python and MATLAB
CINDA was implemented using C based on the efficient implementation of cost-scaling algorithm [1]. Interfaces for Python and MATLAB are also provided respectively, on which the efficiency is also guaranteed. Try CINDA now!

Any problem? CINDA does not work on your data? Please open an issue. We are happy to help!

# Efficiency comparison
## Theoretical bounds
![Theoretical bound of CINDA](img/theoretical_bound.png)
## Pracitical comparisons
### On natural image benchmars
![kitti-car dataset](img/kitti_car.png)
![CVPR19 ETHZ dataset](img/cvpr_ethz.png)
### On microscopy imaging data for particle and cell tracking
![Particle and cell tracking](img/theoretical_bound.png)

# Case studies using CINDA
## Identity inference using more history frames retrieve identities of occluded objects
![kitti-car illustration](img/KITTI_track_res.png)
## Enable us to iteratively refine tracking results on larger scale data
![Embryo_res](img/Embryo_res.png)

[1].Goldberg, A. V. (1997). An efficient implementation of a scaling minimum-cost flow algorithm. Journal of algorithms, 22(1), 1-29.
