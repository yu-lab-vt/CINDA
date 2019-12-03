# Welcome to CINDA
CINDA (**CI**rculation **N**etwork based **D**ata-**A**ssociation) is a minimum-cost circulation framework for solving the global data association problem, which plays a key role in the tracking-by-detection paradigm of multi-object tracking (MOT). CINDA maintains the same optimal solution as the previously widely used minimum-cost flow framework, while enjoys both a better theoretical complexity bound and orders of practical efficiency improvement. The improved computational efficiency is expected to enable more sophisticated tracking framework and yields better tracking accuracy.
## Overview of CINDA and its comparison with minmum-cost flow-based framework
![Overview of CINDA](img/fig1_mot_min_cost_v3.png)

# Supports to Python and MATLAB
CINDA was implemented using C based on the efficient implementation of cost-scaling algorithm[1]. Interfaces for Python and MATLAB are also provided respectively, on which the efficiency is also guaranteed. Try CINDA now!

Any problem? CINDA does not work on your data? Please open an issue. We are happy to help!

# Efficiency comparison
## Theoretical bounds compared with minimum-cost flow-based methods
<p align="center">
  <img height="170" src="img/theoretical_bound.png">
</p>

## Pracitical efficiency comparisons with minimum-cost flow-based methods

### Experiments on natural image benchmars
<p align="center">
  <img height="330" src="img/kitti_car.png">
</p>
<p align="center">
  <img height="270" src="img/cvpr_ethz.png">
</p>

### Experiments on microscopy imaging data for particle and cell tracking
<p align="center">
  <img height="120" src="img/PTC_Embryo.png">
</p>

# Case studies using CINDA
1. CINDA makes it possible to do identity inference using more history frames, which retrieve identities of occluded objects
![kitti-car illustration](img/KITTI_track_res.png)
2. CINDA enable us to iteratively refine tracking results on larger scale data (see Table 4)
<p align="center">
  <img height="120" src="img/Embryo_res.png">
</p>

# Reference
## Please cite our paper if you find the code useful for your research.
Congchao Wang, Yizhi Wang, Guoqiang Yu, [Efficient Global Multi-object Tracking Under Minimum-cost Circulation Framework](https://arxiv.org/abs/1911.00796), 	arXiv:1911.00796.
```
@article{cinda_mot,
  title={Efficient Global Multi-object Tracking Under Minimum-cost Circulation Framework},
  author={Wang, Congchao and Wang, Yizhi and Yu, Guoqiang},
  journal={arXiv preprint arXiv:1911.00796},
  year={2019}
}
```
## cs2
[1].Goldberg, A. V. (1997). An efficient implementation of a scaling minimum-cost flow algorithm. Journal of algorithms, 22(1), 1-29.
## Affinity models and identity inference solvers
[8] S. Sharma, J. A. Ansari, J. K. Murthy, and K. M. Krishna, “Beyond
pixels: Leveraging geometry and shape cues for online multiobject tracking,” in 2018 IEEE International Conference on Robotics
and Automation (ICRA). IEEE, 2018, pp. 3508–3515.

[12] J. Berclaz, F. Fleuret, E. Turetken, and P. Fua, “Multiple object
tracking using k-shortest paths optimization,” IEEE transactions
on pattern analysis and machine intelligence, vol. 33, no. 9, pp. 1806–
1819, 2011.

[13] L. Zhang, Y. Li, and R. Nevatia, “Global data association for multiobject tracking using network flows,” in 2008 IEEE Conference on
Computer Vision and Pattern Recognition. IEEE, 2008, pp. 1–8.

[14] P. Lenz, A. Geiger, and R. Urtasun, “Followme: Efficient online
min-cost flow tracking with bounded memory and computation,”
in Proceedings of the IEEE International Conference on Computer
Vision, 2015, pp. 4364–4372.

[15] H. Pirsiavash, D. Ramanan, and C. C. Fowlkes, “Globally-optimal
greedy algorithms for tracking a variable number of objects,” in
CVPR 2011. IEEE, 2011, pp. 1201–1208.

[16] A. V. Goldberg, S. Hed, H. Kaplan, and R. E. Tarjan, “Minimum-cost flows in unit-capacity networks,” Theory of Computing Systems,
vol. 61, no. 4, pp. 987–1010, 2017.

