# RSSA
This repository contains the code for MECC 2021 implementation of the Robust safe set algorithm (RSSA) and the numerical studies found there in.

C. Noren, W. Zhao, and C. Liu, "Safe Adaptation with Multiplicative Uncertainties Using Robust Safe Set Algorithm", in Modeling Estimation and Controls Conference. IFAC, 2021
[Preprint](https://arxiv.org/abs/1912.09095)

Please also see the foundation for this work in:

C. Liu, and M. Tomizuka, "Control in a safe set: addressing safety in human-robot interactions", in Dynamic Systems and Control Conference. ASME, 2014, p. V003T42A003. [Preprint](http://web.stanford.edu/~cliuliu/files/dscc14.pdf)

C. Liu, and M. Tomizuka, "Modeling and controller design of cooperative robots in workspace sharing human-robot assembly teams", in IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2014, pp. 1386 ? 1391. [Preprint](http://web.stanford.edu/~cliuliu/files/iros14.pdf)

## SCARA_Interactive

run RSSA.m to play with a two-link SCARA robot arm or see the numerical studies

There are multiple modes that one can work with, the default configuration in all files is for the RSSA Algortihm with a Robust Safety Index

You may change the setting(s) in RSSA.m to switch between the following modes

### Interactive Session:

> Make certain that use_cursor = 1 is selected. By default it is. 
>
>Place cursor on top of the blue circles for 3 seconds for calibration (one circle will be at the center, the other will be at the up right corner)
>
> You control the blue agent to pick the blue dots. The SCARA robot is in red. It tries to pick the red dots.
>
> The robot behavior is specified in robust_robotmove.m, which is by default set to implement RSSA
>

Output should look something like:
![](https://github.com/changliuliu/SafeSetAlgorithm/blob/master/SCARA_Interactive/outcome.gif)

### Numerical Studies
To switch to the numerical studies run in the MECC 2021 Paper, consider using the different configuration options in RSSA.m

Option "standstill_flag" will hold the system in the top right corner and corresponds to the first "NO OBSTACLE" Case reported in the manuscript

Different pre-recorded trajectories that were used in the experiements are provided as: "test.mat","test2.mat","test5.mat"
For the different trajectories used in the paper, the trials correspond as follows:

Trial 1: use_cursor=0, standstill_flag=0, noviol_flag=1
Trial 2: use_cursor=0, standstill_flag=0, noviol_flag=2
Trial 3: use_cursor=0, standstill_flag=0, noviol_flag=5

Additionally Trajectories are also provided for more review. These are the additional "test[n].mat" where n = {3,4}. Note that Trial 3 requires additional tuning for RSSA beyond the standard parameters used in the paper. Try kd (k1) = 10 and kphir (kxi) = 200 to avoid collisions.

Futhermore, the actual datasets themselves collected during the runs (including all parameters collected and stored during the simulation) are provided in .mat files for convenience and use in validation:

You can read these files as such:

(Example for Method 4 in the Paper, Trial 3)
load('group3_safetytrial3_str_phir.mat')

This will load the .mat file for Method 4, Trial 3. Here, Method 4 uses RSSA (with g_star, "str" for short) and the Robust Safety Index (Phir)

## Tips and Tricks:

To select between the different methods, there are a number of steps. Default parameters should have the system with RSSA and Robust Safety Index by default

IMPORTANT: Default Parameters for sim should recreate M4, Trial 3 if not used in interactive mode.

Note:

Changing whether the estimates for RSSA are used shows up in Line 75 (hat) vs. 77 (str)
Additionally, one needs to ensure that Line 141 is uncommented and being used for the safety control
Furthermore, ensure that Line 104 is uncommented 

Lines 128-136 all correspond to the regular implementation of the Safe Set Algorithm as discussed in the paper.

Line(s) 86-88 all correspond to tuning the Robust Safety Index
Selecting whether to use the Robust Safety Index occurs on both Lines 96 (vs. 93) and 115 (vs. 113)
