Run testEKF.m to run the filter for estimation of dynamic quantities defining the pose of the foot and the foot is wrenches subjected to.
There are options that can be set in this script. It is an offline estimation procedure over the collected datasets for different experiments.
The datasets are available in the folder robotData.


calibIMU is used to compute the existing misalignment between the IMU sensor and the body.

The scripts use iDynTree library's Matlab bindings as a dependency. This can be installed using codyco-superbuild's iDynTree module.
