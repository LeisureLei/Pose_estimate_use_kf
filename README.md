# Pose_estimate_use_kf
Using KF to estimate robot pose

## Instruction
The Kalman filter is an optimal estimator for the case where the process and measurement noise are zero-mean Gaussian noise. The filter has two steps. The first is a prediction of the state based on the previous state and the inputs that were applied (According to the book Robotics, Vision and Control by Peter Corke). There are extensive applications for KF. For example, we can apply KF to deal with pictures to make pictures clearer. 

There is a robot always moving to a point by the information sensors giving. The problem is that the measurements by sensor are not accurate enough. That is to say, robot will not move to the point it planned. Thus, we apply Kalman Filter here to get a more accurate point compared with measurements (We assume that the measurement noise are zero-mean Gaussian noise ).

## Run
Python codes are based on python 2.7.14 version.
Also,if you want to run this program,you should install matplotlib.
After finishing these two steps,just open the file named "kalmanfilter.py" and click "run" button.
Attention : Python file should run under path name in English rather than Chinese.

## Results
![result](https://github.com/LeisureLei/Pose_estimate_use_kf/blob/master/result.png)
