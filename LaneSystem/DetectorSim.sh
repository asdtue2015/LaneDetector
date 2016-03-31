# This file runs the lane and face sim the database

# Author: Li Xuanpeng <li_xuanpeng@esiee-amiens.fr>
# Date: 07/10/2013

# Database
# 10-07-2013_18h30m21s: 1840 ~ 18000(lane), 2028 ~ 20093(face)
# 16-03-2014_16h37m38s: 1 ~ 15015(lane), 75 ~ 15555(face)
# 22-03-2014_13h05m12s: 1 ~ 18468(lane), 11 ~ 18533(face)

# command
#LANE_DETECTOR, StartFrame, EndFrame, YAW_ANGLE for HK, PITCH_ANGLE for HK, IPM-P (1) or HK(2), Coef_thetaMax
command1="./LaneDetector64 1 1 611 0.0 0.1 2 0.5 " 
echo $command1

#run
$command1 #&
# $command2 #& $command3
