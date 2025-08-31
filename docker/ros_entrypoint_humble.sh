#!/bin/bash
## source ros humble
source /opt/ros/humble/setup.bash
## source project if exists (ROS 2 colcon)
if [ -f ~/mppi_swerve_drive_ros/install/setup.bash ]; then
    source ~/mppi_swerve_drive_ros/install/setup.bash
fi
## add commands above to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "[ -f ~/mppi_swerve_drive_ros/install/setup.bash ] && source ~/mppi_swerve_drive_ros/install/setup.bash" >> ~/.bashrc
echo "source /etc/bash_completion" >> ~/.bashrc
## run command
exec "$@"

