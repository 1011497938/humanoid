set -o errexit
set -o verbose

export ZJUDANCER_ROBOTID=5
export ZJUDANCER_GUI=1
export ZJUDANCER_GPU=0
export ZJUDANCER_SIMULATION=true

# active ros
source /opt/ros/kinetic/setup.bash

# build humanoid-lib
cd $HOME/humanoid-lib
catkin_make -j8
source $HOME/humanoid-lib/devel/setup.bash

# build humanoid
cd $HOME/humanoid
catkin_make -j8
source $HOME/humanoid/devel/setup.zsh
