source ~/phoenixZ/racer_ws/devel/setup.bash

roslaunch exploration_manager swarm_exploration.launch &

sleep 2

roslaunch exploration_manager rviz.launch &