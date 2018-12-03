# Tetris Player

## Plan
- Develop a piece-recognition node.
	- This node will detect the shapes of a given set of Tetris pieces placed in a starting area using Baxter’s cameras.
- Develop a Tetris solver.
	- Given a set of Tetris pieces, the solver will return a complete placement for them in the form of an array representing the board. This will be our solution configuration.
- Develop a piece-placing node.
	- This node will use Baxter’s suction end effector to place the pieces in the solution configuration using closed-loop control.

## Getting Started

First-time setup:
```sh
$ pip2 install --user -r requirements.txt
$ echo 'export PATH=$PATH:~/.local/bin' >> ~/.bashrc
$ echo 'export PYTHONPATH=$PYTHONPATH:~/.local/share/python2.7/site-packages' >> ~/.bashrc
$ source ~/.bashrc
```

For connecting to a robot (need to run every session):
```sh
$ export ROBOT_NAME=[name]
$ ./baxter.sh "$ROBOT_NAME".local
$ rosrun baxter_tools enable_robot.py -e
```

### For pick-and-place
- `roslaunch baxter_moveit_gui_noexec.launch` for RViz GUI path planning
- `./baxter.sh [name].local` which is either `asimov, ayrton, archytas, ada,` or `alan`
1. Enable Baxter by running `rosrun baxter_tools enable_robot.py -e`
2. Start the Baxter trajectory controller by running `rosrun baxter_interface joint_trajectory_action_server.py`
3. Start MoveIt for Baxter by running `roslaunch baxter_moveit_config demo_baxter.launch right_electric_gripper:=true left_electric_gripper:=true`
omitting the last argument if the robot lacks (a) gripper(s).  If this fails on Baxter, try `roslaunch baxter_moveit_config baxter_grippers.launch`
MoveIt is now ready to compute and execute trajectories on the robot

- `rosrun tetris path_test.py`
- `rosrun tf tf_echo base reference/right_gripper`
- `rosrun baxter_tools tuck_arms.py -u`	

## TODO
1. Detect what pieces there are ?
2. Input the pieces into the solver, get solution !
3. From solution figure out location (coordinates) to place pieces !
4. Detect where to pick up the pieces for each piece
5. Place them in their correct positions (how to get accuracy right?)