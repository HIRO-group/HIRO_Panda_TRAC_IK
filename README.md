# Franka Panda TracIK Inverse Kinematics Solver

The Franka Panda Inverse Kinematics Solver Package is a ROS package developed in the HIRO Lab at CU Boulder. Users, when running the package, publish to a topic with a `geometry_msgs/Pose` message. Trac_IK, an IK solver with a solve rate of 99.88% for the Franka Emika Panda, is used to compute the 7 resulting joint angles for the arm. From there, the user can specify the trajectory method in which the robot moves from one position to another. Additionally, there is an accompanying Jupyter Notebook (`src/splines.ipynb`) for Catmull-Rom Splines, which is the best trajectory method for the Panda in this repo.

## Installation, Building, and Usage

### Requires

* `trac_ik`: The inverse kinematics package compatible with Franka Panda (and a variety of other robot arms)

### Steps

1. Go into the `src/` folder for your catkin workspace.
2. `git clone https://bitbucket.org/traclabs/trac_ik.git`
3. `git clone <this-repo-name-here>`

### Building

1. Go to the root of the workspace (if you're in `src`, do `cd ..`).
2. `catkin build <package-name>` or `catkin_make`

## User Information

1. In order to run through the Inverse Kinematics, once the build command succeeds, to see IK in action:

2. Run `source/devel.bash` from the root of your workspace.

3. Run `roslaunch hiro_panda_ik hiro_panda_ik_controller.launch robot_ip:=<robot-url> trajectory_method:=<1,2,3 or 4>`

4. Note: The trajectory method is the method which is used to move the arm after the inverse kinematics are calculated. The options are:
    * **1**: Constant velocity to each point (default option when `trajectory_method` is not provided)
    * **2**: Increase velocity at a constant rate, then maintain max rate, then decrease at constant rate once near end of trajectory ('trapezoid' method)
    * **3**: Catmull-Rom splines where velocity is calculated as a function of position (smoothest and best method)

5. To publish to the topic: `rostopic pub -1 /hiro_panda/goto_pose geometry_msgs/Pose "{position:{ x: 0.4, y: 0.4, z: 0.5}, orientation:{ x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"`

6. The above example will not changed the orientation of the arm; it will move it to the desired 3-d coordinates. Make sure that the orientation (quarternion) normalizes to 1.
